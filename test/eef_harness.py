"""Shared helpers of the hardware layer's vNext M2.6 tests (ADR-0022): rclpy on a private domain.

Isolation (no test may reach a robot):
- ROS domain 86, set PROGRAMMATICALLY on the test's own rclpy context (authoritative even from a
  shell that exports another ROS_DOMAIN_ID), and exported for any child process;
- ROS_LOCALHOST_ONLY=1;
- a graph guard: before the first test, a probe node polls the domain for 6 s (two default Fast DDS
  participant announcement periods; cross-process discovery took up to 2.54 s on this machine) and
  refuses to run at the first node this process did not create. After every test the graph must
  be empty again (``expect_clean``).

One pytest process at a time on domain 86; every adapter of a test is in-process (its own node,
namespace and parameters), spun by one MultiThreadedExecutor in a background thread.
"""
from __future__ import annotations

import hashlib
import os
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

import yaml

DOMAIN_ID = 86
PROBE = 'm2_6_hw_graph_guard'
HELPER_NS = '/m26hw'
DISCOVERY_S = 6.0
DATA = Path(__file__).resolve().parent / 'data' / 'm2_6_contract'

# The byte copies of the vNext M2.6 contract files (provenance: test/data/m2_6_contract/README.md).
PINS = {
    'normalize_eef.py': '9fba00c3981762974d9e916464fd670c8f83964392649c3e0854a94149290a0b',
    'eef_conformance.yaml': '873a1d582c4e3947a62026d153bd49ccb02d7b4d38e5cb3a5b4b3a8f5a27a04f',
    'eef_conformance.expected.txt': '1f6bd1d2ac2fb18865ee67a815877483537d0a325fbc9a5400cf2977710d242c',
    'eef_contract_tokens.txt': '55048ea44d466979f57c28a634295a060a79b837fe614a30c49f60572a2bd5de',
}


# ------------------------------------------------------------------------------------------------
# pinned test data (no ROS)
# ------------------------------------------------------------------------------------------------

def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pinned(name: str) -> Path:
    """The path of a pinned byte copy, after checking its sha256 (a local edit fails here)."""
    path = DATA / name
    assert sha256(path) == PINS[name], f'{name}: not the pinned bytes (sha256 {sha256(path)})'
    return path


def _flatten(tree: Dict[str, Any], prefix: str = '') -> Dict[str, Any]:
    out = {}
    for k, v in tree.items():
        name = f'{prefix}.{k}' if prefix else str(k)
        if isinstance(v, dict):
            out.update(_flatten(v, name))
        else:
            out[name] = v
    return out


def fixture_flat() -> Dict[str, Any]:
    """The conformance fixture as the flat {dotted: value} map a node sees (the test's own loader)."""
    doc = yaml.safe_load(pinned('eef_conformance.yaml').read_text(encoding='utf-8'))
    return _flatten(doc['/**']['ros__parameters'])


def device_block(device_id: str, joint_prefix: Optional[Tuple[str, str]] = None,
                 changes: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """The ``eef.<id>.*`` keys of one fixture device, optionally with its joints re-prefixed
    (``('robot_a_', 'robot_b_')`` gives robot_b's description) and keys changed (None deletes)."""
    pre = f'eef.{device_id}.'
    out = {}
    for k, v in fixture_flat().items():
        if not k.startswith(pre):
            continue
        if joint_prefix and k.endswith('.joints'):
            old, new = joint_prefix
            v = [new + j[len(old):] if j.startswith(old) else j for j in v]
        out[k] = v
    for k, v in (changes or {}).items():
        key = k if k.startswith('eef.') else pre + k
        if v is None:
            out.pop(key, None)
        else:
            out[key] = v
    return out


def load_tokens() -> Dict[str, Dict[str, str]]:
    """{group: {name: value}} of the pinned canonical token file, in file order."""
    groups: Dict[str, Dict[str, str]] = {}
    for line in pinned('eef_contract_tokens.txt').read_text(encoding='utf-8').splitlines():
        if not line or line.startswith('#'):
            continue
        group, name, value = line.split()
        assert name not in groups.setdefault(group, {}), f'duplicate token {group} {name}'
        groups[group][name] = value
    return groups


def wait_until(pred: Callable[[], bool], timeout: float, period: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(period)
    return pred()


# ------------------------------------------------------------------------------------------------
# rclpy (imported lazily: the pure tests never initialise ROS)
# ------------------------------------------------------------------------------------------------

class RosSession:
    """One rclpy context on domain 86 for the whole pytest session, behind the graph guard."""

    def __init__(self, tmp: Optional[Path] = None):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        os.environ['ROS_DOMAIN_ID'] = str(DOMAIN_ID)        # for child processes
        if tmp is not None:
            os.environ.setdefault('ROS_LOG_DIR', str(tmp / 'ros_log'))
        import rclpy
        from rclpy.node import Node
        self.rclpy = rclpy
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=DOMAIN_ID)   # programmatic: beats ROS_DOMAIN_ID
        assert self.ctx.get_domain_id() == DOMAIN_ID
        self.probe = Node(PROBE, context=self.ctx, use_global_arguments=False,
                          enable_rosout=False, start_parameter_services=False)
        deadline = time.monotonic() + DISCOVERY_S
        foreign = set()
        while not foreign and time.monotonic() < deadline:
            time.sleep(0.05)
            foreign = self.visible()
        if foreign:
            self.close()
            raise RuntimeError(
                f'M2.6 GRAPH GUARD: foreign nodes on private domain {DOMAIN_ID}: {sorted(foreign)} '
                '-- refusing to run (no test may reach a robot)')

    def visible(self) -> set:
        """Every node on the domain but the probe (hidden ros2cli daemons excluded)."""
        out = set()
        for name, ns in self.probe.get_node_names_and_namespaces():
            if name.startswith('_') or (name == PROBE and ns == '/'):
                continue
            out.add(ns.rstrip('/') + '/' + name)
        return out

    def expect_clean(self, what: str, budget: float = 10.0) -> None:
        if not wait_until(lambda: not self.visible(), budget, 0.02):
            raise RuntimeError(f'M2.6 GRAPH GUARD: before {what}: nodes left on private domain '
                               f'{DOMAIN_ID}: {sorted(self.visible())}')

    def close(self) -> None:
        self.probe.destroy_node()
        self.rclpy.shutdown(context=self.ctx)


class StateWatch:
    """A TRANSIENT_LOCAL subscription to one EndEffectorState topic, keeping every sample."""

    def __init__(self, node, topic: str, depth: int = 100):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from trainit_interfaces.msg import EndEffectorState
        self.topic = topic
        self._lock = threading.Lock()
        self.samples: List[Tuple[float, Any]] = []
        qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = node.create_subscription(EndEffectorState, topic, self._on, qos)

    def _on(self, msg) -> None:
        with self._lock:
            self.samples.append((time.monotonic(), msg))

    def count(self) -> int:
        with self._lock:
            return len(self.samples)

    def latest(self):
        with self._lock:
            return self.samples[-1][1] if self.samples else None

    def all(self) -> List[Any]:
        with self._lock:
            return [m for _, m in self.samples]

    def wait(self, pred: Callable[[Any], bool], timeout: float = 5.0, what: str = ''):
        """The first sample, newest first, that satisfies `pred` within `timeout`; else fail."""
        found = []

        def check():
            m = self.latest()
            if m is not None and pred(m):
                found.append(m)
                return True
            return False
        if not wait_until(check, timeout):
            raise AssertionError(f'{self.topic}: no sample {what} within {timeout} s; '
                                 f'last: {describe(self.latest())}')
        return found[0]


def site(msg, s: str) -> Tuple[int, int, int]:
    """(grasp, evidence, stamp ns) of site `s` in a state sample."""
    i = list(msg.sites).index(s)
    st = msg.site_stamp[i]
    return msg.site_grasp[i], msg.site_evidence[i], st.sec * 1_000_000_000 + st.nanosec


def actuator(msg, a: str):
    for entry in msg.actuators:
        if entry.actuator == a:
            return entry
    raise KeyError(a)


def ns_of(t) -> int:
    return t.sec * 1_000_000_000 + t.nanosec


def describe(msg) -> str:
    if msg is None:
        return 'none'
    acts = ', '.join(f'{a.actuator}(st={a.status} bin={a.binary} conf={a.binary_confirmed} '
                     f'cmd={a.last_command})' for a in msg.actuators)
    return (f'{msg.end_effector} status={msg.status} sites={list(msg.sites)} '
            f'grasp={list(msg.site_grasp)} evidence={list(msg.site_evidence)} [{acts}] '
            f'detail={msg.detail!r}')


class Harness:
    """One test's nodes and the executor that spins them."""

    def __init__(self, session: RosSession, threads: int = 48):
        from rclpy.executors import MultiThreadedExecutor
        self.session = session
        self.ctx = session.ctx
        self.executor = MultiThreadedExecutor(num_threads=threads, context=self.ctx)
        self.adapters: List[Any] = []
        self.nodes: List[Any] = []
        self._helpers = 0
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except Exception:  # noqa: BLE001 -- the executor stops at shutdown
            pass

    def _add(self, node, spin: bool = True):
        self.nodes.append(node)
        if spin:
            self.executor.add_node(node)
        return node

    @staticmethod
    def overrides(params: Dict[str, Any]):
        from rclpy.parameter import Parameter
        return [Parameter(k, value=v) for k, v in params.items()]

    def mock(self, namespace: str, device_id: str, block: Optional[Dict[str, Any]] = None, *,
             spin: bool = True, **params):
        """A mock adapter for one fixture device (or `block`), with extra parameters.

        ``spin=False`` keeps it out of the executor: it publishes only what its constructor does
        (the first, latched sample), and it is closed and destroyed with the others.
        """
        from trainit_hardware_layer.eef.mock_eef_adapter import MockEefAdapter
        values = {'device_id': device_id}
        values.update(block if block is not None else device_block(device_id))
        values.update(params)
        node = MockEefAdapter(context=self.ctx, namespace=namespace,
                              parameter_overrides=self.overrides(values))
        self.adapters.append(node)
        return self._add(node, spin)

    def isaac(self, namespace: str, device_id: str = 'suction_simple',
              block: Optional[Dict[str, Any]] = None, **params):
        from trainit_hardware_layer.eef.isaac_eef_adapter import IsaacEefAdapter
        values = {'device_id': device_id}
        values.update(block if block is not None else device_block(device_id))
        values.update(params)
        node = IsaacEefAdapter(context=self.ctx, namespace=namespace,
                               parameter_overrides=self.overrides(values))
        self.adapters.append(node)
        return self._add(node)

    def helper(self, name: Optional[str] = None, namespace: str = HELPER_NS, **kwargs):
        """A plain node for the test's clients, subscriptions and fakes."""
        from rclpy.node import Node
        self._helpers += 1
        kwargs.setdefault('start_parameter_services', False)
        node = Node(name or f'helper_{self._helpers}', context=self.ctx, namespace=namespace,
                    use_global_arguments=False, **kwargs)
        return self._add(node)

    @staticmethod
    def _destroy(node) -> None:
        # rclpy's destroy_node leaves waitables (action clients and servers) alone, and they keep
        # the node handle alive: destroy them first or the node lingers in the graph.
        for waitable in list(node.waitables):
            destroy = getattr(waitable, 'destroy', None)
            if destroy is not None:
                destroy()
        node.destroy_node()

    def remove(self, node) -> None:
        """Destroy one node mid-test (e.g. a fake stage going away)."""
        self.executor.remove_node(node)
        if node in self.adapters:
            node.close()
            self.adapters.remove(node)
        self._destroy(node)
        self.nodes.remove(node)

    def close(self) -> None:
        for adapter in self.adapters:
            adapter.close()                    # release every blocked endpoint first
        self.executor.shutdown(timeout_sec=10.0)
        self._thread.join(timeout=10.0)
        errors = []
        for node in reversed(self.nodes):
            try:
                self._destroy(node)
            except Exception as e:  # noqa: BLE001 -- report every failure, after trying them all
                errors.append(f'{node}: {e!r}')
        self.nodes.clear()
        self.adapters.clear()
        assert not errors, errors


def call(client, request, timeout: float = 5.0):
    """A service call through a spun client: the response, or None when none came in time."""
    if not client.wait_for_service(timeout_sec=timeout):
        raise AssertionError(f'{client.srv_name}: no server within {timeout} s')
    future = client.call_async(request)
    if not wait_until(future.done, timeout):
        client.remove_pending_request(future)
        return None
    return future.result()


def send_goal(action_client, goal, timeout: float = 5.0):
    """(accepted, goal handle) for one goal; (None, None) when no goal response came in time."""
    if not action_client.wait_for_server(timeout_sec=timeout):
        raise AssertionError('no action server within the timeout')
    future = action_client.send_goal_async(goal)
    if not wait_until(future.done, timeout):
        return None, None
    handle = future.result()
    return handle.accepted, handle


def result_of(goal_handle, timeout: float = 10.0):
    """(status, result) of an accepted goal, or (None, None) when no result came in time."""
    future = goal_handle.get_result_async()
    if not wait_until(future.done, timeout):
        return None, None
    response = future.result()
    return response.status, response.result


def graph_names(node) -> Tuple[set, set]:
    """(topics, services) visible to `node`."""
    topics = {n for n, _ in node.get_topic_names_and_types()}
    services = {n for n, _ in node.get_service_names_and_types()}
    return topics, services


def action_names(prefixes: Iterable[str]) -> Tuple[set, set]:
    """The topics and services an action server creates under each action name."""
    topics, services = set(), set()
    for p in prefixes:
        topics |= {f'{p}/_action/feedback', f'{p}/_action/status'}
        services |= {f'{p}/_action/send_goal', f'{p}/_action/cancel_goal', f'{p}/_action/get_result'}
    return topics, services
