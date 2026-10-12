"""The ROS shell both TrainIt end-effector adapters share (ADR-0022 D2, D3, D7, D8).

One adapter process serves ONE device, in the cell namespace. Every name below is RELATIVE, so it
resolves in the node's namespace (``/r1/eef/<id>/state`` under ``/r1``, ``/eef/<id>/state`` at the
root); nothing global is created unless a compatibility parameter names it.

| What | Name | Type | QoS |
|---|---|---|---|
| tool state | ``eef/<id>/state`` | trainit_interfaces/EndEffectorState | RELIABLE, TRANSIENT_LOCAL, 1 |
| binary actuator | ``eef/<id>/actuators/<a>/set_engaged`` | std_srvs/SetBool | service |
| adapter-owned joints | ``eef/<id>/joint_states`` | sensor_msgs/JointState | RELIABLE, 10 |
| scene signal (``scene_signal: adapter``) | ``eef/<id>/engaged`` | std_msgs/Bool | RELIABLE, TRANSIENT_LOCAL, 1 |

The scene signal carries the APPLIED engage (true) or release (false) of the single site's grip,
from any commander, never a grasp. Adapter-owned joints are the ``joints`` of the actuators this
adapter serves: a set_bool actuator's are an echo of its applied state (``joint_positions``),
description-compatibility state and never evidence.

Common parameters:
- ``device_id`` (string, required) and the device's ``eef.<id>.*`` block, the SAME block TMR reads
  (``contract.device_for_adapter``);
- ``publish_rate_hz`` (double, 10.0): the heartbeat rate of the state topic;
- compatibility endpoints, all OFF by default, set here and never in TMR's ``eef`` block. Each maps
  onto the grip of the device's only site (a single-site device), and is a mapping, never a rename:

  - ``compat.trigger_on`` / ``compat.trigger_off`` (strings, both or neither): the policy card's
    std_srvs/Trigger pair at its exact names (e.g. ``/suction/on``, ``/suction/off`` at the root).
    ON is an ENGAGE of the grip, OFF a RELEASE; they need a ``set_bool`` grip;
  - ``compat.pseudo_joint_topic`` / ``compat.pseudo_joint_name`` (strings, both or neither),
    ``compat.pseudo_joint_engaged`` (1.0), ``compat.pseudo_joint_released`` (0.0),
    ``compat.pseudo_joint_rate_hz`` (20.0): the robot description's pseudo-joint echo (for example
    ``/suction/joint_states`` with ``suctioncup_joint``, as the legacy bridge publishes it). It
    follows the grip's newest APPLIED command and is robot-description compatibility state for
    TF/MoveIt: never grasp or release evidence (pseudo-joint position != object attached);
  - ``compat.gripper_command_suction_action`` (string): a control_msgs/GripperCommand handle for
    MoveIt's SRDF suction group. A position >= 0.5 is an ENGAGE, below it a RELEASE; the result is
    honest (SUCCEEDED only when applied, else ABORTED). It needs a ``set_bool`` grip.

Never published by an adapter: ``/suction/attached`` (it would be a second writer of the policy
runtime's observation slot and feed the legacy ``CheckRobotState``).

Concurrency: a MultiThreadedExecutor; every service and action server sits in its own
MutuallyExclusive callback group, the timers in one more; all model access is serialised by one
lock held for short sections only. Long work (an emulated motion, an injected "no answer") runs on
its own executor thread and never blocks another endpoint. No asyncio runs inside rclpy.
"""
from __future__ import annotations

import contextlib
import sys
import threading
from typing import Dict, List, NamedTuple, Optional, Sequence, Set, Tuple, Type

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from trainit_interfaces.msg import EndEffectorState

from . import contract, core
from .contract import Refused
from .core import Binary, Capability, Command

STATE_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
LATCHED_QOS = STATE_QOS
JOINT_STATE_QOS = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)

TICK_HZ = 50.0           # the backend's periodic logic (timeouts, conditions, switches)
JOINT_STATE_HZ = 20.0    # adapter-owned joints
SUCTION_THRESHOLD = 0.5  # the GripperCommand suction handle: position >= 0.5 engages

COMPAT_KEYS = (
    'compat.trigger_on', 'compat.trigger_off',
    'compat.pseudo_joint_topic', 'compat.pseudo_joint_name', 'compat.pseudo_joint_engaged',
    'compat.pseudo_joint_released', 'compat.pseudo_joint_rate_hz',
    'compat.gripper_command_suction_action',
)
COMMON_PARAMS = ('device_id', 'publish_rate_hz', 'use_sim_time')


class Decision(NamedTuple):
    """What a backend does with one binary command."""

    apply: bool    # apply it (record it, drive the backend) or reject it
    answer: bool   # answer the commander, or never answer (an injected "no answer")
    message: str


class EefAdapterNode(Node):
    """The common shell: contract, model, state topic, SetBool actuators, scene signal, compat.

    A backend subclass sets ``ADAPTER_ROW`` and ``DEFAULT_NODE_NAME`` and implements the hooks
    marked "backend" below.
    """

    ADAPTER_ROW = ''
    DEFAULT_NODE_NAME = ''
    BACKEND_PARAMS: Tuple[str, ...] = ()
    BACKEND_PREFIXES: Tuple[str, ...] = ()

    def __init__(self, node_name: Optional[str] = None, **kwargs):
        kwargs.setdefault('allow_undeclared_parameters', True)
        kwargs.setdefault('automatically_declare_parameters_from_overrides', True)
        super().__init__(node_name or self.DEFAULT_NODE_NAME, **kwargs)
        self._lock = threading.RLock()
        self._closing = threading.Event()
        self._idle = threading.Condition()
        self._executions = 0
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._started = False
        self._timer_group = MutuallyExclusiveCallbackGroup()
        try:
            self._build()
        except BaseException:
            self._closing.set()
            try:
                self.destroy_node()
            except Exception:  # noqa: BLE001 -- the original error is the one that matters
                pass
            raise

    # ============================================================================================
    # construction
    # ============================================================================================
    def _build(self) -> None:
        device_id = self.param('device_id', str, None)
        self.device = contract.device_for_adapter(contract.params_from_node(self), device_id)
        dev = self.device
        self.end_effector = contract.identity(self.get_namespace(), dev.id)
        self.publish_rate_hz = self.param('publish_rate_hz', float, 10.0, positive=True)
        self._read_compat()
        self._check_parameter_names()
        sources, silence_ns = self._configure()                      # backend
        self.core = core.DeviceModel(dev, self.end_effector, self.ADAPTER_ROW, sources, silence_ns)
        self._gate = core.PublishGate(int(1e9 / self.publish_rate_hz))

        base = f'eef/{dev.id}'
        self._state_pub = self.create_publisher(EndEffectorState, f'{base}/state', STATE_QOS)
        self._scene_pub = None
        if dev.scene_signal == 'adapter':
            self._scene_pub = self.create_publisher(Bool, f'{base}/engaged', LATCHED_QOS)
        self._set_bool_services = {}
        for a in self.served_set_bool():
            self._set_bool_services[a] = self.create_service(
                SetBool, f'{base}/actuators/{a}/set_engaged',
                lambda req, res, _a=a: self._on_set_engaged(_a, req, res),
                callback_group=MutuallyExclusiveCallbackGroup())
        self._create_compat()
        self._create_backend()                                       # backend
        self._joint_names = self._owned_joint_names()
        self._joint_pub = None
        if self._joint_names:
            self._joint_pub = self.create_publisher(JointState, f'{base}/joint_states',
                                                    JOINT_STATE_QOS)
            self.create_timer(1.0 / JOINT_STATE_HZ, self._publish_joint_states,
                              callback_group=self._timer_group, clock=self._steady)
        self.create_timer(1.0 / TICK_HZ, self._on_tick, callback_group=self._timer_group,
                          clock=self._steady)
        self.create_timer(1.0 / self.publish_rate_hz, self._on_heartbeat,
                          callback_group=self._timer_group, clock=self._steady)
        with self._lock:
            self._maybe_start()
            self._publish_changes()
        self.get_logger().info(self._describe())

    def _read_compat(self) -> None:
        dev = self.device
        on = self.param('compat.trigger_on', str, '')
        off = self.param('compat.trigger_off', str, '')
        if bool(on) != bool(off):
            raise Refused('compat.trigger_on', 'compat.trigger_on and compat.trigger_off: both or neither')
        pj_topic = self.param('compat.pseudo_joint_topic', str, '')
        pj_name = self.param('compat.pseudo_joint_name', str, '')
        if bool(pj_topic) != bool(pj_name):
            raise Refused('compat.pseudo_joint_topic',
                          'compat.pseudo_joint_topic and compat.pseudo_joint_name: both or neither')
        gc = self.param('compat.gripper_command_suction_action', str, '')
        self.compat_trigger = (on, off) if on else None
        self.compat_pseudo_joint = None
        if pj_topic:
            self.compat_pseudo_joint = (
                pj_topic, pj_name,
                self.param('compat.pseudo_joint_engaged', float, 1.0),
                self.param('compat.pseudo_joint_released', float, 0.0),
                self.param('compat.pseudo_joint_rate_hz', float, 20.0, positive=True))
        self.compat_gripper_command = gc or None
        wanted = [k for k, v in (('compat.trigger_on', on), ('compat.pseudo_joint_topic', pj_topic),
                                 ('compat.gripper_command_suction_action', gc)) if v]
        if wanted and dev.default_site is None:
            raise Refused(wanted[0], f'a compatibility endpoint maps onto the only site of a '
                                     f'single-site device; {dev.id} has sites {list(dev.site_ids)}')
        for key in ('compat.trigger_on', 'compat.gripper_command_suction_action'):
            if key in wanted:
                grip = dev.actuators[dev.default_site.grip]
                if grip.command_interface != 'set_bool':
                    raise Refused(key, f'needs a set_bool grip; {grip.id} is {grip.command_interface}')

    def _check_parameter_names(self) -> None:
        """Unknown keys of our own namespaces are refused; any other unknown name is a warning."""
        exact = set(COMMON_PARAMS) | set(self.BACKEND_PARAMS) | {'gripper_action'} | set(
            contract.LEGACY_KEYS) | set(COMPAT_KEYS)
        own_prefixes = ('compat.',) + tuple(p + '.' for p in self.BACKEND_PREFIXES)
        for name in sorted(self.get_parameters_by_prefix('')):
            if name in exact or name.startswith(('eef.', 'qos_overrides.')):
                continue
            if name.startswith(own_prefixes):
                if not self._backend_knows(name):                    # backend
                    raise Refused(name, 'unknown key')
                continue
            self.get_logger().warn(f'parameter {name} is not read by this adapter')

    def _create_compat(self) -> None:
        if self.compat_trigger:
            on, off = self.compat_trigger
            for key, name, command in (('compat.trigger_on', on, Command.ENGAGE),
                                       ('compat.trigger_off', off, Command.RELEASE)):
                self._named(key, lambda n=name, c=command: self.create_service(
                    Trigger, n, lambda req, res, _c=c: self._on_trigger(_c, req, res),
                    callback_group=MutuallyExclusiveCallbackGroup()))
        if self.compat_pseudo_joint:
            topic, _name, _eng, _rel, rate = self.compat_pseudo_joint
            self._pj_pub = self._named('compat.pseudo_joint_topic', lambda: self.create_publisher(
                JointState, topic, JOINT_STATE_QOS))
            self.create_timer(1.0 / rate, self._publish_pseudo_joint,
                              callback_group=self._timer_group, clock=self._steady)
        if self.compat_gripper_command:
            name = self.compat_gripper_command
            self._compat_gc = self._named('compat.gripper_command_suction_action', lambda: ActionServer(
                self, GripperCommand, name, execute_callback=self._execute_suction_handle,
                goal_callback=lambda goal: GoalResponse.ACCEPT,
                handle_accepted_callback=self._accept_suction_handle,
                cancel_callback=lambda goal_handle: CancelResponse.REJECT,
                callback_group=MutuallyExclusiveCallbackGroup()))

    def _named(self, key: str, create):
        """Create an endpoint whose name comes from a parameter; a bad name is a refusal."""
        try:
            return create()
        except Exception as e:  # noqa: BLE001 -- rclpy raises one exception type per name check
            raise Refused(key, f'not a valid ROS name: {e}') from None

    def _owned_joint_names(self) -> List[str]:
        names = []
        for a in self.served_actuators():
            names.extend(self.device.actuators[a].joints)
        return names

    def _describe(self) -> str:
        dev = self.device
        acts = ', '.join(f'{a}: {dev.actuators[a].command_interface}' for a in dev.actuator_ids)
        sites = ', '.join(
            f'{s}: grip {dev.sites[s].grip}, '
            f'{"evidence " + self.core.site_source(s).name.lower() if self.core.site_source(s) else "UNAVAILABLE"}'
            for s in dev.site_ids)
        compat = []
        if self.compat_trigger:
            compat.append(f'Trigger {self.compat_trigger[0]} / {self.compat_trigger[1]}')
        if self.compat_pseudo_joint:
            compat.append(f'pseudo-joint {self.compat_pseudo_joint[1]} on {self.compat_pseudo_joint[0]}')
        if self.compat_gripper_command:
            compat.append(f'GripperCommand suction handle {self.compat_gripper_command}')
        provides = ', '.join(sorted(c.value for c in self.provided_capabilities()))
        return (f'{self.ADAPTER_ROW} adapter for {self.end_effector} (ADR-0022): actuators [{acts}]; '
                f'sites [{sites}]; provides [{provides}]; compatibility '
                f'[{"; ".join(compat) if compat else "none"}]')

    # ============================================================================================
    # parameters
    # ============================================================================================
    def param(self, name: str, kind: type, default=None, *, required: bool = False,
              positive: bool = False, non_negative: bool = False):
        """A typed parameter: an int is accepted where a float is expected; a wrong type is refused."""
        if not self.has_parameter(name):
            if required:
                raise Refused(name, 'required')
            return default
        value = contract.plain(self.get_parameter(name).value)
        if value is None:
            if required:
                raise Refused(name, 'required')
            return default
        if kind is float and isinstance(value, int) and not isinstance(value, bool):
            value = float(value)
        ok = isinstance(value, kind) and not (kind is not bool and isinstance(value, bool))
        if not ok:
            raise Refused(name, f'a {kind.__name__}')
        if positive and not value > 0:
            raise Refused(name, 'a value > 0')
        if non_negative and not value >= 0:
            raise Refused(name, 'a value >= 0')
        return value

    # ============================================================================================
    # time
    # ============================================================================================
    def now_ns(self) -> int:
        """The adapter's ROS clock (sim time with use_sim_time): the clock of every stamp."""
        return self.get_clock().now().nanoseconds

    def steady_ns(self) -> int:
        return self._steady.now().nanoseconds

    def _maybe_start(self) -> bool:
        """The initial observation, once the ROS clock runs (sim time starts at 0 until /clock)."""
        if self._started:
            return True
        now = self.now_ns()
        if now <= 0:
            return False
        self.core.start(now)                 # from now on a source that has said nothing is named
        self._initial_observation(now)                               # backend
        self._started = True
        return True

    # ============================================================================================
    # publishing (call with the lock held)
    # ============================================================================================
    def _publish_changes(self) -> None:
        sample = self._gate.offer(self.core.sample(), self.steady_ns())
        if sample is not None:
            self._state_pub.publish(sample.to_msg())

    def _on_heartbeat(self) -> None:
        if self._closing.is_set():
            return
        with self._lock:
            sample = self._gate.heartbeat(self.core.sample(), self.steady_ns())
            if sample is not None:
                self._state_pub.publish(sample.to_msg())

    def _on_tick(self) -> None:
        if self._closing.is_set():
            return
        with self._lock:
            now = self.now_ns()
            if self._maybe_start() and now > 0:
                self._tick(now)                                      # backend
                self.core.tick(now)
            self._publish_changes()

    def _publish_joint_states(self) -> None:
        if self._closing.is_set() or self._joint_pub is None:
            return
        with self._lock:
            positions = self._joint_positions()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(positions)
        msg.position = [float(positions[n]) for n in msg.name]
        self._joint_pub.publish(msg)

    def _joint_positions(self) -> Dict[str, float]:
        """Adapter-owned joints in contract order: set_bool echoes plus the backend's own joints."""
        out: Dict[str, float] = {}
        extra = self._backend_joint_positions()                      # backend
        for a in self.served_actuators():
            spec = self.device.actuators[a]
            if spec.command_interface == 'set_bool' and spec.joints:
                engaged = self.core.binary(a) == Binary.ENGAGED
                values = spec.joint_positions_engaged if engaged else spec.joint_positions_released
                out.update(zip(spec.joints, values))
            else:
                for j in spec.joints:
                    if j in extra:
                        out[j] = extra[j]
        return out

    def _publish_pseudo_joint(self) -> None:
        if self._closing.is_set():
            return
        _topic, name, engaged_value, released_value, _rate = self.compat_pseudo_joint
        with self._lock:
            engaged = self.core.last_command(self.device.default_site.grip)[0] == Command.ENGAGE
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [name]
        msg.position = [float(engaged_value if engaged else released_value)]
        self._pj_pub.publish(msg)

    # ============================================================================================
    # commands
    # ============================================================================================
    def command_binary(self, a: str, command: Command) -> Tuple[bool, str]:
        """One binary command from any commander: decide, apply or reject, publish, answer.

        Returns (success, message) for the commander. An injected "no answer" blocks THIS
        endpoint's thread until the adapter closes (the commander then has its own timeout).
        """
        if self._closing.is_set():
            return False, 'the adapter is shutting down'
        with self._lock:
            # The initial observation comes first, or it would overwrite an applied command.
            started = self._maybe_start()
            now = self.now_ns()
            if not started or now <= 0:
                decision = Decision(False, True, 'no ROS time yet (use_sim_time and no /clock)')
            else:
                decision = self._decide_binary(a, command)           # backend
            if decision.apply:
                self._drive_binary(a, command)                       # backend
                self.core.apply(a, command, now)
                self._after_applied(a, command)
            elif decision.answer:
                # A rejection is named in detail; a server that never answers tells nobody.
                self.core.reject(a, command, decision.message)
            self._publish_changes()
        if not decision.answer:
            self._closing.wait()
            return False, 'never answered (injected); the adapter is shutting down'
        return decision.apply, decision.message

    def _after_applied(self, a: str, command: Command) -> None:
        """After an APPLIED command (lock held): the scene signal and the backend's own hook."""
        site = self.device.default_site
        if (self._scene_pub is not None and site is not None and site.grip == a
                and command in (Command.ENGAGE, Command.RELEASE)):
            self._scene_pub.publish(Bool(data=command == Command.ENGAGE))
        self._on_applied(a, command)                                 # backend

    def _on_set_engaged(self, a: str, request, response):
        command = Command.ENGAGE if request.data else Command.RELEASE
        response.success, response.message = self.command_binary(a, command)
        return response

    def _on_trigger(self, command: Command, request, response):
        response.success, response.message = self.command_binary(self.device.default_site.grip,
                                                                 command)
        return response

    def _accept_suction_handle(self, goal_handle) -> None:
        self.begin_execution()
        goal_handle.execute()

    def _execute_suction_handle(self, goal_handle):
        with self.execution():
            return self._suction_handle(goal_handle)

    def _suction_handle(self, goal_handle):
        position = float(goal_handle.request.command.position)
        command = Command.ENGAGE if position >= SUCTION_THRESHOLD else Command.RELEASE
        grip = self.device.default_site.grip
        ok, _message = self.command_binary(grip, command)
        with self._lock:
            engaged = self.core.binary(grip) == Binary.ENGAGED
        result = GripperCommand.Result()
        result.position = 1.0 if engaged else 0.0   # the APPLIED state, whatever was asked
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = bool(ok)
        if ok and not self._closing.is_set():
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ============================================================================================
    # lifecycle
    # ============================================================================================
    def close(self, timeout_s: float = 5.0) -> None:
        """Release every blocked endpoint, stop the periodic work, and wait for running goals.

        Call it before the executor stops: an executor waits for its callbacks, but not for the
        tasks that execute accepted goals, and a node must not be destroyed under a running one.
        """
        self._closing.set()
        with self._idle:
            self._idle.wait_for(lambda: self._executions == 0, timeout_s)

    def begin_execution(self) -> None:
        """An accepted goal is about to be executed: ``close`` waits until ``end_execution``.

        Counted from the acceptance, before the execution task is scheduled, so that a ``close``
        can never fall between the two (``close`` runs while the executor still spins).
        """
        with self._idle:
            self._executions += 1

    def end_execution(self) -> None:
        with self._idle:
            self._executions -= 1
            self._idle.notify_all()

    @contextlib.contextmanager
    def execution(self):
        """The body of a goal execution that ``begin_execution`` announced."""
        try:
            yield
        finally:
            self.end_execution()

    @property
    def closing(self) -> bool:
        return self._closing.is_set()

    def destroy_node(self):
        """Destroy the node AND its action servers.

        rclpy's ``Node.destroy_node`` leaves waitables alone, and an ActionServer keeps the node
        handle alive: without this the node would linger in the graph after its destruction.
        """
        self._closing.set()
        for waitable in list(self.waitables):
            destroy = getattr(waitable, 'destroy', None)
            if destroy is not None:
                destroy()
        return super().destroy_node()

    def recommended_threads(self) -> int:
        """Executor threads: one per endpoint that may hold a thread, plus the timers."""
        return 6 + 2 * len(self.device.actuator_ids) + 3

    # ============================================================================================
    # backend hooks
    # ============================================================================================
    def served_actuators(self) -> Sequence[str]:
        """The actuators this adapter serves (their commands and their joints)."""
        return self.device.actuator_ids

    def served_set_bool(self) -> List[str]:
        return [a for a in self.served_actuators()
                if self.device.actuators[a].command_interface == 'set_bool']

    def provided_capabilities(self) -> Set[Capability]:
        raise NotImplementedError

    def _backend_knows(self, name: str) -> bool:
        return False

    def _configure(self):
        """Validate the backend's parameters; return ({site: source class or None}, {site: ns})."""
        raise NotImplementedError

    def _create_backend(self) -> None:
        pass

    def _initial_observation(self, now: int) -> None:
        raise NotImplementedError

    def _tick(self, now: int) -> None:
        pass

    def _decide_binary(self, a: str, command: Command) -> Decision:
        raise NotImplementedError

    def _drive_binary(self, a: str, command: Command) -> None:
        pass

    def _on_applied(self, a: str, command: Command) -> None:
        pass

    def _backend_joint_positions(self) -> Dict[str, float]:
        return {}


def run(node_cls: Type[EefAdapterNode], args=None) -> int:
    """The console-script body: 2 on a refused contract (the runner's code), 0 on a clean stop."""
    rclpy.init(args=args)
    try:
        node = node_cls()
    except Refused as e:
        rclpy.logging.get_logger(node_cls.DEFAULT_NODE_NAME).fatal(
            f'end-effector contract refused: {e} (ADR-0022); exiting with code 2')
        print(f'REFUSED {e}', file=sys.stderr, flush=True)
        rclpy.try_shutdown()
        return 2
    executor = MultiThreadedExecutor(num_threads=node.recommended_threads())
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # rclpy (Humble) can raise RCLError ("failed to initialize wait set: the given context is
        # not valid") when the SIGINT handler shuts the context down while spin() rebuilds its wait
        # set: that is the same clean stop. While the context is still valid it is a real failure.
        if rclpy.ok():
            raise
    finally:
        node.close()
        executor.shutdown(timeout_sec=5.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
