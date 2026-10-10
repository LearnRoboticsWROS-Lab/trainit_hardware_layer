#!/usr/bin/env python3
"""mock_eef_adapter -- the ``trainit_mock`` adapter row (ADR-0022 D8): one end-effector device with
no simulator and no hardware, every actuator served on its standard interface at its canonical
name, the tool state published as the contract says.

One process per device, in the cell namespace (``ros2 run trainit_hardware_layer mock_eef_adapter
--ros-args -r __ns:=/r1 --params-file <file>``), with ``device_id`` and the device's ``eef.<id>.*``
block (the SAME block TMR reads). It serves:

- every ``set_bool`` actuator: std_srvs/SetBool ``eef/<id>/actuators/<a>/set_engaged``;
- every ``gripper_command`` actuator: control_msgs/GripperCommand at its declared ``action``,
  emulating the visible behaviour of a ros2_control GripperActionController: ONE joint, a
  server-side ``goal_tolerance`` (a GripperCommand goal cannot carry one), the ``stalled`` and
  ``reached_goal`` flags, CANCELED with an empty result on a cancel, and on a preemption at once
  (when the newer goal is accepted);
- every ``follow_joint_trajectory`` actuator: control_msgs/FollowJointTrajectory at its declared
  ``action``, emulating a joint_trajectory_controller: EXACTLY the actuator's joint set (any order;
  a partial or foreign goal is REJECTED), effort REJECTED, the trajectory followed over its
  ``time_from_start`` (linear), the tolerances CARRIED in the goal checked as the JTC resolves them
  (below), ABORTED -4 / -5, and CANCELED with an EMPTY result (code 0, no error string) on a cancel
  and on a preemption. A preempted goal stays EXECUTING until the goal that preempted it stops
  driving the axis (it terminates, whatever its outcome, or is preempted in turn) and only then
  ends; a client cancel of it meanwhile is accepted and ends nothing. So a preempted client whose
  own deadline comes first times out.

The JTC behaviour emulated here is the installed joint_trajectory_controller 2.54.0, as M9 measured
it (pins P3, P5, X3, X4, X7; a preemption of the preempting goal is read in its code, not measured).
The tolerances a goal carries (control_msgs/JointTolerance) are resolved per value: > 0 is checked,
0 keeps the server default and -1 erases it (neither checks anything: the installed defaults check
nothing). An entry that names a joint the controller does not have, or holds any other negative
value (velocity and acceleration included), makes the JTC drop EVERY carried tolerance of that goal,
the goal time tolerance included ("Using default tolerances"). An axis outside its goal tolerance at
the end of its trajectory is ABORTED -5 once its goal time tolerance has passed; with a goal time
tolerance of 0 it is never aborted: the goal waits until it is cancelled or preempted.

It publishes the state (``eef/<id>/state``), the joints it serves (``eef/<id>/joint_states``) and,
with ``scene_signal: adapter``, the scene signal (``eef/<id>/engaged``). It never publishes
``/suction/attached`` and never runs asyncio inside rclpy.

Every site has a SCRIPTED deciding source, sampled periodically and driven by the APPLIED commands
of the site's grip (from any commander): with nothing commanded it reports not held; after an
applied engage it is undecided (PENDING) for ``decide_after_ms`` and then reports its ``outcome``;
after an applied release it keeps its pre-release reading for ``decide_after_ms`` (a sensor's lag)
and then reports not held.

PARAMETERS. Failure injection is by node parameters ONLY: there is no service, topic or action that
injects anything. Unknown ``mock.*`` keys, and keys that do not fit the actuator's interface, are
refused at start-up (exit code 2).

``mock.sample_rate_hz`` (double, 20.0)
    The scripted sources' sampling rate; at least 1000 / ``verification.max_age_ms`` (a grasp
    source is periodic, at a rate of at least 1/max_age).

``mock.actuators.<a>.fail`` (string, ``none``)
    ``none``: served normally.
    ``reject``: SetBool ``success: false`` / the goal is REJECTED; ``detail`` names it.
    ``no_answer``: SetBool is never answered and nothing is applied / no goal response.
    ``applied_unanswered``: applied (recorded, visible in the state) but never answered / the goal
    is accepted and never finishes (a cancel ends it CANCELED).
    ``fault``: the actuator is FAULT from ``fail_after_s`` on; commands are still served.
    ``unavailable``: UNAVAILABLE from ``fail_after_s`` on; SetBool ``success: false``, goals
    REJECTED.
    ``abort_m1``: FJT ABORTED -1 halfway (a deactivated controller); GripperCommand ABORTED with
    both flags false, halfway.
    ``abort_m4`` (FJT): ABORTED -4 (PATH_TOLERANCE_VIOLATED) halfway.
    ``abort_m5`` (FJT): the axis stops halfway; ABORTED -5 (GOAL_TOLERANCE_VIOLATED) at the end of
    the trajectory plus its goal time tolerance.
    ``blocked`` (FJT): the axis stops halfway and the CARRIED tolerances decide, as on a real JTC:
    -4 once a path tolerance is exceeded, -5 when a goal tolerance is (once the goal time tolerance
    has passed; with a goal time tolerance of 0 the goal waits instead), SUCCEEDED when the goal
    carries none, or only ones the JTC drops (the reason TSA refuses an FJT actuator without
    ``goal_tolerance``).
    ``stall_succeeded`` (GripperCommand): the jaw stalls halfway; SUCCEEDED, ``stalled``,
    ``reached_goal`` false (``allow_stalling: true``).
    ``stall_aborted`` (GripperCommand): the same, ABORTED (``allow_stalling: false``).
    ``canceled`` (actions): the goal ends CANCELED halfway, with an empty result (FJT code 0 and
    no error string), as a cancel or a preemption ends it on the real controllers.
    Valid per interface: set_bool takes none, reject, no_answer, applied_unanswered, fault,
    unavailable; gripper_command adds abort_m1, stall_succeeded, stall_aborted, canceled;
    follow_joint_trajectory adds abort_m1, abort_m4, abort_m5, blocked, canceled.

``mock.actuators.<a>.fail_after_s`` (double, 0.0)
    ``fault`` / ``unavailable`` begin this long after the adapter starts.
``mock.actuators.<a>.switch_confirms`` (bool, true)
    set_bool with ``actuation_feedback`` only: the end switch confirms each applied state after
    ``confirm_after_ms`` (``binary_confirmed``); with false it never confirms and the actuator
    stays MOVING (the commander's own confirm timeout decides).
``mock.actuators.<a>.confirm_after_ms`` (int, 50)
    The switch's delay.
``mock.actuators.<a>.goal_tolerance`` (double, 0.001)
    gripper_command only: the server-side tolerance of ``reached_goal``.
``mock.actuators.<a>.travel_s`` (double, 0.2)
    gripper_command only: the jaw's travel time for one goal.
``mock.actuators.<a>.initial_target`` (string)
    gripper_command / follow_joint_trajectory: the named target the joints start at. Default: a
    grip's release target; otherwise ``home`` when declared, else the first target name in sorted
    order.

``mock.sites.<s>.outcome`` (string, ``held``)
    What the scripted source reports after an applied engage of the site's grip:
    ``held``: HELD after ``decide_after_ms``.
    ``not_held``: NOT_HELD after ``decide_after_ms``.
    ``pending``: undecided for ever (a fresh PENDING).
    ``lost_after``: as ``held``; ``after_s`` after the decision it reports not held with no release:
    the site reads LOST.
    ``stale_after``: as ``held``; ``after_s`` after the decision the source stops sampling (its
    stamp freezes) until the next applied engage or release.
    ``unknown``: the source samples but can never tell: UNKNOWN.
    ``unavailable``: the site has no deciding source: UNAVAILABLE (no grasp_detection).
``mock.sites.<s>.evidence`` (string, ``sensor``)
    The source's class: ``sensor``, ``sim_ground_truth`` or ``actuation_inference``.
``mock.sites.<s>.after_s`` (double, 1.0)
    For ``lost_after`` and ``stale_after``.
``mock.sites.<s>.decide_after_ms`` (int, 100)
    The source's decision time after an applied engage or release.
``mock.sites.<s>.silence_timeout_ms`` (int, 0)
    > 0: a source silent for longer reads UNKNOWN (0: off, so a frozen stamp reads stale to a
    consumer).

The common parameters (``publish_rate_hz``, the ``compat.*`` endpoints) are documented in
``adapter_node``. Times: the scripted sources run on the ROS clock (their stamps' clock); emulated
motions run on the monotonic clock (a hardware speed).
"""
from __future__ import annotations

import functools
import math
import sys
import time
from typing import Dict, List, Optional, Sequence, Set, Tuple

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import GoalEvent
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint

from . import core
from .adapter_node import Decision, EefAdapterNode, run
from .contract import Refused
from .core import ActuatorStatus, Binary, Capability, Command, Reading

FAILS = {
    'set_bool': ('none', 'reject', 'no_answer', 'applied_unanswered', 'fault', 'unavailable'),
    'gripper_command': ('none', 'reject', 'no_answer', 'applied_unanswered', 'fault', 'unavailable',
                        'abort_m1', 'stall_succeeded', 'stall_aborted', 'canceled'),
    'follow_joint_trajectory': ('none', 'reject', 'no_answer', 'applied_unanswered', 'fault',
                                'unavailable', 'abort_m1', 'abort_m4', 'abort_m5', 'blocked',
                                'canceled'),
}
OUTCOMES = ('held', 'not_held', 'pending', 'lost_after', 'stale_after', 'unknown', 'unavailable')
ACTUATOR_KEYS = ('fail', 'fail_after_s', 'switch_confirms', 'confirm_after_ms', 'goal_tolerance',
                 'travel_s', 'initial_target')
SITE_KEYS = ('outcome', 'evidence', 'after_s', 'decide_after_ms', 'silence_timeout_ms')

LOOP_S = 0.01            # an emulated motion's step
FEEDBACK_PERIOD_S = 0.05
_SILENT = None           # a scripted source that takes no sample at this instant
ERASE_TOLERANCE = -1.0   # control_msgs/JointTolerance: "erased", no restriction for this goal
FLOAT_EPSILON = 1.1920928955078125e-07   # the JTC matches the erase value within float epsilon


def _ns(duration) -> int:
    return int(duration.sec) * 1_000_000_000 + int(duration.nanosec)


def _same(x: float, y: float) -> bool:
    return abs(x - y) <= 1e-9 * max(1.0, abs(y))


def _interp(start: Sequence[float], points: Sequence[Tuple[int, Sequence[float]]],
            t_ns: float) -> List[float]:
    """Piecewise-linear reference at t, from `start` at t=0 through (time_from_start, positions)."""
    prev_t, prev_p = 0, list(start)
    for pt, pp in points:
        if t_ns <= pt:
            if pt <= prev_t:
                return list(pp)
            f = (t_ns - prev_t) / (pt - prev_t)
            return [a + (b - a) * f for a, b in zip(prev_p, pp)]
        prev_t, prev_p = pt, list(pp)
    return list(points[-1][1])


def _resolved(value: float) -> Optional[float]:
    """One carried tolerance value as the JTC resolves it against its installed default, 0.

    > 0 is checked; 0 (the default) and -1 (erased) check nothing (0.0); any other value < 0 is
    illegal (None). A NaN is not > 0 and not < 0: the default.
    """
    if value > 0.0:
        return value
    if abs(value - ERASE_TOLERANCE) < FLOAT_EPSILON or not value < 0.0:
        return 0.0
    return None


def carried_tolerances(goal, joints: Sequence[str]) -> Tuple[Dict[str, float], Dict[str, float], int, str]:
    """({joint: path tolerance}, {joint: goal tolerance}, goal time tolerance ns, why) of a goal.

    What a joint_trajectory_controller with the installed defaults (every ``constraints`` value 0)
    applies; 0 means "not checked". On an entry that names a joint the controller does not have,
    or holds an illegal value, it uses its defaults for the WHOLE goal: nothing is checked and the
    goal time tolerance is 0; ``why`` then names the entry, as the JTC's log does ('' otherwise).
    """
    defaults = ({}, {}, 0)
    goal_time_ns = _ns(goal.goal_time_tolerance)
    goal_time = _resolved(goal_time_ns / 1e9)
    if goal_time is None:
        return defaults + (f'an illegal goal_time_tolerance ({goal_time_ns / 1e9:g} s)',)
    resolved = []
    for field_name, entries in (('path_tolerance', goal.path_tolerance),
                                ('goal_tolerance', goal.goal_tolerance)):
        tolerances = {}
        for entry in entries:
            if entry.name not in joints:
                return defaults + (f"joint '{entry.name}' specified in goal.{field_name} does not exist",)
            values = [_resolved(v) for v in (entry.position, entry.velocity, entry.acceleration)]
            if any(v is None for v in values):
                return defaults + (f"joint '{entry.name}' specified in goal.{field_name} has an invalid "
                                   'tolerance',)
            tolerances[entry.name] = values[0]
        resolved.append(tolerances)
    return resolved[0], resolved[1], goal_time_ns if goal_time > 0.0 else 0, ''


def server_cancel(goal_handle) -> None:
    """End a goal CANCELED from the server side, as a controller ends a preempted goal.

    rcl_action forbids EXECUTING -> CANCELED, so the goal goes through CANCELING first, exactly as
    rclpy's own ActionServer does for an accepted cancel request (``GoalEvent.CANCEL_GOAL``).
    """
    try:
        goal_handle._update_state(GoalEvent.CANCEL_GOAL)
    except Exception:  # noqa: BLE001 -- already CANCELING: a client cancel raced the preemption
        pass
    goal_handle.canceled()


class _Axis:
    """An emulated controller: its joints' positions and its current goal's generation."""

    def __init__(self, spec, positions: List[float], goal_tolerance: float, travel_s: float):
        self.spec = spec
        self.positions = positions
        self.goal_tolerance = goal_tolerance
        self.travel_s = travel_s
        self.generation = 0                 # the newest accepted goal (one more per accepted goal)
        self.active: Optional[int] = None   # the goal driving the axis; None once it has ended
        self.server = None


class _SiteScript:
    """The scripted deciding source of one site."""

    def __init__(self, outcome: str, decide_ns: int, after_ns: int):
        self.outcome = outcome
        self.decide_ns = decide_ns
        self.after_ns = after_ns
        self.last: Optional[Reading] = None
        self.before_release = Reading.NOT_HELD

    def reading(self, command: Command, tc: int, now: int) -> Optional[Reading]:
        if self.outcome == 'unknown':
            return Reading.CANNOT_TELL
        if command == Command.ENGAGE:
            if now < tc + self.decide_ns:
                return Reading.UNDECIDED
            decided = tc + self.decide_ns
            if self.outcome == 'held':
                return Reading.HELD
            if self.outcome == 'not_held':
                return Reading.NOT_HELD
            if self.outcome == 'pending':
                return Reading.UNDECIDED
            if self.outcome == 'lost_after':
                return Reading.HELD if now < decided + self.after_ns else Reading.NOT_HELD
            if self.outcome == 'stale_after':
                return Reading.HELD if now < decided + self.after_ns else _SILENT
        if command == Command.RELEASE:
            if now < tc + self.decide_ns:
                return self.before_release
            return Reading.NOT_HELD
        if command == Command.TARGET:
            return self.last or Reading.NOT_HELD
        return Reading.NOT_HELD


class MockEefAdapter(EefAdapterNode):
    """The ``trainit_mock`` row: every actuator emulated, every site scripted."""

    ADAPTER_ROW = 'trainit_mock'
    DEFAULT_NODE_NAME = 'mock_eef_adapter'
    BACKEND_PREFIXES = ('mock',)

    # --- configuration ----------------------------------------------------------------------------
    def _backend_knows(self, name: str) -> bool:
        if name == 'mock.sample_rate_hz':
            return True
        parts = name.split('.')
        if len(parts) != 4 or parts[0] != 'mock':
            return False
        if parts[1] == 'actuators':
            return parts[2] in self.device.actuators and parts[3] in ACTUATOR_KEYS
        if parts[1] == 'sites':
            return parts[2] in self.device.sites and parts[3] in SITE_KEYS
        return False

    def _not_for(self, key: str, why: str) -> None:
        if self.has_parameter(key):
            raise Refused(key, why)

    def _configure(self):
        dev = self.device
        self.sample_rate_hz = self.param('mock.sample_rate_hz', float, 20.0, positive=True)
        if self.sample_rate_hz * dev.verification.max_age_ms < 1000.0:
            raise Refused('mock.sample_rate_hz',
                          f'at least 1000 / max_age_ms = {1000.0 / dev.verification.max_age_ms:g} Hz '
                          '(a grasp source is periodic, at a rate of at least 1/max_age)')
        self._fail: Dict[str, str] = {}
        self._fail_after_ns: Dict[str, int] = {}
        self._condition: Dict[str, Optional[ActuatorStatus]] = {}
        self._switch_confirms: Dict[str, bool] = {}
        self._confirm_after_ns: Dict[str, int] = {}
        self._switch: Dict[str, Binary] = {}
        self._switch_pending: Dict[str, bool] = {}
        self._axes: Dict[str, _Axis] = {}
        for a in dev.actuator_ids:
            spec = dev.actuators[a]
            iface = spec.command_interface
            p = f'mock.actuators.{a}.'
            fail = self.param(p + 'fail', str, 'none')
            if fail not in FAILS[iface]:
                raise Refused(p + 'fail', f'one of {FAILS[iface]} for a {iface} actuator')
            self._fail[a] = fail
            self._fail_after_ns[a] = int(
                self.param(p + 'fail_after_s', float, 0.0, non_negative=True) * 1e9)
            self._condition[a] = None
            if iface == 'set_bool':
                for k in ('goal_tolerance', 'travel_s', 'initial_target'):
                    self._not_for(p + k, 'not for a set_bool actuator')
                if not spec.actuation_feedback:
                    for k in ('switch_confirms', 'confirm_after_ms'):
                        self._not_for(p + k, 'only for a set_bool actuator with actuation_feedback')
                if spec.actuation_feedback:
                    self._switch_confirms[a] = self.param(p + 'switch_confirms', bool, True)
                    self._confirm_after_ns[a] = self.param(
                        p + 'confirm_after_ms', int, 50, positive=True) * 1_000_000
                    self._switch[a] = Binary.UNKNOWN
                    self._switch_pending[a] = False
                continue
            for k in ('switch_confirms', 'confirm_after_ms'):
                self._not_for(p + k, 'only for a set_bool actuator with actuation_feedback')
            if iface == 'follow_joint_trajectory':
                for k in ('goal_tolerance', 'travel_s'):
                    self._not_for(p + k, 'only for a gripper_command actuator')
            site = dev.site_gripped_by(a)
            if site is not None:
                default_target = site.release
            elif 'home' in spec.targets:
                default_target = 'home'
            else:
                default_target = sorted(spec.targets)[0]
            initial = self.param(p + 'initial_target', str, default_target)
            if initial not in spec.targets:
                raise Refused(p + 'initial_target', f'one of {sorted(spec.targets)}')
            value = spec.targets[initial]
            positions = [float(value)] if iface == 'gripper_command' else [float(x) for x in value]
            self._axes[a] = _Axis(
                spec, positions,
                self.param(p + 'goal_tolerance', float, 0.001, positive=True),
                self.param(p + 'travel_s', float, 0.2, positive=True))
        self._scripts: Dict[str, _SiteScript] = {}
        sources, silence_ns = {}, {}
        for s in dev.site_ids:
            p = f'mock.sites.{s}.'
            outcome = self.param(p + 'outcome', str, 'held')
            if outcome not in OUTCOMES:
                raise Refused(p + 'outcome', f'one of {OUTCOMES}')
            evidence = self.param(p + 'evidence', str, 'sensor')
            if evidence not in core.EVIDENCE_BY_NAME:
                raise Refused(p + 'evidence', f'one of {tuple(core.EVIDENCE_BY_NAME)}')
            after_ns = int(self.param(p + 'after_s', float, 1.0, positive=True) * 1e9)
            decide_ns = self.param(p + 'decide_after_ms', int, 100, non_negative=True) * 1_000_000
            silence = self.param(p + 'silence_timeout_ms', int, 0, non_negative=True)
            self._scripts[s] = _SiteScript(outcome, decide_ns, after_ns)
            sources[s] = None if outcome == 'unavailable' else core.EVIDENCE_BY_NAME[evidence]
            silence_ns[s] = silence * 1_000_000
        self._t0 = 0
        return sources, silence_ns

    def _create_backend(self) -> None:
        for a, axis in self._axes.items():
            action_type = (GripperCommand if axis.spec.command_interface == 'gripper_command'
                           else FollowJointTrajectory)
            axis.server = self._named(
                f'eef.{self.device.id}.actuators.{a}.action',
                lambda _a=a, _t=action_type: ActionServer(
                    self, _t, self._axes[_a].spec.action,
                    execute_callback=functools.partial(self._unexpected_execute, _t),
                    goal_callback=functools.partial(self._on_goal, _a),
                    handle_accepted_callback=functools.partial(self._on_accepted, _a),
                    cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
                    callback_group=MutuallyExclusiveCallbackGroup()))
        self.create_timer(1.0 / self.sample_rate_hz, self._sample_sites,
                          callback_group=self._timer_group, clock=self._steady)

    def provided_capabilities(self) -> Set[Capability]:
        caps = {Capability.CONTINUOUS_STATE}
        for a in self.device.actuator_ids:
            spec = self.device.actuators[a]
            caps.update(core.IMPLIED_CAPABILITIES[spec.command_interface])
            if spec.command_interface == 'set_bool' and spec.actuation_feedback:
                caps.add(Capability.ACTUATION_FEEDBACK)
            if spec.command_interface == 'gripper_command':
                caps.add(Capability.EFFORT_CONTROL)
        if any(self.core.site_source(s) is not None for s in self.device.site_ids):
            caps.update((Capability.GRASP_DETECTION, Capability.RELEASE_DETECTION))
        return caps

    # --- periodic ---------------------------------------------------------------------------------
    def _initial_observation(self, now: int) -> None:
        self._t0 = now
        for a in self.device.actuator_ids:
            spec = self.device.actuators[a]
            if spec.command_interface == 'set_bool':
                self.core.observe_actuator(a, now, binary=Binary.RELEASED)
                if spec.actuation_feedback and self._switch_confirms[a]:
                    self.core.observe_switch(a, Binary.RELEASED, now)   # the switch at rest
                    self._switch[a] = Binary.RELEASED
            else:
                self.core.observe_actuator(a, now)
        self._apply_conditions(now)

    def _condition_now(self, a: str, now: int) -> Optional[ActuatorStatus]:
        fail = self._fail[a]
        # _t0 is the first thing the initial observation sets: a condition injected from t0 is
        # already in the first (latched) sample, which the constructor publishes
        if fail not in ('fault', 'unavailable') or self._t0 <= 0:
            return None
        if now < self._t0 + self._fail_after_ns[a]:
            return None
        return ActuatorStatus.FAULT if fail == 'fault' else ActuatorStatus.UNAVAILABLE

    def _apply_conditions(self, now: int) -> None:
        for a in self.device.actuator_ids:
            status = self._condition_now(a, now)
            if status != self._condition[a]:
                self._condition[a] = status
                note = f'injected {status.name} (mock)' if status is not None else ''
                self.core.set_condition(a, status, note)

    def _tick(self, now: int) -> None:
        self._apply_conditions(now)
        for a in self._switch_pending:
            if not self._switch_pending[a] or not self._switch_confirms[a]:
                continue
            _cmd, tc = self.core.last_command(a)
            if now >= tc + self._confirm_after_ns[a]:
                binary = self.core.binary(a)
                self.core.observe_switch(a, binary, now)   # the end switch reaches the new state
                self._switch[a] = binary
                self._switch_pending[a] = False

    def _sample_sites(self) -> None:
        if self.closing:
            return
        with self._lock:
            now = self.now_ns()
            if not self._maybe_start() or now <= 0:
                return
            for s, script in self._scripts.items():
                if script.outcome == 'unavailable':
                    continue
                command, tc = self.core.last_command(self.device.sites[s].grip)
                reading = script.reading(command, tc, now)
                if reading is _SILENT:
                    continue
                self.core.observe_site(s, reading, now)
                script.last = reading
            self._publish_changes()

    def _on_applied(self, a: str, command: Command) -> None:
        if a in self._switch_pending:
            self._switch_pending[a] = True          # every applied state waits for its switch
        site = self.device.site_gripped_by(a)
        if site is not None and command == Command.RELEASE:
            script = self._scripts[site.id]
            script.before_release = Reading.HELD if script.last == Reading.HELD else Reading.NOT_HELD

    def _backend_joint_positions(self) -> Dict[str, float]:
        out = {}
        for axis in self._axes.values():
            out.update(zip(axis.spec.joints, axis.positions))
        return out

    # --- set_bool ---------------------------------------------------------------------------------
    def _decide_binary(self, a: str, command: Command) -> Decision:
        fail = self._fail[a]
        if self._condition_now(a, self.now_ns()) == ActuatorStatus.UNAVAILABLE:
            return Decision(False, True, 'injected UNAVAILABLE (mock): the hardware link is down')
        if fail == 'reject':
            return Decision(False, True, 'injected reject (mock)')
        if fail == 'no_answer':
            return Decision(False, False, 'injected no answer (mock)')
        if fail == 'applied_unanswered':
            return Decision(True, False, 'injected applied but unanswered (mock)')
        return Decision(True, True, 'engaged' if command == Command.ENGAGE else 'released')

    # --- actions ----------------------------------------------------------------------------------
    def _classify(self, a: str, goal) -> Command:
        """ENGAGE / RELEASE for a goal to the site's engage / release target, else TARGET.

        The contract refuses two targets of one gripper_command actuator with the same value, so a
        position names one target (to within ``_same``'s relative 1e-9).
        """
        spec = self.device.actuators[a]
        site = self.device.site_gripped_by(a)
        if spec.command_interface == 'gripper_command' and site is not None:
            position = float(goal.command.position)
            if _same(position, float(spec.targets[site.engage])):
                return Command.ENGAGE
            if _same(position, float(spec.targets[site.release])):
                return Command.RELEASE
        return Command.TARGET

    def _goal_problem(self, a: str, goal) -> Optional[str]:
        spec = self.device.actuators[a]
        if spec.command_interface == 'gripper_command':
            if not math.isfinite(goal.command.position) or not math.isfinite(goal.command.max_effort):
                return 'a non-finite position or effort'
            return None
        names = list(goal.trajectory.joint_names)
        n = len(names)
        if goal.multi_dof_trajectory.joint_names or goal.multi_dof_trajectory.points:
            return 'a multi-DOF trajectory is not supported'
        if len(set(names)) != n:
            return f'duplicate joint names {names}'
        if set(names) != set(spec.joints):
            return f'joints {names} are not exactly the controller joints {list(spec.joints)}'
        if not goal.trajectory.points:
            return 'an empty trajectory'
        previous = -1
        for i, p in enumerate(goal.trajectory.points):
            if len(p.positions) != n:
                return f'point {i}: {len(p.positions)} positions for {n} joints'
            if len(p.effort):
                return f'point {i}: effort is not supported'
            if len(p.velocities) not in (0, n) or len(p.accelerations) not in (0, n):
                return f'point {i}: a velocity or acceleration size mismatch'
            if not all(math.isfinite(x) for x in p.positions):
                return f'point {i}: a non-finite position'
            t = _ns(p.time_from_start)
            if t <= previous:
                return f'point {i}: time_from_start is not increasing'
            previous = t
        return None

    def _on_goal(self, a: str, goal) -> GoalResponse:
        if self.closing:
            return GoalResponse.REJECT
        fail = self._fail[a]
        with self._lock:
            started = self._maybe_start()
            now = self.now_ns()
            if not started or now <= 0:                     # no ROS time yet: nothing to stamp
                return GoalResponse.REJECT
            command = self._classify(a, goal)
            if self._condition_now(a, now) == ActuatorStatus.UNAVAILABLE:
                why = 'injected UNAVAILABLE (mock): the controller is not active'
            elif fail == 'reject':
                why = 'injected reject (mock)'
            else:
                why = self._goal_problem(a, goal)
            if why is not None:
                self.core.reject(a, command, why)
                self._publish_changes()
                return GoalResponse.REJECT
        if fail == 'no_answer':
            self._closing.wait()
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_accepted(self, a: str, goal_handle) -> None:
        axis = self._axes[a]
        with self._lock:
            axis.generation += 1
            generation = axis.generation
            axis.active = generation            # it drives the axis now: any older goal is preempted
            command = self._classify(a, goal_handle.request)
            self.core.apply(a, command, max(self.now_ns(), 1))   # > 0: the goal callback checked
            self._after_applied(a, command)
            self._publish_changes()
        execute = (self._execute_gripper if axis.spec.command_interface == 'gripper_command'
                   else self._execute_trajectory)
        self.begin_execution()
        goal_handle.execute(functools.partial(execute, a, generation))

    def _unexpected_execute(self, action_type, goal_handle):
        """Never used: every accepted goal is executed with its own bound callback."""
        goal_handle.abort()
        return action_type.Result()

    def _current(self, a: str, generation: int) -> bool:
        return self._axes[a].generation == generation

    def _finish(self, a: str, generation: int) -> None:
        """The goal of `generation` reached its terminal state while it drove the axis."""
        with self._lock:
            axis = self._axes[a]
            if axis.active == generation:
                axis.active = None              # the goal it preempted, if any, ends now
            if self._current(a, generation):
                now = self.now_ns()
                if now > 0:
                    self.core.goal_done(a, now)
                self._publish_changes()

    def _set_positions(self, a: str, generation: int, positions: Sequence[float]) -> None:
        with self._lock:
            if self._current(a, generation):
                self._axes[a].positions = [float(x) for x in positions]

    def _execute_gripper(self, a: str, generation: int, goal_handle):
        with self.execution():
            return self._run_gripper(a, generation, goal_handle)

    def _execute_trajectory(self, a: str, generation: int, goal_handle):
        with self.execution():
            return self._run_trajectory(a, generation, goal_handle)

    def _run_gripper(self, a: str, generation: int, goal_handle):
        axis = self._axes[a]
        fail = self._fail[a]
        target = float(goal_handle.request.command.position)
        max_effort = float(goal_handle.request.command.max_effort)
        with self._lock:
            start = axis.positions[0]
        stop = 0.5 if fail in ('stall_succeeded', 'stall_aborted', 'abort_m1', 'canceled') else None
        t0 = time.monotonic()
        while True:
            if self.closing:
                goal_handle.abort()
                return GripperCommand.Result()
            if not self._current(a, generation):           # preempted: a GAC ends it at once
                server_cancel(goal_handle)
                return GripperCommand.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._finish(a, generation)
                return GripperCommand.Result()
            if fail == 'applied_unanswered':
                time.sleep(LOOP_S)
                continue
            frac = min(1.0, (time.monotonic() - t0) / axis.travel_s)
            reached = min(frac, stop) if stop is not None else frac
            position = start + (target - start) * reached
            self._set_positions(a, generation, [position])
            if stop is not None and frac >= stop:
                if fail == 'canceled':
                    server_cancel(goal_handle)
                    self._finish(a, generation)
                    return GripperCommand.Result()
                result = GripperCommand.Result()
                result.position = position
                result.reached_goal = abs(position - target) <= axis.goal_tolerance
                if fail == 'abort_m1':
                    result.effort, result.stalled = 0.0, False
                    goal_handle.abort()
                else:
                    result.effort, result.stalled = max_effort, True
                    (goal_handle.succeed if fail == 'stall_succeeded' else goal_handle.abort)()
                self._finish(a, generation)
                return result
            if frac >= 1.0:
                result = GripperCommand.Result()
                result.position = position
                result.effort = 0.0
                result.stalled = False
                result.reached_goal = abs(position - target) <= axis.goal_tolerance
                goal_handle.succeed()
                self._finish(a, generation)
                return result
            time.sleep(LOOP_S)

    def _run_trajectory(self, a: str, generation: int, goal_handle):
        axis = self._axes[a]
        spec = axis.spec
        fail = self._fail[a]
        goal = goal_handle.request
        names = list(goal.trajectory.joint_names)
        order = [names.index(j) for j in spec.joints]
        points = [(_ns(p.time_from_start), [p.positions[i] for i in order])
                  for p in goal.trajectory.points]
        total = points[-1][0]
        target = points[-1][1]
        path_tol, goal_tol, goal_time_ns, why = carried_tolerances(goal, spec.joints)
        if why:
            self.get_logger().error(f'{a}: {why}: using the default tolerances, as a '
                                    'joint_trajectory_controller does (nothing is checked)')
        with self._lock:
            start = list(axis.positions)
        stop_at = 0.5 * total if fail in ('blocked', 'abort_m5') else None
        event_at = 0.5 * total if fail in ('abort_m1', 'abort_m4', 'canceled') else None
        t0 = time.monotonic_ns()
        last_feedback = 0.0
        while True:
            if self.closing:
                goal_handle.abort()
                return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                                                    error_string='the adapter is shutting down')
            if not self._current(a, generation):           # a newer goal preempted this one
                return self._end_preempted(a, generation, goal_handle)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._finish(a, generation)
                return FollowJointTrajectory.Result()
            if fail == 'applied_unanswered':
                time.sleep(LOOP_S)
                continue
            elapsed = time.monotonic_ns() - t0
            reference = _interp(start, points, elapsed)
            actual = reference if stop_at is None else _interp(start, points, min(elapsed, stop_at))
            self._set_positions(a, generation, actual)
            if time.monotonic() - last_feedback >= FEEDBACK_PERIOD_S:
                last_feedback = time.monotonic()
                self._feedback(goal_handle, spec.joints, reference, actual)
            outcome = None
            if event_at is not None and elapsed >= event_at:
                if fail == 'canceled':
                    server_cancel(goal_handle)
                    self._finish(a, generation)
                    return FollowJointTrajectory.Result()               # empty: code 0, no string
                outcome = ((FollowJointTrajectory.Result.INVALID_GOAL,
                            'Current goal aborted: the controller was deactivated (mock)')
                           if fail == 'abort_m1' else
                           (FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                            'Aborted due to path tolerance violation'))
            elif fail == 'blocked' and any(path_tol.get(j, 0.0) > 0.0 and abs(r - x) > path_tol[j]
                                           for j, r, x in zip(spec.joints, reference, actual)):
                outcome = (FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                           'Aborted due to path tolerance violation')
            elif elapsed >= total:
                within = fail != 'abort_m5' and all(
                    goal_tol.get(j, 0.0) <= 0.0 or abs(t - x) <= goal_tol[j]
                    for j, t, x in zip(spec.joints, target, actual))
                if within:
                    goal_handle.succeed()
                    self._finish(a, generation)
                    return FollowJointTrajectory.Result(
                        error_code=FollowJointTrajectory.Result.SUCCESSFUL, error_string='')
                # outside: -5 once the goal time tolerance has passed; with a goal time tolerance
                # of 0 a JTC never aborts, the goal waits (the injected abort_m5 aborts even then)
                if (goal_time_ns > 0 or fail == 'abort_m5') and elapsed >= total + goal_time_ns:
                    outcome = (FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                               'Aborted due to goal tolerance violation')
            if outcome is not None:
                goal_handle.abort()
                self._finish(a, generation)
                return FollowJointTrajectory.Result(error_code=outcome[0], error_string=outcome[1])
            time.sleep(LOOP_S)

    def _end_preempted(self, a: str, generation: int, goal_handle):
        """A preempted trajectory goal, ended the JTC's way: CANCELED with an EMPTY result, and only
        once the goal that preempted it (the next generation) no longer drives the axis.

        Until then the goal is EXECUTING and moves nothing; a client cancel of it is accepted
        (CANCELING) and ends nothing. A third goal preempting the second ends this one at once.
        """
        axis = self._axes[a]
        while True:
            if self.closing:
                goal_handle.abort()
                return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                                                    error_string='the adapter is shutting down')
            with self._lock:
                ended = axis.active != generation + 1
            if ended:
                server_cancel(goal_handle)
                return FollowJointTrajectory.Result()
            time.sleep(LOOP_S)

    def _feedback(self, goal_handle, joints: Sequence[str], reference: Sequence[float],
                  actual: Sequence[float]) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(joints)
        feedback.desired = JointTrajectoryPoint(positions=[float(x) for x in reference])
        feedback.actual = JointTrajectoryPoint(positions=[float(x) for x in actual])
        feedback.error = JointTrajectoryPoint(
            positions=[float(r - x) for r, x in zip(reference, actual)])
        goal_handle.publish_feedback(feedback)


def main(args=None):
    sys.exit(run(MockEefAdapter, args))


if __name__ == '__main__':
    main()
