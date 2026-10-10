#!/usr/bin/env python3
"""isaac_eef_adapter -- the ``trainit_isaac_surface_gripper`` adapter row (ADR-0022 D8): the ROS
side of a binary suction gripper simulated by an Isaac Sim SurfaceGripper.

A system-ROS node (Isaac's bundled ROS 2 has no TrainIt or control_msgs types), one per robot, in
the robot's cell namespace, honouring ``use_sim_time``. The stage side stays a framework template
in TSA's resources; this node only talks to it over two plain std_msgs topics.

It serves exactly ONE binary suction vacuum: a device with one ``set_bool`` actuator that grips the
device's only site (kind suction; ADR-0022 R20). Anything else is refused at start-up (exit code
2), and so is ``actuation_feedback`` (this binding provides no switch).

Commands. std_srvs/SetBool ``eef/<id>/actuators/<vacuum>/set_engaged`` (and the compatibility
endpoints of ``adapter_node``). An applied command is published on ``stage_cmd_topic`` as a
std_msgs/Bool (true engages). That publisher is TRANSIENT_LOCAL and is republished at
``republish_rate_hz``, because the stage's subscription is volatile; the adapter starts released
(it publishes false), as the legacy bridge does. With no stage the command is refused
(``success: false``), nothing is applied, and the vacuum reads UNAVAILABLE until the stage is
back. The stage is there when its command topic has a subscriber and, with ``stage_status_topic``,
when its status arrived within ``status_timeout_ms`` of WALL time: a subscriber alone (a recorder,
an echo) is not the stage. Without a status topic a subscriber is all the adapter can see.

State. Without ``stage_status_topic`` the site is UNAVAILABLE with evidence NONE: on this backend
there is then no ``grasp_detection``, and a grasp is never inferred from the command echo. With it,
the stage publishes a std_msgs/String "<Open|Closing|Closed> <gripped_count>", mapped BY NAME (never
by an integer: Isaac's SurfaceGripperManager documents an order different from its enum) to
SIM_GROUND_TRUTH:

| Stage status | Gripped count | Context | Vacuum binary / site grasp |
|---|---|---|---|
| Closed | > 0 | - | ENGAGED / HELD |
| Closing | > 0 | - | ENGAGED / HELD, ``detail``: partial attachment |
| Closing | 0 | - | ENGAGED / PENDING |
| Closed | 0 | after an applied ENGAGE | ENGAGED / PENDING: a transient (Isaac 5.1 reports it for |
| | | | one frame on every attach, a re-grip included), never a decision |
| Open | 0 | after an applied ENGAGE | PENDING within ``engage_timeout_ms``; |
| | | | then NOT_HELD, or LOST if it had been HELD |
| Open (or Closed) | 0 | after an applied RELEASE, or no command | RELEASED / NOT_HELD |
| no sample within ``status_timeout_ms`` | - | - | site UNKNOWN, evidence NONE |

Silence is measured on both clocks: the ROS clock (sim time) and the WALL clock, because a dead or
paused stage stops ``/clock`` and with it any sim-time silence; a stage that sent nothing for
``status_timeout_ms`` of real time reads UNKNOWN, never its last decision.

The vacuum's ``binary`` is always the APPLIED command (the stage status is site evidence, never a
switch: ``binary_confirmed`` stays false). A malformed sample ("2 1", "Open x", a count that is not
plain ASCII digits such as "Closed ²") and the contradictory "Open <n > 0>" read UNKNOWN, are named
in ``detail`` and logged (throttled); no sample can raise in the callback and stop the adapter.
A status topic that never publishes is named in ``detail`` too, once ``status_timeout_ms`` has
passed since the start. SurfaceGripper Closed means "holding something", not "holding the
target": object identity is M3.

The stage must publish its status faster than once per ``status_timeout_ms``, with a margin for
its jitter: at least 5 Hz with the default 400 ms. A grasp source's floor in ADR-0022,
1/``max_age_ms`` (2 Hz with the default 500 ms), is not enough here: a status older than
``status_timeout_ms`` reads as silent, so a 2 Hz stage would read UNKNOWN for part of every period.

Reset. With ``scene_reset_topic`` (relative, std_msgs/Bool) a RISING edge is this robot's scene
reset: the stage opens the gripper, so the adapter records it as an APPLIED release (it clears a
LOST latch, and a following "Open 0" reads NOT_HELD, not LOST) and commands the stage released.

Parameters (besides ``device_id``, the ``eef.<id>.*`` block and the common ones of
``adapter_node``):
- ``stage_cmd_topic`` (string, REQUIRED, no default: TSA fills it from the stage topic it detects);
- ``stage_status_topic`` (string, '' = none);
- ``republish_rate_hz`` (double, 2.0);
- ``engage_timeout_ms`` (int, 2000; at most ``verification.timeout_ms``);
- ``status_timeout_ms`` (int, 400; at most ``verification.max_age_ms``; the stage publishes its
  status faster than once per this, see above);
- ``scene_reset_topic`` (string, '' = none).

The pseudo-joint echo the robot description needs (``compat.pseudo_joint_*``, e.g.
``/suction/joint_states`` with ``suctioncup_joint``) is description-compatibility state for
TF/MoveIt, never grasp or release evidence.
"""
from __future__ import annotations

import sys
from typing import Set, Tuple

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, String

from . import core
from .adapter_node import LATCHED_QOS, Decision, EefAdapterNode, run
from .contract import Refused
from .core import ActuatorStatus, Binary, Capability, Command, Reading

STATUS_NAMES = ('Open', 'Closing', 'Closed')
SHOWN_CHARS = 64            # how much of a malformed stage sample ``detail`` quotes
LOG_THROTTLE_S = 5.0        # a malformed or failing stage sample is logged at most this often
# The stage's publisher may be reliable or best effort; a best-effort subscription matches both.
STATUS_QOS = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
# A volatile subscription matches a latched reset publisher too, and replays no old pulse.
RESET_QOS = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)


def _cut(text: str) -> str:
    """A stage sample as ``detail`` shows it: at most SHOWN_CHARS characters of it."""
    return text if len(text) <= SHOWN_CHARS else text[:SHOWN_CHARS] + '...'


def _quoted(text: str) -> str:
    return repr(_cut(text))


def map_stage_status(text: str, engaged: bool, since_engage_ns: int,
                     engage_timeout_ns: int) -> Tuple[Reading, str]:
    """One stage status sample -> (reading, detail note), by NAME (ADR-0022 D8 table).

    `engaged`: the vacuum's newest applied command is ENGAGE, `since_engage_ns` ago. Total: any
    text gives a reading, never an exception. The count is plain ASCII digits (str.isdigit() also
    accepts '²' and '٣'; int() refuses the first and reads the second as 3).
    """
    parts = text.split()
    if (len(parts) != 2 or parts[0] not in STATUS_NAMES or not parts[1].isascii()
            or not parts[1].isdigit()):
        return Reading.CANNOT_TELL, f'malformed stage status {_quoted(text)}'
    try:
        count = int(parts[1])
    except ValueError:      # more digits than int() converts (Python's integer string limit)
        return Reading.CANNOT_TELL, f'malformed stage status {_quoted(text)}'
    name = parts[0]
    if count > 0:
        if name == 'Closed':
            return Reading.HELD, ''
        if name == 'Closing':
            return Reading.HELD, f'partial attachment ({_cut(text)})'
        return Reading.CANNOT_TELL, f'contradictory stage status {_quoted(text)}'
    if name == 'Closing':
        return Reading.UNDECIDED, ''
    if name == 'Closed' and engaged:
        # Isaac Sim 5.1 reports "Closed 0" for one frame on every attach, a re-grip included: a
        # transient, never a decision. A gripper that finds nothing retries as Closing and gives up
        # as Open, and "Open 0" is what decides (vNext M2.6 T5).
        return Reading.UNDECIDED, ''
    if engaged and since_engage_ns <= engage_timeout_ns:
        return Reading.UNDECIDED, ''
    return Reading.NOT_HELD, ''


class IsaacEefAdapter(EefAdapterNode):
    """The ``trainit_isaac_surface_gripper`` row: binary suction on an Isaac SurfaceGripper."""

    ADAPTER_ROW = 'trainit_isaac_surface_gripper'
    DEFAULT_NODE_NAME = 'isaac_eef_adapter'
    BACKEND_PARAMS = ('stage_cmd_topic', 'stage_status_topic', 'republish_rate_hz',
                      'engage_timeout_ms', 'status_timeout_ms', 'scene_reset_topic')

    def _configure(self):
        dev = self.device
        site = dev.default_site
        if (site is None or len(dev.actuator_ids) != 1
                or dev.actuators[site.grip].command_interface != 'set_bool'):
            raise Refused(f'eef.{dev.id}',
                          f'{self.ADAPTER_ROW} serves one binary suction vacuum: one set_bool actuator '
                          f'gripping the only site (actuators {list(dev.actuator_ids)}, sites '
                          f'{list(dev.site_ids)}; ADR-0022 D6, R20)')
        self.vacuum = site.grip
        self.site = site.id
        if dev.actuators[self.vacuum].actuation_feedback:
            raise Refused(f'eef.{dev.id}.actuators.{self.vacuum}.actuation_feedback',
                          f'{self.ADAPTER_ROW} provides no actuation_feedback')
        self.stage_cmd_topic = self.param('stage_cmd_topic', str, '')
        if not self.stage_cmd_topic:
            raise Refused('stage_cmd_topic', 'required, no default: the stage command topic '
                                             '(TSA fills it from the stage topic it detects)')
        self.stage_status_topic = self.param('stage_status_topic', str, '')
        self.scene_reset_topic = self.param('scene_reset_topic', str, '')
        self.republish_rate_hz = self.param('republish_rate_hz', float, 2.0, positive=True)
        v = dev.verification
        self.engage_timeout_ms = self.param('engage_timeout_ms', int, 2000, positive=True)
        if self.engage_timeout_ms > v.timeout_ms:
            raise Refused('engage_timeout_ms',
                          f'at most verification.timeout_ms ({v.timeout_ms}): VerifyGrasp must see the decision')
        self.status_timeout_ms = self.param('status_timeout_ms', int, 400, positive=True)
        if self.status_timeout_ms > v.max_age_ms:
            raise Refused('status_timeout_ms',
                          f'at most verification.max_age_ms ({v.max_age_ms}): a silent stage must '
                          'read UNKNOWN before a consumer judges its last sample fresh')
        self._commanded = False
        self._reset_level = False
        self._stage_resolved = ''
        self._status_resolved = ''
        self._status_arrival = 0          # steady ns of the newest stage status, 0 = none yet
        self._stage_note = ''
        source = core.Evidence.SIM_GROUND_TRUTH if self.stage_status_topic else None
        return ({self.site: source}, {self.site: self.status_timeout_ms * 1_000_000})

    def _create_backend(self) -> None:
        self._stage_pub = self._named('stage_cmd_topic', lambda: self.create_publisher(
            Bool, self.stage_cmd_topic, LATCHED_QOS))
        self._stage_resolved = self._stage_pub.topic_name
        self._stage_pub.publish(Bool(data=False))         # start released, as the bridge does
        group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / self.republish_rate_hz, self._republish,
                          callback_group=self._timer_group, clock=self._steady)
        if self.stage_status_topic:
            sub = self._named('stage_status_topic', lambda: self.create_subscription(
                String, self.stage_status_topic, self._on_stage_status, STATUS_QOS,
                callback_group=group))
            self._status_resolved = sub.topic_name
        if self.scene_reset_topic:
            self._named('scene_reset_topic', lambda: self.create_subscription(
                Bool, self.scene_reset_topic, self._on_reset, RESET_QOS, callback_group=group))

    def provided_capabilities(self) -> Set[Capability]:
        caps = {Capability.BINARY_ACTUATION, Capability.CONTINUOUS_STATE}
        if self.stage_status_topic:
            caps.update((Capability.GRASP_DETECTION, Capability.RELEASE_DETECTION))
        return caps

    # --- periodic ---------------------------------------------------------------------------------
    def _initial_observation(self, now: int) -> None:
        # The receipt of the adapter's own start-up command (released): an echo, no commander's.
        self.core.observe_actuator(self.vacuum, now, binary=Binary.RELEASED)
        self._check_subscriber()

    def _status_silent(self) -> bool:
        """No stage status for status_timeout_ms of WALL time (none yet counts as silent). The ROS
        clock cannot tell: a dead or paused stage stops /clock, and with it the sim-time silence."""
        return (self._status_arrival == 0
                or self.steady_ns() - self._status_arrival > self.status_timeout_ms * 1_000_000)

    def _check_subscriber(self) -> bool:
        """Is the stage there? With a status topic the stage must also be TALKING: a subscriber
        alone (a recorder, an echo) is not the stage. Without one, a subscriber is all there is to
        see (recorded as debt)."""
        if self._stage_pub.get_subscription_count() == 0:
            note = f'no stage subscriber on {self._stage_resolved}'
        elif self.stage_status_topic and self._status_silent():
            note = (f'no stage status on {self._status_resolved} for {self.status_timeout_ms} ms '
                    '(a subscriber alone is not the stage)')
        else:
            self._stage_note = ''
            self.core.set_condition(self.vacuum, None)
            return True
        self._stage_note = note
        self.core.set_condition(self.vacuum, ActuatorStatus.UNAVAILABLE, note)
        return False

    def _tick(self, now: int) -> None:
        if self.stage_status_topic and self._status_arrival:
            self.core.set_source_silent(self.site, self._status_silent())
        self._check_subscriber()

    def _republish(self) -> None:
        if self.closing:
            return
        with self._lock:
            self._stage_pub.publish(Bool(data=self._commanded))

    # --- commands ---------------------------------------------------------------------------------
    def _decide_binary(self, a: str, command: Command) -> Decision:
        if not self._check_subscriber():
            return Decision(False, True, self._stage_note)
        return Decision(True, True, 'engaged' if command == Command.ENGAGE else 'released')

    def _drive_binary(self, a: str, command: Command) -> None:
        self._commanded = command == Command.ENGAGE
        self._stage_pub.publish(Bool(data=self._commanded))

    # --- the stage ----------------------------------------------------------------------------------
    def _on_stage_status(self, msg: String) -> None:
        if self.closing:
            return
        failure = None
        with self._lock:
            self._status_arrival = self.steady_ns()
            now = self.now_ns()
            if not self._maybe_start() or now <= 0:
                return
            command, tc = self.core.last_command(self.vacuum)
            try:
                reading, note = map_stage_status(msg.data, command == Command.ENGAGE, now - tc,
                                                 self.engage_timeout_ms * 1_000_000)
            except Exception as e:  # noqa: BLE001 -- a raise here would stop the whole adapter
                failure = e
                reading = Reading.CANNOT_TELL
                note = f'malformed stage status {_quoted(msg.data)} ({type(e).__name__})'
            self.core.set_site_note(self.site, note)
            self.core.observe_site(self.site, reading, now)
            self._publish_changes()
        if failure is not None:
            self.get_logger().error(f'{self.site}: the stage status could not be read ({failure!r}): '
                                    'it reads as cannot tell', throttle_duration_sec=LOG_THROTTLE_S)
        elif reading == Reading.CANNOT_TELL:
            self.get_logger().warn(f'{self.site}: {note}: it reads as cannot tell',
                                   throttle_duration_sec=LOG_THROTTLE_S)

    def _on_reset(self, msg: Bool) -> None:
        if self.closing:
            return
        with self._lock:
            rising = bool(msg.data) and not self._reset_level
            self._reset_level = bool(msg.data)
            now = self.now_ns()
            if not rising or not self._maybe_start() or now <= 0:
                return
            self._drive_binary(self.vacuum, Command.RELEASE)
            self.core.apply(self.vacuum, Command.RELEASE, now)
            self._after_applied(self.vacuum, Command.RELEASE)
            self._publish_changes()
        self.get_logger().info(f'scene reset: recorded as an applied RELEASE of {self.vacuum}')


def main(args=None):
    sys.exit(run(IsaacEefAdapter, args))


if __name__ == '__main__':
    main()
