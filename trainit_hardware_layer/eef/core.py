"""The pure per-device state model of a TrainIt end-effector adapter (ADR-0022 D3, D8).

No ROS graph and no clock in here: every time is an integer of nanoseconds on the adapter's ROS
clock (sim time with ``use_sim_time``), handed in by the caller, and 0 means "no stamp". Both
adapters (``mock_eef_adapter``, ``isaac_eef_adapter``) drive the SAME model, so every rule below is
implemented once:

- ACTUATORS, in contract order. Each one has a status (UNKNOWN until observed; IDLE; MOVING while
  a goal it serves executes, or while a valve with ``actuation_feedback`` switches before its
  switch confirms; FAULT / UNAVAILABLE / UNKNOWN as a condition the backend reports), the APPLIED
  binary state of a ``set_bool`` actuator, ``binary_confirmed`` (only a switch ON the actuator sets
  it), the stamp of its newest observation, and the newest APPLIED command from ANY commander.
  A rejected command is never recorded as ``last_command``: ``detail`` names it.
- GRASP SITES, in contract order. A site's deciding source (none = UNAVAILABLE) reports readings
  (HELD, NOT_HELD, UNDECIDED, CANNOT_TELL) with their stamps; the model turns them into the
  published grasp:

  - UNAVAILABLE / NONE when the site has no deciding source;
  - UNKNOWN / NONE before the first reading, while the source is silent (no reading within its
    silence timeout), and while it reports CANNOT_TELL. A source that has sent nothing since the
    adapter started is named in ``detail`` ("<site>: no sample from its source yet") once its
    silence timeout has passed, at once without one;
  - PENDING after an applied engage of the site's grip until a reading at or after that engage
    decides (and while an UNDECIDED reading follows it). "After" is decided by arrival as well as
    by stamp: a reading the adapter received BEFORE it applied the engage predates it even when
    both carry the same stamp (a stepped simulation clock), so it is never published as a decision
    about that engage (ADR-0022 D4);
  - LOST, latched, when a decisive NOT_HELD follows a HELD that was observed while the grip was
    engaged, with no applied release of the grip since (from any commander; a scene reset counts
    as a release). The next applied engage or release of the grip clears it. A rejected release
    clears nothing;
  - otherwise the reading: HELD or NOT_HELD.

  Evidence is NONE if and only if the grasp is UNAVAILABLE or UNKNOWN; PENDING and LOST carry the
  deciding source's class and the stamp of its newest reading. A TrainIt adapter never publishes
  COMMAND_ECHO or SIM_ATTACHMENT: such a source is refused when the model is built.
- STATUS: FAULT > UNAVAILABLE > UNKNOWN > BUSY > READY over the actuators plus the adapter's
  tool-level fault and link loss. Sites never feed it.
- STAMPS: never re-stamped; ``header.stamp`` is the maximum of the site stamps and the actuator
  stamps.
- PUBLISHING (``PublishGate``): a sample on every change (a stamp advance included), plus a
  heartbeat that republishes the SAME sample, with the SAME stamps, only when nothing changed.

``StateSample.to_msg()`` builds ``trainit_interfaces/msg/EndEffectorState``; the message package is
imported only there, so the rules can be tested without ROS.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Dict, Iterable, Mapping, Optional, Tuple

from . import contract as _contract


# ------------------------------------------------------------------------------------------------
# The canonical tokens (checked token for token against the pinned eef_contract_tokens.txt)
# ------------------------------------------------------------------------------------------------

class Interface(IntEnum):
    """EndEffectorActuatorState.INTERFACE_*: the one standard interface of an actuator."""

    UNKNOWN = 0
    SET_BOOL = 1
    GRIPPER_COMMAND = 2
    FOLLOW_JOINT_TRAJECTORY = 3


class ToolStatus(IntEnum):
    """EndEffectorState.STATUS_*: the device's availability to act."""

    UNKNOWN = 0
    READY = 1
    BUSY = 2
    FAULT = 3
    UNAVAILABLE = 4


class ActuatorStatus(IntEnum):
    """EndEffectorActuatorState.STATUS_*."""

    UNKNOWN = 0
    IDLE = 1
    MOVING = 2
    FAULT = 3
    UNAVAILABLE = 4


class Binary(IntEnum):
    """EndEffectorActuatorState.BINARY_*: the APPLIED state of a set_bool actuator."""

    UNKNOWN = 0
    RELEASED = 1
    ENGAGED = 2


class Command(IntEnum):
    """EndEffectorActuatorState.COMMAND_*: the newest APPLIED command."""

    NONE = 0
    ENGAGE = 1
    RELEASE = 2
    TARGET = 3


class Grasp(IntEnum):
    """EndEffectorState.GRASP_*: the grasp observation of one site."""

    UNAVAILABLE = 0
    UNKNOWN = 1
    NOT_HELD = 2
    PENDING = 3
    HELD = 4
    LOST = 5


class Evidence(IntEnum):
    """EndEffectorState.EVIDENCE_*: the provenance of a site's grasp."""

    NONE = 0
    COMMAND_ECHO = 1
    ACTUATION_INFERENCE = 2
    SENSOR = 3
    SIM_GROUND_TRUTH = 4
    SIM_ATTACHMENT = 5


class Capability(str, Enum):
    """The closed capability vocabulary (ADR-0022 D6): no free strings."""

    BINARY_ACTUATION = 'binary_actuation'
    ACTUATION_FEEDBACK = 'actuation_feedback'
    GRIPPER_CONTROL = 'gripper_control'
    POSITION_CONTROL = 'position_control'
    EFFORT_CONTROL = 'effort_control'
    JOINT_POSITION_CONTROL = 'joint_position_control'
    TRAJECTORY_CONTROL = 'trajectory_control'
    GRASP_DETECTION = 'grasp_detection'
    RELEASE_DETECTION = 'release_detection'
    CONTINUOUS_STATE = 'continuous_state'


INTERFACE_BY_NAME: Mapping[str, Interface] = {
    'set_bool': Interface.SET_BOOL,
    'gripper_command': Interface.GRIPPER_COMMAND,
    'follow_joint_trajectory': Interface.FOLLOW_JOINT_TRAJECTORY,
}

# The evidence classes a contract may accept, and the only classes a TrainIt adapter publishes.
EVIDENCE_BY_NAME: Mapping[str, Evidence] = {
    'actuation_inference': Evidence.ACTUATION_INFERENCE,
    'sensor': Evidence.SENSOR,
    'sim_ground_truth': Evidence.SIM_GROUND_TRUTH,
}
ADAPTER_EVIDENCE = frozenset(EVIDENCE_BY_NAME.values())

# The capabilities a command interface implies (ADR-0022 D6 table).
IMPLIED_CAPABILITIES: Mapping[str, Tuple[Capability, ...]] = {
    'set_bool': (Capability.BINARY_ACTUATION,),
    'gripper_command': (Capability.GRIPPER_CONTROL, Capability.POSITION_CONTROL),
    'follow_joint_trajectory': (Capability.JOINT_POSITION_CONTROL, Capability.TRAJECTORY_CONTROL),
}

# The token groups of eef_contract_tokens.txt this module owns, as {group: {name: value}}.
TOKEN_ENUMS = {
    'command_interface': Interface,
    'tool_status': ToolStatus,
    'actuator_status': ActuatorStatus,
    'binary': Binary,
    'command': Command,
    'grasp': Grasp,
    'evidence': Evidence,
}


def token_table(enum_cls) -> Dict[str, int]:
    """{token name: value} of one token enum, the token file's spelling (lower case)."""
    return {m.name.lower(): int(m) for m in enum_cls}


class Reading(Enum):
    """What a site's deciding source reports at one instant."""

    HELD = 'held'
    NOT_HELD = 'not_held'
    UNDECIDED = 'undecided'      # alive, has not decided (e.g. a SurfaceGripper still Closing)
    CANNOT_TELL = 'cannot_tell'  # alive, but this sample says nothing valid


# ------------------------------------------------------------------------------------------------
# Status aggregation
# ------------------------------------------------------------------------------------------------

TOOL_PRECEDENCE = (ToolStatus.FAULT, ToolStatus.UNAVAILABLE, ToolStatus.UNKNOWN, ToolStatus.BUSY,
                   ToolStatus.READY)
_TOOL_OF = {
    ActuatorStatus.FAULT: ToolStatus.FAULT,
    ActuatorStatus.UNAVAILABLE: ToolStatus.UNAVAILABLE,
    ActuatorStatus.UNKNOWN: ToolStatus.UNKNOWN,
    ActuatorStatus.MOVING: ToolStatus.BUSY,
    ActuatorStatus.IDLE: ToolStatus.READY,
}


def aggregate_status(actuator_statuses: Iterable[int], tool_fault: bool = False,
                     link_lost: bool = False) -> ToolStatus:
    """FAULT > UNAVAILABLE > UNKNOWN > BUSY > READY over the actuators and the tool-level flags.

    A value outside the actuator vocabulary reads as UNKNOWN; no actuator at all is UNKNOWN.
    """
    seen = set()
    for s in actuator_statuses:
        try:
            seen.add(_TOOL_OF[ActuatorStatus(s)])
        except ValueError:
            seen.add(ToolStatus.UNKNOWN)
    if tool_fault:
        seen.add(ToolStatus.FAULT)
    if link_lost:
        seen.add(ToolStatus.UNAVAILABLE)
    for status in TOOL_PRECEDENCE:
        if status in seen:
            return status
    return ToolStatus.UNKNOWN


# ------------------------------------------------------------------------------------------------
# Samples (immutable: equality is the change detector) and the message builder
# ------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class ActuatorSample:
    actuator: str
    command_interface: int
    joints: Tuple[str, ...]
    status: int
    binary: int
    binary_confirmed: bool
    stamp: int
    last_command: int
    last_command_stamp: int


@dataclass(frozen=True)
class StateSample:
    """One EndEffectorState, as plain values."""

    end_effector: str
    adapter: str
    stamp: int
    status: int
    sites: Tuple[str, ...]
    site_grip: Tuple[str, ...]
    site_grasp: Tuple[int, ...]
    site_evidence: Tuple[int, ...]
    site_stamp: Tuple[int, ...]
    actuators: Tuple[ActuatorSample, ...]
    detail: str

    def site(self, site_id: str) -> Tuple[int, int, int]:
        """(grasp, evidence, stamp) of one site."""
        i = self.sites.index(site_id)
        return self.site_grasp[i], self.site_evidence[i], self.site_stamp[i]

    def actuator(self, actuator_id: str) -> ActuatorSample:
        for a in self.actuators:
            if a.actuator == actuator_id:
                return a
        raise KeyError(actuator_id)

    def to_msg(self):
        """The trainit_interfaces/msg/EndEffectorState of this sample."""
        from trainit_interfaces.msg import EndEffectorActuatorState, EndEffectorState
        msg = EndEffectorState()
        msg.header.stamp = time_msg(self.stamp)
        msg.header.frame_id = ''
        msg.end_effector = self.end_effector
        msg.adapter = self.adapter
        msg.status = self.status
        msg.sites = list(self.sites)
        msg.site_grip = list(self.site_grip)
        msg.site_grasp = list(self.site_grasp)
        msg.site_evidence = list(self.site_evidence)
        msg.site_stamp = [time_msg(t) for t in self.site_stamp]
        out = []
        for a in self.actuators:
            m = EndEffectorActuatorState()
            m.actuator = a.actuator
            m.command_interface = a.command_interface
            m.joints = list(a.joints)
            m.status = a.status
            m.binary = a.binary
            m.binary_confirmed = a.binary_confirmed
            m.stamp = time_msg(a.stamp)
            m.last_command = a.last_command
            m.last_command_stamp = time_msg(a.last_command_stamp)
            out.append(m)
        msg.actuators = out
        msg.detail = self.detail
        return msg


def time_msg(ns: int):
    """builtin_interfaces/Time of a nanosecond stamp (0 stays the zero time)."""
    from builtin_interfaces.msg import Time
    return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def stamp_ns(t) -> int:
    """Nanoseconds of a builtin_interfaces/Time."""
    return int(t.sec) * 1_000_000_000 + int(t.nanosec)


# ------------------------------------------------------------------------------------------------
# The model
# ------------------------------------------------------------------------------------------------

class _ActuatorState:

    def __init__(self, spec: _contract.Actuator):
        self.spec = spec
        self.interface = INTERFACE_BY_NAME[spec.command_interface]
        self.observed = False
        self.condition: Optional[ActuatorStatus] = None
        self.condition_note = ''
        self.active = False
        self.binary = Binary.UNKNOWN
        self.binary_confirmed = False
        self.stamp = 0
        self.last_command = Command.NONE
        self.last_command_stamp = 0
        self.last_command_seq = 0   # the model's arrival order of that command
        self.rejected = ''

    def status(self) -> ActuatorStatus:
        if self.condition is not None:
            return self.condition
        if not self.observed:
            return ActuatorStatus.UNKNOWN
        return ActuatorStatus.MOVING if self.active else ActuatorStatus.IDLE


class _SiteState:

    def __init__(self, spec: _contract.Site, source: Optional[Evidence], silence_ns: int):
        self.spec = spec
        self.source = source
        self.silence_ns = silence_ns
        self.reading: Optional[Reading] = None
        self.reading_stamp = 0
        self.reading_seq = 0        # the model's arrival order of that reading
        self.silent = False
        self.wall_silent = False  # silent on the adapter's wall clock (a stopped simulation clock)
        self.unheard = False     # no reading yet, and its silence window since the start has passed
        self.lost = False
        self.armed = False
        self.note = ''


def _check_stamp(stamp: int) -> None:
    if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0:
        raise ValueError(f'a stamp is a positive integer of nanoseconds, not {stamp!r}')


class DeviceModel:
    """The state of ONE device, as its adapter observes it (not thread-safe: the node serialises)."""

    def __init__(self, device: _contract.Device, end_effector: str, adapter: str,
                 site_sources: Mapping[str, Optional[Evidence]],
                 silence_ns: Optional[Mapping[str, int]] = None):
        if set(site_sources) != set(device.site_ids):
            raise ValueError(f'one deciding-source entry per site {list(device.site_ids)}, '
                             f'got {sorted(site_sources)}')
        for s, source in site_sources.items():
            if source is not None and source not in ADAPTER_EVIDENCE:
                raise ValueError(
                    f'site {s}: a TrainIt adapter publishes only {sorted(e.name for e in ADAPTER_EVIDENCE)}'
                    f' evidence, never {Evidence(source).name} (ADR-0022 D3)')
        silence_ns = dict(silence_ns or {})
        self.device = device
        self.end_effector = end_effector
        self.adapter = adapter
        self._actuators = {a: _ActuatorState(device.actuators[a]) for a in device.actuator_ids}
        self._sites = {s: _SiteState(device.sites[s], site_sources[s], int(silence_ns.get(s, 0)))
                       for s in device.site_ids}
        self._site_of_grip = {st.spec.grip: st for st in self._sites.values()}
        self._tool_fault = ''
        self._link_lost = ''
        self._start = 0
        self._seq = 0   # arrival order of applied commands and site readings

    # --- lookups ----------------------------------------------------------------------------------
    def _act(self, a: str) -> _ActuatorState:
        try:
            return self._actuators[a]
        except KeyError:
            raise KeyError(f'{self.end_effector}: no actuator {a!r}') from None

    def _site(self, s: str) -> _SiteState:
        try:
            return self._sites[s]
        except KeyError:
            raise KeyError(f'{self.end_effector}: no site {s!r}') from None

    def last_command(self, a: str) -> Tuple[Command, int]:
        st = self._act(a)
        return st.last_command, st.last_command_stamp

    def binary(self, a: str) -> Binary:
        return self._act(a).binary

    def grip_engaged(self, s: str) -> bool:
        """True when the newest applied command of the site's grip is ENGAGE."""
        return self._act(self._site(s).spec.grip).last_command == Command.ENGAGE

    def site_source(self, s: str) -> Optional[Evidence]:
        return self._site(s).source

    def is_lost_latched(self, s: str) -> bool:
        return self._site(s).lost

    # --- actuators --------------------------------------------------------------------------------
    def observe_actuator(self, a: str, stamp: int, binary: Optional[Binary] = None) -> None:
        """An observation of the actuator at `stamp` (its server seen, an emulated initial state)."""
        _check_stamp(stamp)
        st = self._act(a)
        st.observed = True
        st.stamp = stamp
        if binary is not None:
            if st.interface != Interface.SET_BOOL:
                raise ValueError(f'{a}: only a set_bool actuator has a binary state')
            st.binary = Binary(binary)

    def apply(self, a: str, command: Command, stamp: int) -> None:
        """An APPLIED command (SetBool success:true, or an accepted goal), from ANY commander.

        A set_bool actuator's binary follows it (an echo, stamped with the receipt; a valve with
        ``actuation_feedback`` is MOVING until its switch confirms). An action actuator is MOVING
        from the accepted goal (a goal event, stamped) until ``goal_done``. An ENGAGE or RELEASE of
        a site's grip clears that site's LOST latch and disarms it.
        """
        _check_stamp(stamp)
        command = Command(command)
        if command == Command.NONE:
            raise ValueError('COMMAND_NONE is not a command')
        st = self._act(a)
        if st.interface == Interface.SET_BOOL:
            if command not in (Command.ENGAGE, Command.RELEASE):
                raise ValueError(f'{a}: a set_bool actuator takes ENGAGE or RELEASE, not {command.name}')
            st.binary = Binary.ENGAGED if command == Command.ENGAGE else Binary.RELEASED
            st.binary_confirmed = False
            st.active = st.spec.actuation_feedback
        else:
            st.active = True
        st.observed = True
        st.stamp = stamp
        st.last_command = command
        st.last_command_stamp = stamp
        self._seq += 1
        st.last_command_seq = self._seq
        st.rejected = ''
        site = self._site_of_grip.get(a)
        if site is not None and command in (Command.ENGAGE, Command.RELEASE):
            site.lost = False
            site.armed = False

    def reject(self, a: str, command: Command, message: str) -> None:
        """A command that was NOT applied: named in ``detail``, never in ``last_command``."""
        st = self._act(a)
        st.rejected = f'{Command(command).name} rejected: {message}'

    def observe_switch(self, a: str, reports: Binary, stamp: int) -> None:
        """A sample of the switch ON a set_bool actuator with ``actuation_feedback``."""
        _check_stamp(stamp)
        st = self._act(a)
        if st.interface != Interface.SET_BOOL or not st.spec.actuation_feedback:
            raise ValueError(f'{a}: only a set_bool actuator with actuation_feedback has a switch')
        st.observed = True
        st.stamp = stamp
        st.binary_confirmed = st.binary != Binary.UNKNOWN and Binary(reports) == st.binary
        if st.binary_confirmed:
            st.active = False

    def goal_done(self, a: str, stamp: int) -> None:
        """The terminal event of a goal this adapter serves: the actuator is IDLE again."""
        _check_stamp(stamp)
        st = self._act(a)
        st.observed = True
        st.active = False
        st.stamp = stamp

    def set_condition(self, a: str, status: Optional[ActuatorStatus], note: str = '') -> None:
        """FAULT / UNAVAILABLE / UNKNOWN reported by the backend (None clears it), with its cause."""
        if status is not None and ActuatorStatus(status) not in (
                ActuatorStatus.FAULT, ActuatorStatus.UNAVAILABLE, ActuatorStatus.UNKNOWN):
            raise ValueError(f'{a}: a condition is FAULT, UNAVAILABLE or UNKNOWN')
        st = self._act(a)
        st.condition = None if status is None else ActuatorStatus(status)
        st.condition_note = note if status is not None else ''

    # --- sites ------------------------------------------------------------------------------------
    def observe_site(self, s: str, reading: Reading, stamp: int) -> None:
        """A reading of the site's deciding source, stamped when it was observed."""
        _check_stamp(stamp)
        site = self._site(s)
        if site.source is None:
            raise ValueError(f'{s}: this site has no deciding source (UNAVAILABLE)')
        reading = Reading(reading)
        grip = self._act(site.spec.grip)
        engaged = grip.last_command == Command.ENGAGE
        site.reading = reading
        site.reading_stamp = stamp
        self._seq += 1
        site.reading_seq = self._seq
        site.silent = False
        site.wall_silent = False
        site.unheard = False
        if reading == Reading.HELD:
            if engaged and not site.lost and stamp >= grip.last_command_stamp:
                site.armed = True
        elif reading == Reading.NOT_HELD:
            if site.armed and not site.lost:
                site.lost = True
                site.armed = False

    def set_site_note(self, s: str, note: str) -> None:
        self._site(s).note = note

    def set_source_silent(self, s: str, silent: bool) -> None:
        """The adapter's own WALL-clock silence of a site's source. ``tick`` measures silence on the
        adapter's ROS clock, which stops with a simulation (a dead or paused stage freezes /clock):
        a source that sends nothing for its silence timeout of real time reads UNKNOWN too, never
        its last decision. The next reading clears it."""
        self._site(s).wall_silent = bool(silent)

    def start(self, stamp: int) -> None:
        """The adapter started observing (its initial observation); the first call counts.

        A site whose source then says nothing for longer than its silence timeout (at once without
        one) is named in ``detail``; its grasp stays UNKNOWN / NONE and the status does not move.
        """
        _check_stamp(stamp)
        if self._start == 0:
            self._start = stamp
            self.tick(stamp)

    def tick(self, now: int) -> None:
        """Silence detection: a source with a silence timeout and no reading within it is silent;
        one with no reading at all is unheard once that timeout has passed since ``start`` (at
        once without a timeout)."""
        for site in self._sites.values():
            if site.source is not None and site.reading is None:
                site.unheard = self._start > 0 and (
                    site.silence_ns == 0 or now - self._start > site.silence_ns)
            if site.silence_ns > 0 and site.reading is not None:
                site.silent = now - site.reading_stamp > site.silence_ns

    # --- tool -------------------------------------------------------------------------------------
    def set_tool_fault(self, note: str) -> None:
        """A tool-level fault ('' clears it): status FAULT, ``detail`` says which."""
        self._tool_fault = note

    def set_link_lost(self, note: str) -> None:
        """The adapter lost its backend link ('' clears it): status UNAVAILABLE."""
        self._link_lost = note

    # --- the sample -------------------------------------------------------------------------------
    def _view(self, site: _SiteState) -> Tuple[Grasp, Evidence, int]:
        if site.source is None:
            return Grasp.UNAVAILABLE, Evidence.NONE, 0
        if site.reading is None:
            return Grasp.UNKNOWN, Evidence.NONE, 0
        stamp = site.reading_stamp
        if site.silent or site.wall_silent or site.reading == Reading.CANNOT_TELL:
            return Grasp.UNKNOWN, Evidence.NONE, stamp
        if site.lost:
            return Grasp.LOST, site.source, stamp
        grip = self._act(site.spec.grip)
        engaged = grip.last_command == Command.ENGAGE
        # a reading older than the engage, by stamp or by arrival, decides nothing about it
        if engaged and (stamp < grip.last_command_stamp or site.reading_seq < grip.last_command_seq):
            return Grasp.PENDING, site.source, stamp
        if site.reading == Reading.HELD:
            return Grasp.HELD, site.source, stamp
        if site.reading == Reading.NOT_HELD:
            return Grasp.NOT_HELD, site.source, stamp
        if engaged:  # UNDECIDED after the engage
            return Grasp.PENDING, site.source, stamp
        return Grasp.UNKNOWN, Evidence.NONE, stamp

    def _detail(self) -> str:
        parts = []
        for a in self.device.actuator_ids:
            st = self._actuators[a]
            if st.condition_note:
                parts.append(f'{a}: {st.condition_note}')
            if st.rejected:
                parts.append(f'{a}: {st.rejected}')
        for s in self.device.site_ids:
            site = self._sites[s]
            if site.note:
                parts.append(f'{s}: {site.note}')
            if site.silent or site.wall_silent:
                parts.append(f'{s}: no deciding-source sample for more than '
                             f'{site.silence_ns // 1_000_000} ms')
            if site.unheard:
                parts.append(f'{s}: no sample from its source yet')
        if self._tool_fault:
            parts.append(self._tool_fault)
        if self._link_lost:
            parts.append(self._link_lost)
        return '; '.join(parts)

    def sample(self) -> StateSample:
        acts = []
        for a in self.device.actuator_ids:
            st = self._actuators[a]
            acts.append(ActuatorSample(
                actuator=a, command_interface=int(st.interface), joints=tuple(st.spec.joints),
                status=int(st.status()), binary=int(st.binary), binary_confirmed=st.binary_confirmed,
                stamp=st.stamp, last_command=int(st.last_command),
                last_command_stamp=st.last_command_stamp))
        grasp, evidence, stamps = [], [], []
        for s in self.device.site_ids:
            g, e, t = self._view(self._sites[s])
            grasp.append(int(g))
            evidence.append(int(e))
            stamps.append(t)
        return StateSample(
            end_effector=self.end_effector,
            adapter=self.adapter,
            stamp=max(stamps + [a.stamp for a in acts], default=0),
            status=int(aggregate_status((a.status for a in acts), bool(self._tool_fault),
                                        bool(self._link_lost))),
            sites=tuple(self.device.site_ids),
            site_grip=tuple(self.device.sites[s].grip for s in self.device.site_ids),
            site_grasp=tuple(grasp),
            site_evidence=tuple(evidence),
            site_stamp=tuple(stamps),
            actuators=tuple(acts),
            detail=self._detail(),
        )


# ------------------------------------------------------------------------------------------------
# The publish rule
# ------------------------------------------------------------------------------------------------

class PublishGate:
    """When to publish: on every change, plus a heartbeat of the unchanged sample.

    ``offer`` is called after every mutation; it returns the sample when it differs from the last
    one published (a stamp advance is a difference). ``heartbeat`` is called by the heartbeat
    timer; it publishes a pending change if there is one, and otherwise republishes the LAST
    sample, unchanged (its stamps included), when that sample is at least half a period old: a
    change published just before the tick is not repeated at once. Times are a monotonic clock in
    nanoseconds (the steady clock), never the stamps' clock.
    """

    def __init__(self, heartbeat_period_ns: int):
        if heartbeat_period_ns <= 0:
            raise ValueError('the heartbeat period is positive')
        self.period_ns = int(heartbeat_period_ns)
        self._last: Optional[StateSample] = None
        self._last_ns = 0

    @property
    def last(self) -> Optional[StateSample]:
        return self._last

    def offer(self, sample: StateSample, now_ns: int) -> Optional[StateSample]:
        if sample == self._last:
            return None
        self._last = sample
        self._last_ns = now_ns
        return sample

    def heartbeat(self, sample: StateSample, now_ns: int) -> Optional[StateSample]:
        if sample != self._last:
            return self.offer(sample, now_ns)
        if now_ns - self._last_ns >= self.period_ns // 2:
            self._last_ns = now_ns
            return self._last
        return None
