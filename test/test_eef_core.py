"""H1 -- core purity (ADR-0022 D3, D8): the per-device model, with no ROS graph and no clock.

Covered: the status precedence; the LOST machine (an applied release disarms it from any commander,
a rejected one does not, a scene reset counts as a release, the next applied engage or release
clears it); PENDING and silence, and a source never heard from since the start, named in detail;
evidence NONE if and only if UNAVAILABLE or UNKNOWN; COMMAND_ECHO
and SIM_ATTACHMENT never published; the publish rule (a sample on every change, a stamp advance
included; the heartbeat only when nothing changed, with the same stamps); "cannot tell"; and a
seeded random walk that checks every invariant after each of 6000 operations.
"""
import itertools
import random

import pytest

import eef_harness as h
from trainit_hardware_layer.eef import contract, core
from trainit_hardware_layer.eef.core import ActuatorStatus as AS
from trainit_hardware_layer.eef.core import Binary, Command, Evidence, Grasp, Reading
from trainit_hardware_layer.eef.core import ToolStatus as TS

DEVICES = contract.devices(contract.parse(h.fixture_flat()))
SENSOR = Evidence.SENSOR
SEC = 1_000_000_000
MS = 1_000_000


def composite(sources=None, silence=None):
    return core.DeviceModel(DEVICES['composite_tool'], '/r1/eef/composite_tool', 'trainit_mock',
                            sources or {'site_a': SENSOR, 'site_b': SENSOR}, silence)


def suction(source=SENSOR, silence_ms=0):
    return core.DeviceModel(DEVICES['suction_simple'], '/eef/suction_simple', 'trainit_mock',
                            {'main': source}, {'main': silence_ms * MS})


def view(model, s='main'):
    return model.sample().site(s)


def engaged_and_held(model, t0=1):
    """vacuum engaged at t0 s, HELD at t0+1 s."""
    model.observe_actuator('vacuum', t0 * SEC, binary=Binary.RELEASED)
    model.apply('vacuum', Command.ENGAGE, t0 * SEC + 500 * MS)
    model.observe_site('main', Reading.HELD, (t0 + 1) * SEC)
    assert view(model)[0] == Grasp.HELD


# ------------------------------------------------------------------------------------------------
# status
# ------------------------------------------------------------------------------------------------

def test_status_precedence_over_every_pair_and_flag():
    order = [AS.FAULT, AS.UNAVAILABLE, AS.UNKNOWN, AS.MOVING, AS.IDLE]
    tool = {AS.FAULT: TS.FAULT, AS.UNAVAILABLE: TS.UNAVAILABLE, AS.UNKNOWN: TS.UNKNOWN,
            AS.MOVING: TS.BUSY, AS.IDLE: TS.READY}
    for combo in itertools.product(AS, repeat=3):
        worst = min(combo, key=order.index)
        assert core.aggregate_status(combo) == tool[worst], combo
        assert core.aggregate_status(combo, tool_fault=True) == TS.FAULT
        expected = TS.FAULT if worst == AS.FAULT else TS.UNAVAILABLE
        assert core.aggregate_status(combo, link_lost=True) == expected
    assert core.aggregate_status([AS.IDLE, 99]) == TS.UNKNOWN     # an unknown value is UNKNOWN
    assert core.aggregate_status([]) == TS.UNKNOWN


def test_actuator_conditions_drive_the_status_and_the_detail_names_them():
    m = composite()
    for a in m.device.actuator_ids:
        m.observe_actuator(a, SEC, binary=Binary.RELEASED if a in ('gripper_b', 'binary_lock') else None)
    assert m.sample().status == TS.READY
    m.apply('linear_axis', Command.TARGET, 2 * SEC)
    assert m.sample().status == TS.BUSY
    m.set_condition('binary_lock', AS.FAULT, 'the lock reports a fault')
    s = m.sample()
    assert s.status == TS.FAULT
    assert s.actuator('binary_lock').status == AS.FAULT
    assert s.detail == 'binary_lock: the lock reports a fault'
    m.set_condition('gripper_a', AS.UNAVAILABLE, 'no server')
    assert m.sample().status == TS.FAULT                       # FAULT > UNAVAILABLE
    m.set_condition('binary_lock', None)
    assert m.sample().status == TS.UNAVAILABLE
    assert m.sample().detail == 'gripper_a: no server'
    m.set_condition('gripper_a', None)
    m.goal_done('linear_axis', 3 * SEC)
    assert m.sample().status == TS.READY and m.sample().detail == ''
    m.set_tool_fault('vendor I/O reports a tool fault')
    assert m.sample().status == TS.FAULT
    m.set_tool_fault('')
    m.set_link_lost('the backend link is down')
    assert m.sample().status == TS.UNAVAILABLE
    with pytest.raises(ValueError):
        m.set_condition('gripper_a', AS.MOVING)


def test_sites_never_feed_the_status():
    m = suction()
    m.observe_actuator('vacuum', SEC, binary=Binary.RELEASED)
    for reading in Reading:
        m.observe_site('main', reading, 2 * SEC)
        assert m.sample().status == TS.READY
    m.apply('vacuum', Command.ENGAGE, 3 * SEC)
    m.observe_site('main', Reading.HELD, 4 * SEC)
    m.observe_site('main', Reading.NOT_HELD, 5 * SEC)
    assert view(m)[0] == Grasp.LOST
    assert m.sample().status == TS.READY


# ------------------------------------------------------------------------------------------------
# "cannot tell"
# ------------------------------------------------------------------------------------------------

def _reads_ready_held_or_released(status, grasps):
    return status in (TS.READY, TS.BUSY) or any(g in (Grasp.HELD, Grasp.NOT_HELD) for g in grasps)


def test_a_model_that_observed_nothing_cannot_tell():
    m = composite({'site_a': SENSOR, 'site_b': None})
    s = m.sample()
    assert s.status == TS.UNKNOWN and s.stamp == 0 and s.detail == ''
    for a in s.actuators:
        assert (a.status, a.binary, a.binary_confirmed, a.stamp, a.last_command, a.last_command_stamp) == (
            AS.UNKNOWN, Binary.UNKNOWN, False, 0, Command.NONE, 0)
    assert s.site_grasp == (Grasp.UNKNOWN, Grasp.UNAVAILABLE)
    assert s.site_evidence == (Evidence.NONE, Evidence.NONE) and s.site_stamp == (0, 0)
    assert not _reads_ready_held_or_released(s.status, s.site_grasp)
    msg = s.to_msg()
    assert not _reads_ready_held_or_released(msg.status, list(msg.site_grasp))


def test_a_default_constructed_message_cannot_tell():
    from trainit_interfaces.msg import EndEffectorState
    m = EndEffectorState()
    assert m.end_effector == '' and m.status == EndEffectorState.STATUS_UNKNOWN
    assert not m.sites and not m.actuators and not m.site_grasp
    assert not _reads_ready_held_or_released(m.status, list(m.site_grasp))


# ------------------------------------------------------------------------------------------------
# the LOST machine
# ------------------------------------------------------------------------------------------------

def test_a_drop_with_no_release_is_lost_and_stays_latched():
    m = suction()
    m.observe_actuator('vacuum', 1 * SEC, binary=Binary.RELEASED)
    m.observe_site('main', Reading.NOT_HELD, 2 * SEC)
    assert view(m) == (Grasp.NOT_HELD, SENSOR, 2 * SEC)
    m.apply('vacuum', Command.ENGAGE, 3 * SEC)
    assert view(m) == (Grasp.PENDING, SENSOR, 2 * SEC)       # no reading since the engage: old
    m.observe_site('main', Reading.UNDECIDED, 4 * SEC)
    assert view(m) == (Grasp.PENDING, SENSOR, 4 * SEC)
    m.observe_site('main', Reading.HELD, 5 * SEC)
    assert view(m) == (Grasp.HELD, SENSOR, 5 * SEC)
    m.observe_site('main', Reading.NOT_HELD, 6 * SEC)
    assert view(m) == (Grasp.LOST, SENSOR, 6 * SEC)
    m.observe_site('main', Reading.HELD, 7 * SEC)            # latched: still LOST
    assert view(m) == (Grasp.LOST, SENSOR, 7 * SEC)
    m.observe_site('main', Reading.NOT_HELD, 8 * SEC)
    assert view(m) == (Grasp.LOST, SENSOR, 8 * SEC)
    m.apply('vacuum', Command.RELEASE, 9 * SEC)              # the next applied release clears it
    assert view(m) == (Grasp.NOT_HELD, SENSOR, 8 * SEC)
    m.observe_site('main', Reading.NOT_HELD, 10 * SEC)
    assert view(m) == (Grasp.NOT_HELD, SENSOR, 10 * SEC)


def test_the_next_applied_engage_clears_lost():
    m = suction()
    engaged_and_held(m)
    m.observe_site('main', Reading.NOT_HELD, 3 * SEC)
    assert view(m)[0] == Grasp.LOST
    m.apply('vacuum', Command.ENGAGE, 4 * SEC)
    assert view(m) == (Grasp.PENDING, SENSOR, 3 * SEC)
    m.observe_site('main', Reading.NOT_HELD, 5 * SEC)       # a failed re-grip: not a second loss
    assert view(m)[0] == Grasp.NOT_HELD


def test_an_applied_release_from_any_commander_explains_the_drop():
    m = suction()
    engaged_and_held(m)
    m.apply('vacuum', Command.RELEASE, 3 * SEC)              # e.g. a foreign SetBool client
    m.observe_site('main', Reading.HELD, 4 * SEC)            # the sensor's lag
    assert view(m)[0] == Grasp.HELD
    m.observe_site('main', Reading.NOT_HELD, 5 * SEC)
    assert view(m)[0] == Grasp.NOT_HELD


def test_a_rejected_release_does_not_disarm_lost():
    m = suction()
    engaged_and_held(m)
    m.reject('vacuum', Command.RELEASE, 'no stage subscriber on /r3/stage/cmd')
    s = m.sample()
    assert s.actuator('vacuum').last_command == Command.ENGAGE
    assert s.detail == 'vacuum: RELEASE rejected: no stage subscriber on /r3/stage/cmd'
    m.observe_site('main', Reading.NOT_HELD, 3 * SEC)
    assert view(m)[0] == Grasp.LOST
    assert m.sample().detail == 'vacuum: RELEASE rejected: no stage subscriber on /r3/stage/cmd'
    m.apply('vacuum', Command.RELEASE, 4 * SEC)             # applied now: LOST cleared, note gone
    assert view(m)[0] == Grasp.NOT_HELD and m.sample().detail == ''


def test_a_scene_reset_counts_as_a_release():
    m = suction()
    engaged_and_held(m)
    m.apply('vacuum', Command.RELEASE, 3 * SEC)              # the reset, recorded as a release
    m.observe_site('main', Reading.NOT_HELD, 4 * SEC)
    assert view(m)[0] == Grasp.NOT_HELD


def test_a_failed_grasp_is_not_held_not_lost():
    m = suction()
    m.observe_actuator('vacuum', SEC, binary=Binary.RELEASED)
    m.apply('vacuum', Command.ENGAGE, 2 * SEC)
    m.observe_site('main', Reading.UNDECIDED, 3 * SEC)
    m.observe_site('main', Reading.NOT_HELD, 4 * SEC)
    assert view(m) == (Grasp.NOT_HELD, SENSOR, 4 * SEC)


def test_held_with_no_engage_never_arms_lost():
    m = suction()
    m.observe_actuator('vacuum', SEC, binary=Binary.RELEASED)
    m.observe_site('main', Reading.HELD, 2 * SEC)
    m.observe_site('main', Reading.NOT_HELD, 3 * SEC)
    assert view(m)[0] == Grasp.NOT_HELD


def test_a_target_command_is_not_a_release():
    m = core.DeviceModel(DEVICES['jaw_simple'], '/eef/jaw_simple', 'trainit_mock', {'main': SENSOR})
    m.apply('gripper', Command.ENGAGE, SEC)
    m.observe_site('main', Reading.HELD, 2 * SEC)
    m.apply('gripper', Command.TARGET, 3 * SEC)
    m.observe_site('main', Reading.NOT_HELD, 4 * SEC)
    assert view(m)[0] == Grasp.LOST


def test_lost_reads_unknown_while_its_source_is_silent_and_comes_back():
    m = suction(silence_ms=500)
    engaged_and_held(m)
    m.observe_site('main', Reading.NOT_HELD, 3 * SEC)
    m.tick(3 * SEC + 400 * MS)
    assert view(m)[0] == Grasp.LOST
    m.tick(3 * SEC + 600 * MS)
    assert view(m) == (Grasp.UNKNOWN, Evidence.NONE, 3 * SEC)
    assert m.sample().detail == 'main: no deciding-source sample for more than 500 ms'
    m.observe_site('main', Reading.NOT_HELD, 4 * SEC)
    assert view(m) == (Grasp.LOST, SENSOR, 4 * SEC) and m.sample().detail == ''


def test_a_source_that_never_spoke_is_named_once_its_silence_window_has_passed():
    m = suction(silence_ms=400)
    m.observe_actuator('vacuum', SEC, binary=Binary.RELEASED)
    m.tick(10 * SEC)                                         # not started: nothing is late yet
    assert m.sample().detail == ''
    m.start(SEC)
    m.start(5 * SEC)                                         # only the first start counts
    m.tick(SEC + 400 * MS)                                   # within the window
    assert m.sample().detail == ''
    m.tick(SEC + 401 * MS)
    s = m.sample()
    assert s.detail == 'main: no sample from its source yet'
    assert s.status == TS.READY and view(m) == (Grasp.UNKNOWN, Evidence.NONE, 0)   # nothing else moves
    m.observe_site('main', Reading.NOT_HELD, 2 * SEC)
    assert m.sample().detail == '' and view(m) == (Grasp.NOT_HELD, SENSOR, 2 * SEC)
    m.tick(3 * SEC)                                          # from now on the silence rule speaks
    assert m.sample().detail == 'main: no deciding-source sample for more than 400 ms'


def test_a_source_that_never_spoke_without_a_silence_timeout_is_named_at_once():
    m = suction(silence_ms=0)
    m.observe_actuator('vacuum', SEC, binary=Binary.RELEASED)
    m.start(SEC)
    assert m.sample().detail == 'main: no sample from its source yet'
    m.tick(SEC)
    assert m.sample().detail == 'main: no sample from its source yet' and m.sample().status == TS.READY
    m.observe_site('main', Reading.UNDECIDED, 2 * SEC)       # any reading is a sample
    m.tick(9 * SEC)
    assert m.sample().detail == ''
    # a site with no deciding source is UNAVAILABLE, never "not heard from"
    none = suction(source=None)
    none.start(SEC)
    none.tick(9 * SEC)
    assert none.sample().detail == ''


def test_pending_turns_unknown_when_its_deciding_source_goes_silent():
    m = suction(silence_ms=500)
    m.observe_actuator('vacuum', SEC, binary=Binary.RELEASED)
    m.apply('vacuum', Command.ENGAGE, 2 * SEC)
    m.observe_site('main', Reading.UNDECIDED, 3 * SEC)
    assert view(m)[0] == Grasp.PENDING
    m.tick(3 * SEC + 501 * MS)
    assert view(m) == (Grasp.UNKNOWN, Evidence.NONE, 3 * SEC)


@pytest.mark.parametrize('before', [Reading.NOT_HELD, Reading.HELD])
def test_a_reading_received_before_the_engage_decides_nothing_even_with_the_same_stamp(before):
    """A stepped simulation clock: the reading the adapter held when it applied the engage carries
    the engage's own stamp. It predates the engage by ARRIVAL, so the site is PENDING (never a
    decided NOT_HELD that a consumer, judging site_stamp >= the engage's stamp, would take for a
    verdict about this engage) until a reading received after it decides (ADR-0022 D4)."""
    t = 100 * SEC
    m = suction()
    m.observe_actuator('vacuum', SEC, binary=Binary.RELEASED)
    m.observe_site('main', before, t)
    m.apply('vacuum', Command.ENGAGE, t)
    assert view(m) == (Grasp.PENDING, SENSOR, t)
    m.observe_site('main', Reading.NOT_HELD, t)      # received after the engage, same stamp
    assert view(m) == (Grasp.NOT_HELD, SENSOR, t)
    m.observe_site('main', Reading.HELD, t + 50 * MS)
    assert view(m) == (Grasp.HELD, SENSOR, t + 50 * MS)


def test_a_reading_received_after_a_release_still_counts_for_it():
    """The arrival rule is about the ENGAGE only: after an applied release, the newest reading is
    the site's grasp, whatever its stamp (a NOT_HELD confirms the release)."""
    t = 100 * SEC
    m = suction()
    engaged_and_held(m, t0=1)
    m.apply('vacuum', Command.RELEASE, t)
    m.observe_site('main', Reading.NOT_HELD, t)
    assert view(m) == (Grasp.NOT_HELD, SENSOR, t)


def test_undecided_without_an_engage_and_cannot_tell_are_unknown():
    m = suction()
    m.observe_actuator('vacuum', SEC, binary=Binary.RELEASED)
    m.observe_site('main', Reading.UNDECIDED, 2 * SEC)
    assert view(m) == (Grasp.UNKNOWN, Evidence.NONE, 2 * SEC)
    engaged_and_held(m, t0=3)
    m.observe_site('main', Reading.NOT_HELD, 5 * SEC)
    m.observe_site('main', Reading.CANNOT_TELL, 6 * SEC)
    assert view(m) == (Grasp.UNKNOWN, Evidence.NONE, 6 * SEC)
    m.observe_site('main', Reading.NOT_HELD, 7 * SEC)       # the latch survived
    assert view(m)[0] == Grasp.LOST


def test_a_site_with_no_deciding_source_is_unavailable_whatever_happens():
    m = suction(source=None)
    m.observe_actuator('vacuum', SEC, binary=Binary.RELEASED)
    m.apply('vacuum', Command.ENGAGE, 2 * SEC)
    assert view(m) == (Grasp.UNAVAILABLE, Evidence.NONE, 0)
    with pytest.raises(ValueError):
        m.observe_site('main', Reading.HELD, 3 * SEC)


@pytest.mark.parametrize('source', [Evidence.COMMAND_ECHO, Evidence.SIM_ATTACHMENT, Evidence.NONE])
def test_echo_and_weld_sources_are_refused(source):
    with pytest.raises(ValueError, match='never'):
        suction(source=source)


# ------------------------------------------------------------------------------------------------
# commands, switches, stamps
# ------------------------------------------------------------------------------------------------

def test_last_command_records_applied_commands_only():
    m = composite()
    m.observe_actuator('gripper_b', SEC, binary=Binary.RELEASED)
    m.reject('gripper_b', Command.ENGAGE, 'injected reject (mock)')
    s = m.sample()
    assert (s.actuator('gripper_b').last_command, s.actuator('gripper_b').last_command_stamp) == (Command.NONE, 0)
    assert s.actuator('gripper_b').binary == Binary.RELEASED
    assert s.detail == 'gripper_b: ENGAGE rejected: injected reject (mock)'
    m.apply('gripper_b', Command.ENGAGE, 2 * SEC)
    a = m.sample().actuator('gripper_b')
    assert (a.last_command, a.last_command_stamp, a.binary, a.binary_confirmed, a.stamp) == (
        Command.ENGAGE, 2 * SEC, Binary.ENGAGED, False, 2 * SEC)
    assert m.sample().detail == ''
    with pytest.raises(ValueError):
        m.apply('gripper_b', Command.TARGET, 3 * SEC)        # a set_bool takes engage / release
    with pytest.raises(ValueError):
        m.apply('gripper_b', Command.NONE, 3 * SEC)
    with pytest.raises(ValueError):
        m.apply('gripper_b', Command.ENGAGE, 0)              # a stamp is never zero


def test_an_end_switch_confirms_the_applied_state():
    m = composite()
    m.observe_actuator('binary_lock', SEC, binary=Binary.RELEASED)
    m.observe_switch('binary_lock', Binary.RELEASED, SEC)
    a = m.sample().actuator('binary_lock')
    assert (a.binary, a.binary_confirmed, a.status) == (Binary.RELEASED, True, AS.IDLE)
    m.apply('binary_lock', Command.ENGAGE, 2 * SEC)
    a = m.sample().actuator('binary_lock')
    assert (a.binary, a.binary_confirmed, a.status) == (Binary.ENGAGED, False, AS.MOVING)
    m.observe_switch('binary_lock', Binary.RELEASED, 3 * SEC)  # the switch has not moved yet
    a = m.sample().actuator('binary_lock')
    assert (a.binary_confirmed, a.status, a.stamp) == (False, AS.MOVING, 3 * SEC)
    m.observe_switch('binary_lock', Binary.ENGAGED, 4 * SEC)
    a = m.sample().actuator('binary_lock')
    assert (a.binary_confirmed, a.status, a.stamp) == (True, AS.IDLE, 4 * SEC)
    with pytest.raises(ValueError):
        m.observe_switch('gripper_b', Binary.ENGAGED, 5 * SEC)  # no actuation_feedback: no switch


def test_goal_events_stamp_an_action_actuator():
    m = composite()
    m.observe_actuator('linear_axis', SEC)
    m.apply('linear_axis', Command.TARGET, 2 * SEC)
    a = m.sample().actuator('linear_axis')
    assert (a.status, a.stamp, a.last_command, a.last_command_stamp, a.binary) == (
        AS.MOVING, 2 * SEC, Command.TARGET, 2 * SEC, Binary.UNKNOWN)
    m.goal_done('linear_axis', 3 * SEC)
    a = m.sample().actuator('linear_axis')
    assert (a.status, a.stamp, a.last_command_stamp) == (AS.IDLE, 3 * SEC, 2 * SEC)


def test_the_header_stamp_is_the_newest_observation():
    m = composite()
    m.observe_actuator('gripper_a', 5 * SEC)
    m.observe_site('site_b', Reading.NOT_HELD, 7 * SEC)
    m.observe_actuator('linear_axis', 6 * SEC)
    assert m.sample().stamp == 7 * SEC
    m.apply('gripper_b', Command.ENGAGE, 9 * SEC)
    assert m.sample().stamp == 9 * SEC


def test_the_message_carries_every_field_in_contract_order():
    m = composite()
    m.observe_actuator('gripper_b', SEC, binary=Binary.RELEASED)
    m.apply('gripper_b', Command.ENGAGE, 2 * SEC)
    m.observe_site('site_b', Reading.HELD, 3 * SEC)
    msg = m.sample().to_msg()
    assert msg.end_effector == '/r1/eef/composite_tool' and msg.adapter == 'trainit_mock'
    assert msg.header.frame_id == '' and h.ns_of(msg.header.stamp) == 3 * SEC
    assert list(msg.sites) == ['site_a', 'site_b'] and list(msg.site_grip) == ['gripper_a', 'gripper_b']
    assert len(msg.site_grasp) == len(msg.site_evidence) == len(msg.site_stamp) == 2
    assert [a.actuator for a in msg.actuators] == list(DEVICES['composite_tool'].actuator_ids)
    assert [a.command_interface for a in msg.actuators] == [3, 3, 3, 2, 1, 1]
    assert [list(a.joints) for a in msg.actuators] == [
        list(DEVICES['composite_tool'].actuators[a].joints) for a in DEVICES['composite_tool'].actuator_ids]
    b = h.actuator(msg, 'gripper_b')
    assert (b.binary, b.last_command, h.ns_of(b.last_command_stamp)) == (Binary.ENGAGED, Command.ENGAGE, 2 * SEC)
    assert h.site(msg, 'site_b') == (Grasp.HELD, SENSOR, 3 * SEC)


# ------------------------------------------------------------------------------------------------
# the publish rule
# ------------------------------------------------------------------------------------------------

def test_a_sample_on_every_change_and_the_heartbeat_only_when_nothing_changed():
    m = suction()
    gate = core.PublishGate(100 * MS)
    m.observe_actuator('vacuum', SEC, binary=Binary.RELEASED)
    first = gate.offer(m.sample(), 0)
    assert first is not None
    assert gate.offer(m.sample(), 1 * MS) is None                       # nothing changed
    m.observe_site('main', Reading.NOT_HELD, 2 * SEC)
    second = gate.offer(m.sample(), 2 * MS)
    assert second is not None and second.site_stamp == (2 * SEC,)
    m.observe_site('main', Reading.NOT_HELD, 3 * SEC)                   # a stamp advance alone
    third = gate.offer(m.sample(), 3 * MS)
    assert third is not None and third.site_grasp == second.site_grasp and third.stamp == 3 * SEC
    assert gate.heartbeat(m.sample(), 20 * MS) is None                  # the change is too recent
    beat = gate.heartbeat(m.sample(), 100 * MS)
    assert beat is third                                                # the SAME sample, same stamps
    again = gate.heartbeat(m.sample(), 200 * MS)
    assert again is third and again.to_msg() == third.to_msg()
    m.apply('vacuum', Command.ENGAGE, 4 * SEC)
    changed = gate.heartbeat(m.sample(), 210 * MS)                      # a pending change goes first
    assert changed is not None and changed is not third and changed.stamp == 4 * SEC
    with pytest.raises(ValueError):
        core.PublishGate(0)


def test_a_heartbeat_never_advances_a_stamp():
    m = composite()
    gate = core.PublishGate(100 * MS)
    m.observe_actuator('gripper_a', SEC)
    m.observe_site('site_a', Reading.NOT_HELD, 2 * SEC)
    published = [gate.offer(m.sample(), 0)]
    for k in range(1, 30):
        beat = gate.heartbeat(m.sample(), k * 100 * MS)
        if beat is not None:
            published.append(beat)
    assert len(published) > 20
    assert {p.stamp for p in published} == {2 * SEC}
    assert {p.site_stamp for p in published} == {(2 * SEC, 0)}
    assert {tuple(a.stamp for a in p.actuators) for p in published} == {(0, 0, 0, SEC, 0, 0)}


# ------------------------------------------------------------------------------------------------
# every invariant, after every operation of a seeded random walk
# ------------------------------------------------------------------------------------------------

def _check(model, sources, seen):
    s = model.sample()
    dev = model.device
    seen.update(('grasp', g) for g in s.site_grasp)
    seen.add(('status', s.status))
    assert len(s.sites) == len(s.site_grip) == len(s.site_grasp) == len(s.site_evidence) == len(s.site_stamp)
    for i, site_id in enumerate(s.sites):
        grasp, evidence, stamp = s.site_grasp[i], s.site_evidence[i], s.site_stamp[i]
        assert (evidence == Evidence.NONE) == (grasp in (Grasp.UNAVAILABLE, Grasp.UNKNOWN))
        assert evidence not in (Evidence.COMMAND_ECHO, Evidence.SIM_ATTACHMENT)
        if sources[site_id] is None:
            assert (grasp, evidence, stamp) == (Grasp.UNAVAILABLE, Evidence.NONE, 0)
        elif evidence != Evidence.NONE:
            assert evidence == sources[site_id]
        grip = s.actuator(dev.sites[site_id].grip)
        if grasp == Grasp.PENDING:
            assert grip.last_command == Command.ENGAGE
        if grasp == Grasp.LOST:
            assert grip.last_command in (Command.ENGAGE, Command.TARGET)
    stamps = list(s.site_stamp) + [a.stamp for a in s.actuators]
    assert s.stamp == max(stamps)
    for a in s.actuators:
        assert (a.last_command == Command.NONE) == (a.last_command_stamp == 0)
        spec = dev.actuators[a.actuator]
        if spec.command_interface != 'set_bool':
            assert a.binary == Binary.UNKNOWN
        if not (spec.command_interface == 'set_bool' and spec.actuation_feedback):
            assert not a.binary_confirmed
    assert s.status == core.aggregate_status(a.status for a in s.actuators)


@pytest.mark.parametrize('sources', [
    {'site_a': Evidence.SENSOR, 'site_b': None},
    {'site_a': Evidence.SIM_GROUND_TRUTH, 'site_b': Evidence.ACTUATION_INFERENCE},
])
def test_every_invariant_holds_on_a_seeded_random_walk(sources):
    rng = random.Random(20261010)
    m = composite(sources, {'site_a': 0, 'site_b': 300 * MS})
    dev = m.device
    seen = set()
    t = SEC
    for _ in range(3000):
        t += rng.randint(1, 200) * MS
        op = rng.choice(['apply', 'apply', 'reject', 'observe', 'observe', 'observe', 'tick',
                         'condition', 'switch', 'switch', 'goal_done', 'goal_done', 'goal_done',
                         'observe_actuator'])
        a = rng.choice(dev.actuator_ids)
        spec = dev.actuators[a]
        s = rng.choice(dev.site_ids)
        if op == 'apply':
            cmds = ([Command.ENGAGE, Command.RELEASE] if spec.command_interface == 'set_bool'
                    else [Command.ENGAGE, Command.RELEASE, Command.TARGET])
            m.apply(a, rng.choice(cmds), t)
        elif op == 'reject':
            m.reject(a, rng.choice([Command.ENGAGE, Command.RELEASE]), 'refused')
        elif op == 'observe' and sources[s] is not None:
            m.observe_site(s, rng.choice(list(Reading)), t)
        elif op == 'tick':
            m.tick(t)
        elif op == 'condition':
            m.set_condition(a, rng.choice([None] * 9 + [AS.FAULT, AS.UNAVAILABLE, AS.UNKNOWN]), 'cond')
        elif op == 'switch' and spec.command_interface == 'set_bool' and spec.actuation_feedback:
            m.observe_switch(a, rng.choice([Binary.RELEASED, Binary.ENGAGED]), t)
        elif op == 'goal_done' and spec.command_interface != 'set_bool':
            m.goal_done(a, t)
        elif op == 'observe_actuator':
            m.observe_actuator(a, t)
        _check(m, sources, seen)
    # the walk is not trivial: it reached every grasp value (UNAVAILABLE only where a site has no
    # source) and every tool status
    grasps = {g for g in Grasp if g != Grasp.UNAVAILABLE or None in sources.values()}
    assert {('grasp', int(g)) for g in grasps} <= seen, sorted(seen)
    assert {('status', int(st)) for st in TS} <= seen, sorted(seen)
