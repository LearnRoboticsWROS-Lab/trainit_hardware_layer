"""H2-H6 -- the mock adapter on a real ROS graph (domain 86; ADR-0022 D2, D3, D7, D8).

H2  two robots with IDENTICAL eef, actuator and site ids in /r1 and /r2, plus the root variant and
    a suction device in /r3, at once: every EEF endpoint exists exactly once per namespace and none
    outside one; each state topic has exactly one publisher; each late joiner receives exactly one
    sample, its own; a command in /r1 changes only /r1.
H3  correlation: contract order, interfaces, joints and site_grip; an axis MOVING gives BUSY with
    the sites unchanged; an injected FAULT gives FAULT, ``detail`` names it, the sites unchanged;
    a site going LOST leaves every actuator entry unchanged.
H4  LOST attribution: a release through a foreign SetBool client, or through the policy card's
    Trigger pair called by a plain Trigger client, reads NOT_HELD (the run-start OFF too); the same
    edge with no command reads LOST, and the next applied release clears it.
H5  the controller emulation, as M9 measured the installed controllers: the JTC (exact joint set,
    effort rejected, a motion over its time_from_start, carried tolerances and their fallbacks, -1 /
    -4 / -5, CANCELED with an EMPTY result on a cancel and on a preemption, where the preempted goal
    ends only when the preempting goal ends, the "blocked axis without tolerances ends SUCCEEDED"
    hazard); the GripperActionController (one joint, the server-side goal tolerance, the stall
    flags, CANCELED with an empty result, at once on a preemption); SetBool (reject, no answer,
    applied but unanswered, unavailable, the end switch); the GripperCommand suction handle (honest
    codes).
H6  the pseudo-joint echo follows the applied command and never touches a site.
Plus: the scene signal, an injected condition in the first (latched) sample, and the parameter
refusals of an adapter.
"""
import itertools
import time

import pytest

import eef_harness as h
from eef_harness import actuator, site

rclpy = pytest.importorskip('rclpy', reason='the adapter tests need a sourced ROS 2 Humble')

from action_msgs.msg import GoalStatus  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory as FJT  # noqa: E402
from control_msgs.action import GripperCommand as GC  # noqa: E402
from control_msgs.msg import JointTolerance  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

from trainit_hardware_layer.eef.contract import Refused  # noqa: E402
from trainit_hardware_layer.eef.core import ActuatorStatus as AS  # noqa: E402
from trainit_hardware_layer.eef.core import Binary, Command, Evidence, Grasp  # noqa: E402
from trainit_hardware_layer.eef.core import ToolStatus as TS  # noqa: E402

FAST = {'mock.sites.site_a.decide_after_ms': 50, 'mock.sites.site_b.decide_after_ms': 50}
L = 'robot_a_linear_axis_joint'
P, R = 'robot_a_wrist_pitch_joint', 'robot_a_wrist_roll_joint'


def _duration(seconds: float) -> Duration:
    return Duration(sec=int(seconds), nanosec=int(round((seconds % 1) * 1e9)))


def fjt_goal(joints, positions, seconds, goal_tol=None, path_tol=None, goal_time=0.2):
    goal = FJT.Goal()
    goal.trajectory.joint_names = list(joints)
    goal.trajectory.points = [JointTrajectoryPoint(positions=[float(x) for x in positions],
                                                   time_from_start=_duration(seconds))]
    if goal_tol:
        goal.goal_tolerance = [JointTolerance(name=j, position=float(v)) for j, v in zip(joints, goal_tol)]
    if path_tol:
        goal.path_tolerance = [JointTolerance(name=j, position=float(v)) for j, v in zip(joints, path_tol)]
    goal.goal_time_tolerance = _duration(goal_time)
    return goal


def gc_goal(position, max_effort=0.0):
    goal = GC.Goal()
    goal.command.position = float(position)
    goal.command.max_effort = float(max_effort)
    return goal


class JointWatch:
    """Every JointState on one topic (name -> list of positions)."""

    def __init__(self, node, topic):
        self.positions = {}
        node.create_subscription(JointState, topic, self._on, 10)

    def _on(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.positions.setdefault(name, []).append(position)

    def latest(self, name):
        values = self.positions.get(name)
        return values[-1] if values else None


def _composite_names(ns):
    base = f'{ns}/eef/composite_tool'
    actions = [f'{ns}/linear_axis_controller/follow_joint_trajectory',
               f'{ns}/rotary_axis_controller/follow_joint_trajectory',
               f'{ns}/wrist_mechanism_controller/follow_joint_trajectory',
               f'{ns}/gripper_a_controller/gripper_cmd']
    action_topics, action_services = h.action_names(actions)
    topics = {f'{base}/state', f'{base}/joint_states'} | action_topics
    services = {f'{base}/actuators/gripper_b/set_engaged',
                f'{base}/actuators/binary_lock/set_engaged'} | action_services
    return topics, services


# ================================================================================================
# H2
# ================================================================================================

def test_h2_two_robots_and_the_root_with_identical_ids_never_collide(harness):
    # Quiet adapters: nothing changes, and the first heartbeat comes 100 s after each one's start,
    # far beyond this test however slow the machine is, so every sample counted below is latched.
    quiet = {'publish_rate_hz': 0.01, 'mock.sites.site_a.outcome': 'unavailable',
             'mock.sites.site_b.outcome': 'unavailable'}
    adapters = {
        '/r1': harness.mock('/r1', 'composite_tool', **quiet),
        '/r2': harness.mock('/r2', 'composite_tool',
                            h.device_block('composite_tool', joint_prefix=('robot_a_', 'robot_b_')), **quiet),
        '': harness.mock('/', 'composite_tool',
                         h.device_block('composite_tool', joint_prefix=('robot_a_', '')), **quiet),
        '/r3': harness.mock('/r3', 'suction_simple', publish_rate_hz=0.01,
                            **{'mock.sites.main.outcome': 'unavailable'}),
    }
    probe = harness.helper('h2_late_joiners')
    states = {ns: f'{ns}/eef/{"suction_simple" if ns == "/r3" else "composite_tool"}/state' for ns in adapters}
    # Late joiners: every adapter published its first sample in its constructor, before them.
    late = {ns: h.StateWatch(probe, topic) for ns, topic in states.items()}
    for ns, watch in late.items():
        assert h.wait_until(lambda w=watch: w.count() >= 1, 3.0), ns
    time.sleep(1.0)   # a quiet adapter publishes nothing more (no change, no heartbeat yet)
    for ns, watch in late.items():
        assert watch.count() == 1, (ns, watch.count())
        assert watch.latest().end_effector == states[ns][:-len('/state')], ns
    joints = {ns: [list(a.joints) for a in late[ns].latest().actuators] for ns in ('/r1', '/r2', '')}
    assert joints['/r1'][0] == ['robot_a_linear_axis_joint']
    assert joints['/r2'][0] == ['robot_b_linear_axis_joint']
    assert joints[''][0] == ['linear_axis_joint']
    assert [a.actuator for a in late['/r1'].latest().actuators] == [
        a.actuator for a in late['/r2'].latest().actuators]

    # The graph: exactly the EEF endpoints, once per namespace, and nothing else but ROS's own.
    expected_topics, expected_services = set(), set()
    for ns in ('/r1', '/r2', ''):
        t, s = _composite_names(ns)
        expected_topics |= t
        expected_services |= s
    expected_topics |= {'/r3/eef/suction_simple/state', '/r3/eef/suction_simple/engaged'}
    expected_services |= {'/r3/eef/suction_simple/actuators/vacuum/set_engaged'}
    node_prefixes = tuple(a.get_fully_qualified_name() + '/' for a in adapters.values())

    def own(names):
        return {n for n in names if n not in ('/rosout', '/parameter_events')
                and not n.startswith(node_prefixes)}
    assert h.wait_until(lambda: own(h.graph_names(probe)[0]) >= expected_topics
                        and own(h.graph_names(probe)[1]) >= expected_services, 6.0)
    topics, services = h.graph_names(probe)
    assert own(topics) == expected_topics
    assert own(services) == expected_services
    for ns, topic in states.items():
        info = probe.get_publishers_info_by_topic(topic)
        assert len(info) == 1, topic
        assert info[0].node_namespace == (ns or '/'), (topic, info[0].node_namespace)
    assert not any('attached' in t for t in topics)

    # A command in /r1 changes only /r1; an axis goal in /r1 moves only /r1's entry.
    client = probe.create_client(SetBool, '/r1/eef/composite_tool/actuators/gripper_b/set_engaged')
    assert h.call(client, SetBool.Request(data=True)).success
    late['/r1'].wait(lambda m: actuator(m, 'gripper_b').last_command == Command.ENGAGE)
    axis = ActionClient(probe, FJT, '/r1/linear_axis_controller/follow_joint_trajectory')
    accepted, handle = h.send_goal(axis, fjt_goal([L], [0.08], 0.2, goal_tol=[0.002]))
    assert accepted
    assert h.result_of(handle)[0] == GoalStatus.STATUS_SUCCEEDED
    late['/r1'].wait(lambda m: actuator(m, 'linear_axis').last_command == Command.TARGET)
    for ns in ('/r2', ''):
        m = late[ns].latest()
        assert actuator(m, 'gripper_b').last_command == Command.NONE, ns
        assert actuator(m, 'linear_axis').last_command == Command.NONE, ns
    assert actuator(late['/r3'].latest(), 'vacuum').last_command == Command.NONE


# ================================================================================================
# H3
# ================================================================================================

def test_h3_the_state_correlates_actuators_and_sites(harness):
    harness.mock('/r1', 'composite_tool', **FAST, **{
        'mock.actuators.binary_lock.fail': 'fault', 'mock.actuators.binary_lock.fail_after_s': 4.0,
        'mock.actuators.gripper_a.travel_s': 0.1})
    probe = harness.helper('h3')
    w = h.StateWatch(probe, '/r1/eef/composite_tool/state')
    # before its first sensor sample a site cannot tell (UNKNOWN); then the sensors decide
    first = w.wait(lambda m: m.status == TS.READY and all(
        site(m, s)[0] != Grasp.UNKNOWN for s in ('site_a', 'site_b')))
    assert first.end_effector == '/r1/eef/composite_tool' and first.adapter == 'trainit_mock'
    assert list(first.sites) == ['site_a', 'site_b'] and list(first.site_grip) == ['gripper_a', 'gripper_b']
    assert [a.actuator for a in first.actuators] == ['linear_axis', 'rotary_axis', 'wrist_mechanism',
                                                     'gripper_a', 'gripper_b', 'binary_lock']
    assert [a.command_interface for a in first.actuators] == [3, 3, 3, 2, 1, 1]
    assert [list(a.joints) for a in first.actuators] == [
        [L], ['robot_a_rotary_axis_joint'], [P, R], ['robot_a_jaw_a_joint'], ['robot_a_jaw_b_joint'], []]
    # healthy actuators with the sites NOT_HELD: READY (sites never feed the status)
    assert [site(first, s)[:2] for s in ('site_a', 'site_b')] == [(Grasp.NOT_HELD, Evidence.SENSOR)] * 2

    jaw = ActionClient(probe, GC, '/r1/gripper_a_controller/gripper_cmd')
    accepted, handle = h.send_goal(jaw, gc_goal(0.0, 20.0))      # site_a's engage target "closed"
    assert accepted
    status, result = h.result_of(handle)
    assert status == GoalStatus.STATUS_SUCCEEDED and result.reached_goal and not result.stalled
    held = w.wait(lambda m: site(m, 'site_a')[0] == Grasp.HELD)
    assert actuator(held, 'gripper_a').last_command == Command.ENGAGE

    axis = ActionClient(probe, FJT, '/r1/linear_axis_controller/follow_joint_trajectory')
    accepted, handle = h.send_goal(axis, fjt_goal([L], [0.08], 0.6, goal_tol=[0.002]))
    assert accepted
    busy = w.wait(lambda m: m.status == TS.BUSY)
    assert actuator(busy, 'linear_axis').status == AS.MOVING
    assert site(busy, 'site_a')[0] == Grasp.HELD and site(busy, 'site_b')[0] == Grasp.NOT_HELD
    status, result = h.result_of(handle)
    assert status == GoalStatus.STATUS_SUCCEEDED and result.error_code == 0
    w.wait(lambda m: m.status == TS.READY)

    fault = w.wait(lambda m: m.status == TS.FAULT, timeout=8.0)
    assert actuator(fault, 'binary_lock').status == AS.FAULT
    assert 'binary_lock: injected FAULT (mock)' in fault.detail
    assert site(fault, 'site_a')[0] == Grasp.HELD and site(fault, 'site_b')[0] == Grasp.NOT_HELD


def test_h3_a_site_going_lost_leaves_every_actuator_unchanged(harness):
    harness.mock('/r2', 'composite_tool', h.device_block('composite_tool', joint_prefix=('robot_a_', 'robot_b_')),
                 **FAST, **{'mock.sites.site_a.outcome': 'lost_after', 'mock.sites.site_a.after_s': 0.5,
                            'mock.actuators.gripper_a.travel_s': 0.1})
    probe = harness.helper('h3_lost')
    w = h.StateWatch(probe, '/r2/eef/composite_tool/state', depth=500)
    w.wait(lambda m: m.status == TS.READY)
    jaw = ActionClient(probe, GC, '/r2/gripper_a_controller/gripper_cmd')
    accepted, handle = h.send_goal(jaw, gc_goal(0.0))
    assert accepted and h.result_of(handle)[0] == GoalStatus.STATUS_SUCCEEDED
    w.wait(lambda m: site(m, 'site_a')[0] == Grasp.LOST, timeout=5.0)
    samples = w.all()
    i = next(k for k, m in enumerate(samples) if site(m, 'site_a')[0] == Grasp.LOST)
    before, after = samples[i - 1], samples[i]
    assert site(before, 'site_a')[0] == Grasp.HELD

    def entries(m):
        return [(a.actuator, a.status, a.binary, a.binary_confirmed, h.ns_of(a.stamp), a.last_command,
                 h.ns_of(a.last_command_stamp)) for a in m.actuators]
    assert entries(before) == entries(after)
    assert after.status == before.status == TS.READY
    assert site(after, 'site_b')[:2] == site(before, 'site_b')[:2] == (Grasp.NOT_HELD, Evidence.SENSOR)


# ================================================================================================
# H4
# ================================================================================================

def _never_lost(watch, s='main'):
    return all(site(m, s)[0] != Grasp.LOST for m in watch.all())


def test_h4_a_release_from_any_commander_is_not_a_loss(harness):
    harness.mock('/', 'suction_simple', **{
        'compat.trigger_on': '/suction/on', 'compat.trigger_off': '/suction/off',
        'mock.sites.main.decide_after_ms': 50})
    probe = harness.helper('h4_card')
    foreign = harness.helper('foreign_commander', namespace='/elsewhere')
    w = h.StateWatch(probe, '/eef/suction_simple/state', depth=500)
    on = probe.create_client(Trigger, '/suction/on')
    off = probe.create_client(Trigger, '/suction/off')
    # the policy runtime's run-start OFF, with nothing held: a release, never a loss
    response = h.call(off, Trigger.Request())
    assert response.success and response.message == 'released'
    w.wait(lambda m: actuator(m, 'vacuum').last_command == Command.RELEASE and site(m, 'main')[0] == Grasp.NOT_HELD)
    # the card's pair, through a plain Trigger client
    assert h.call(on, Trigger.Request()).success
    held = w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD)
    assert actuator(held, 'vacuum').last_command == Command.ENGAGE
    assert h.call(off, Trigger.Request()).success
    w.wait(lambda m: site(m, 'main')[0] == Grasp.NOT_HELD and actuator(m, 'vacuum').last_command == Command.RELEASE)
    # a foreign SetBool client, from another namespace
    client = foreign.create_client(SetBool, '/eef/suction_simple/actuators/vacuum/set_engaged')
    assert h.call(client, SetBool.Request(data=True)).success
    w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD)
    assert h.call(client, SetBool.Request(data=False)).success
    w.wait(lambda m: site(m, 'main')[0] == Grasp.NOT_HELD)
    time.sleep(0.2)
    assert _never_lost(w)


def test_h4_the_same_edge_with_no_command_is_lost_until_the_next_release(harness):
    harness.mock('/r5', 'suction_simple', **{'mock.sites.main.outcome': 'lost_after',
                                             'mock.sites.main.after_s': 0.3,
                                             'mock.sites.main.decide_after_ms': 50})
    probe = harness.helper('h4_lost')
    w = h.StateWatch(probe, '/r5/eef/suction_simple/state', depth=500)
    client = probe.create_client(SetBool, '/r5/eef/suction_simple/actuators/vacuum/set_engaged')
    assert h.call(client, SetBool.Request(data=True)).success
    w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD)
    lost = w.wait(lambda m: site(m, 'main')[0] == Grasp.LOST, timeout=3.0)
    assert actuator(lost, 'vacuum').last_command == Command.ENGAGE      # no release was applied
    assert site(lost, 'main')[1] == Evidence.SENSOR
    assert h.call(client, SetBool.Request(data=False)).success
    w.wait(lambda m: site(m, 'main')[0] == Grasp.NOT_HELD and actuator(m, 'vacuum').last_command == Command.RELEASE)


# ================================================================================================
# H5 -- the joint_trajectory_controller emulation
# ================================================================================================

def test_h5_the_trajectory_controller_emulation(harness):
    harness.mock('/r1', 'composite_tool', **{'mock.actuators.wrist_mechanism.initial_target': 'stow'})
    probe = harness.helper('h5_fjt')
    w = h.StateWatch(probe, '/r1/eef/composite_tool/state', depth=500)
    joints = JointWatch(probe, '/r1/eef/composite_tool/joint_states')
    wrist = ActionClient(probe, FJT, '/r1/wrist_mechanism_controller/follow_joint_trajectory')
    linear = ActionClient(probe, FJT, '/r1/linear_axis_controller/follow_joint_trajectory')
    w.wait(lambda m: m.status == TS.READY)

    # the exact joint set: a one-joint goal on the 2-joint mechanism, and a foreign joint
    assert h.send_goal(wrist, fjt_goal([P], [0.3], 0.3))[0] is False
    w.wait(lambda m: 'wrist_mechanism: TARGET rejected: joints' in m.detail)
    assert h.send_goal(wrist, fjt_goal([P, 'robot_b_wrist_roll_joint'], [0.3, -0.3], 0.3))[0] is False
    # effort is rejected
    goal = fjt_goal([P, R], [0.3, -0.3], 0.3)
    goal.trajectory.points[0].effort = [1.0, 1.0]
    assert h.send_goal(wrist, goal)[0] is False
    assert actuator(w.latest(), 'wrist_mechanism').last_command == Command.NONE

    # the exact set in another order, with carried tolerances: SUCCEEDED, over its time_from_start
    start = time.monotonic()
    accepted, handle = h.send_goal(wrist, fjt_goal([R, P], [-0.3, 0.3], 0.5, goal_tol=[0.01, 0.01]))
    assert accepted
    status, result = h.result_of(handle)
    elapsed = time.monotonic() - start
    assert status == GoalStatus.STATUS_SUCCEEDED and result.error_code == 0, (status, result)
    assert elapsed >= 0.45, elapsed
    assert any(0.02 < x < 0.28 for x in joints.positions[P]), 'no intermediate position was published'
    assert h.wait_until(lambda: abs(joints.latest(P) - 0.3) < 1e-9 and abs(joints.latest(R) + 0.3) < 1e-9, 2.0)
    done = w.wait(lambda m: actuator(m, 'wrist_mechanism').status == AS.IDLE
                  and actuator(m, 'wrist_mechanism').last_command == Command.TARGET)
    entry = actuator(done, 'wrist_mechanism')
    assert h.ns_of(entry.stamp) > h.ns_of(entry.last_command_stamp) > 0

    # (a preemption: test_h5_a_preempted_trajectory_goal_ends_only_when_the_preempting_goal_ends)
    # a cancel: CANCELED with an empty result
    accepted, handle = h.send_goal(linear, fjt_goal([L], [0.08], 1.0, goal_tol=[0.002]))
    assert accepted
    time.sleep(0.2)
    future = handle.cancel_goal_async()
    assert h.wait_until(future.done, 5.0)
    status, result = h.result_of(handle)
    assert status == GoalStatus.STATUS_CANCELED and result.error_code == 0 and result.error_string == ''
    w.wait(lambda m: actuator(m, 'linear_axis').status == AS.IDLE)


@pytest.mark.parametrize('fail,status,code', [
    ('abort_m1', GoalStatus.STATUS_ABORTED, -1),
    ('abort_m4', GoalStatus.STATUS_ABORTED, -4),
    ('abort_m5', GoalStatus.STATUS_ABORTED, -5),
    ('canceled', GoalStatus.STATUS_CANCELED, 0),
])
def test_h5_trajectory_failures_are_injected_by_parameters(harness, fail, status, code):
    harness.mock('/f1', 'composite_tool', **{'mock.actuators.linear_axis.fail': fail})
    probe = harness.helper('h5_fjt_fail')
    w = h.StateWatch(probe, '/f1/eef/composite_tool/state')
    linear = ActionClient(probe, FJT, '/f1/linear_axis_controller/follow_joint_trajectory')
    accepted, handle = h.send_goal(linear, fjt_goal([L], [0.08], 0.4, goal_tol=[0.002]))
    assert accepted
    got_status, result = h.result_of(handle)
    assert (got_status, result.error_code) == (status, code), result
    if fail == 'canceled':
        assert result.error_string == ''                                # an EMPTY result, as the JTC's
    entry = actuator(w.wait(lambda m: actuator(m, 'linear_axis').status == AS.IDLE), 'linear_axis')
    assert entry.last_command == Command.TARGET                         # it was applied (accepted)


def test_h5_a_blocked_axis_is_judged_by_the_carried_tolerances_only(harness):
    harness.mock('/f2', 'composite_tool', **{'mock.actuators.linear_axis.fail': 'blocked'})
    probe = harness.helper('h5_blocked')
    linear = ActionClient(probe, FJT, '/f2/linear_axis_controller/follow_joint_trajectory')
    # a goal tolerance: -5 after the goal time tolerance
    accepted, handle = h.send_goal(linear, fjt_goal([L], [0.08], 0.4, goal_tol=[0.002]))
    status, result = h.result_of(handle)
    assert (status, result.error_code) == (GoalStatus.STATUS_ABORTED, -5)
    # a path tolerance: -4 during the motion
    accepted, handle = h.send_goal(linear, fjt_goal([L], [0.0], 0.4, path_tol=[0.005]))
    status, result = h.result_of(handle)
    assert (status, result.error_code) == (GoalStatus.STATUS_ABORTED, -4)
    # no tolerance at all: SUCCEEDED although the axis is blocked (why TSA refuses it)
    accepted, handle = h.send_goal(linear, fjt_goal([L], [0.08], 0.4))
    status, result = h.result_of(handle)
    assert (status, result.error_code) == (GoalStatus.STATUS_SUCCEEDED, 0)


def _tolerance(name, position=0.0, velocity=0.0, acceleration=0.0):
    return JointTolerance(name=name, position=float(position), velocity=float(velocity),
                          acceleration=float(acceleration))


def test_h5_carried_tolerances_fall_back_as_the_jtc_does(harness):
    """M9 X3, X4 and P3 on a blocked axis: the JTC's fallbacks, never a per-entry guess."""
    harness.mock('/f5', 'composite_tool', **{'mock.actuators.linear_axis.fail': 'blocked'})
    probe = harness.helper('h5_tolerance_fallbacks')
    linear = ActionClient(probe, FJT, '/f5/linear_axis_controller/follow_joint_trajectory')
    # the axis stops halfway: alternating ends keep it at least 0.02 short of every target
    targets = itertools.cycle([0.08, 0.0])

    def outcome(goal_tol=(), path_tol=(), goal_time=0.2):
        goal = fjt_goal([L], [next(targets)], 0.4, goal_time=goal_time)
        goal.goal_tolerance, goal.path_tolerance = list(goal_tol), list(path_tol)
        accepted, handle = h.send_goal(linear, goal)
        assert accepted
        status, result = h.result_of(handle)
        return status, result.error_code

    succeeded = (GoalStatus.STATUS_SUCCEEDED, 0)
    # X3: one entry naming a joint the controller does not have drops EVERY carried tolerance, the
    # valid one on the axis included: the blocked axis SUCCEEDS at T (the defaults check nothing)
    assert outcome(goal_tol=[_tolerance(L, 0.002), _tolerance('no_such_joint', 0.01)]) == succeeded
    # X4: so does one illegal value (negative, not the erase value -1), in any field of any entry,
    # and an illegal goal time tolerance
    assert outcome(goal_tol=[_tolerance(L, 0.002)], path_tol=[_tolerance(L, -2.0)]) == succeeded
    assert outcome(goal_tol=[_tolerance(L, 0.002, velocity=-0.5)]) == succeeded
    assert outcome(goal_tol=[_tolerance(L, 0.002)], goal_time=-2.0) == succeeded
    # -1 erases one tolerance (not checked) and is legal: the other carried tolerances still apply
    assert outcome(goal_tol=[_tolerance(L, -1.0)]) == succeeded
    assert outcome(goal_tol=[_tolerance(L, -1.0)], path_tol=[_tolerance(L, 0.005)]) == (
        GoalStatus.STATUS_ABORTED, -4)

    # P3: a goal tolerance with a goal time tolerance of 0 is never ABORTED -5: the goal waits at
    # the end of its trajectory (T = 0.4 s) until it is cancelled, then ends with an empty result
    accepted, handle = h.send_goal(linear, fjt_goal([L], [next(targets)], 0.4, goal_tol=[0.002], goal_time=0.0))
    assert accepted
    assert h.result_of(handle, timeout=1.5) == (None, None)
    future = handle.cancel_goal_async()
    assert h.wait_until(future.done, 5.0)
    status, result = h.result_of(handle)
    assert (status, result.error_code, result.error_string) == (GoalStatus.STATUS_CANCELED, 0, '')


# ================================================================================================
# H5 -- preemption, as the installed controllers end a preempted goal (M9 P5, X7, P7)
# ================================================================================================

CANCELED_EMPTY = (GoalStatus.STATUS_CANCELED, 0, '')


def _watched(goal_handle, ended, name):
    """The result future of a goal; ``ended[name]`` becomes the instant its result arrived.

    The done-callback runs on the executor AFTER ``done()`` turns true: wait on ``ended``.
    """
    future = goal_handle.get_result_async()
    future.add_done_callback(lambda _f: ended.setdefault(name, time.monotonic()))
    return future


def _fjt_end(future):
    response = future.result()
    return response.status, response.result.error_code, response.result.error_string


def test_h5_a_preempted_trajectory_goal_ends_only_when_the_preempting_goal_ends(harness):
    harness.mock('/p1', 'composite_tool')
    probe = harness.helper('h5_fjt_preemption')
    linear = ActionClient(probe, FJT, '/p1/linear_axis_controller/follow_joint_trajectory')

    # P5.preemption: the first goal stays EXECUTING while the second runs, then ends CANCELED with
    # an EMPTY result (no -1, no message); the second succeeds
    ended = {}
    accepted, first = h.send_goal(linear, fjt_goal([L], [0.08], 1.0, goal_tol=[0.002]))
    assert accepted
    time.sleep(0.2)
    sent = time.monotonic()
    accepted, second = h.send_goal(linear, fjt_goal([L], [0.0], 1.0, goal_tol=[0.002]))
    assert accepted
    f1, f2 = _watched(first, ended, 'first'), _watched(second, ended, 'second')
    time.sleep(0.5)
    assert not f1.done() and first.status == GoalStatus.STATUS_EXECUTING
    assert h.wait_until(lambda: len(ended) == 2, 10.0), ended
    assert _fjt_end(f1) == CANCELED_EMPTY
    assert _fjt_end(f2) == (GoalStatus.STATUS_SUCCEEDED, 0, '')
    assert ended['first'] >= sent + 1.0              # never before the second goal's 1.0 s ended

    # X7, and "whatever its outcome": a cancel of the preempted goal is accepted and ends nothing;
    # the preempting goal ending CANCELED (its own client cancels it) ends it
    ended = {}
    accepted, first = h.send_goal(linear, fjt_goal([L], [0.08], 1.0, goal_tol=[0.002]))
    assert accepted
    time.sleep(0.2)
    accepted, second = h.send_goal(linear, fjt_goal([L], [0.08], 5.0, goal_tol=[0.002]))
    assert accepted
    f1, f2 = _watched(first, ended, 'first'), _watched(second, ended, 'second')
    cancel = first.cancel_goal_async()
    assert h.wait_until(cancel.done, 5.0)
    assert cancel.result().return_code == CancelGoal.Response.ERROR_NONE
    assert len(cancel.result().goals_canceling) == 1
    assert h.wait_until(lambda: first.status == GoalStatus.STATUS_CANCELING, 5.0)
    time.sleep(0.3)
    assert not f1.done()
    canceled = time.monotonic()
    assert h.wait_until(second.cancel_goal_async().done, 5.0)
    assert h.wait_until(lambda: len(ended) == 2, 5.0), ended
    assert _fjt_end(f1) == _fjt_end(f2) == CANCELED_EMPTY
    assert ended['first'] >= canceled


def test_h5_a_preempted_gripper_goal_ends_at_once(harness):
    """P7.gac.preemption: unlike the JTC, a GripperActionController ends a preempted goal when the
    newer goal is accepted, CANCELED with an all-default result."""
    harness.mock('/p2', 'jaw_simple', **{'mock.actuators.gripper.travel_s': 2.0})
    probe = harness.helper('h5_gc_preemption')
    jaw = ActionClient(probe, GC, '/p2/gripper_controller/gripper_cmd')
    ended = {}
    accepted, first = h.send_goal(jaw, gc_goal(0.7))
    assert accepted
    time.sleep(0.05)
    accepted, second = h.send_goal(jaw, gc_goal(0.0))
    assert accepted
    f1, f2 = _watched(first, ended, 'first'), _watched(second, ended, 'second')
    assert h.wait_until(lambda: len(ended) == 2, 10.0), ended
    response = f1.result()
    assert response.status == GoalStatus.STATUS_CANCELED
    assert (response.result.position, response.result.effort, response.result.stalled,
            response.result.reached_goal) == (0.0, 0.0, False, False)
    assert f2.result().status == GoalStatus.STATUS_SUCCEEDED
    assert ended['first'] < ended['second'] - 1.0    # long before the newer goal's 2 s travel ended


def test_h5_trajectory_reject_unavailable_and_no_answer(harness):
    harness.mock('/f3', 'composite_tool', **{'mock.actuators.linear_axis.fail': 'reject',
                                             'mock.actuators.rotary_axis.fail': 'unavailable',
                                             'mock.actuators.wrist_mechanism.fail': 'no_answer'})
    harness.mock('/f4', 'composite_tool', **{'mock.actuators.linear_axis.fail': 'applied_unanswered'})
    probe = harness.helper('h5_fjt_refusals')
    w3 = h.StateWatch(probe, '/f3/eef/composite_tool/state')
    w4 = h.StateWatch(probe, '/f4/eef/composite_tool/state')
    linear = ActionClient(probe, FJT, '/f3/linear_axis_controller/follow_joint_trajectory')
    rotary = ActionClient(probe, FJT, '/f3/rotary_axis_controller/follow_joint_trajectory')
    wrist = ActionClient(probe, FJT, '/f3/wrist_mechanism_controller/follow_joint_trajectory')
    unanswered = ActionClient(probe, FJT, '/f4/linear_axis_controller/follow_joint_trajectory')
    m = w3.wait(lambda m: m.status == TS.UNAVAILABLE)
    assert actuator(m, 'rotary_axis').status == AS.UNAVAILABLE
    assert h.send_goal(linear, fjt_goal([L], [0.08], 0.3))[0] is False
    w3.wait(lambda m: 'linear_axis: TARGET rejected: injected reject (mock)' in m.detail)
    assert h.send_goal(rotary, fjt_goal(['robot_a_rotary_axis_joint'], [0.9], 0.3))[0] is False
    assert h.send_goal(wrist, fjt_goal([P, R], [0.3, -0.3], 0.3), timeout=1.5) == (None, None)
    m = w3.latest()
    assert [actuator(m, a).last_command for a in ('linear_axis', 'rotary_axis', 'wrist_mechanism')] == [
        Command.NONE] * 3
    # applied but unanswered: accepted, recorded, no result; a cancel ends it CANCELED
    accepted, handle = h.send_goal(unanswered, fjt_goal([L], [0.08], 0.2))
    assert accepted
    assert h.result_of(handle, timeout=1.0) == (None, None)
    entry = actuator(w4.wait(lambda m: actuator(m, 'linear_axis').last_command == Command.TARGET), 'linear_axis')
    assert entry.status == AS.MOVING
    future = handle.cancel_goal_async()
    assert h.wait_until(future.done, 5.0)
    assert h.result_of(handle)[0] == GoalStatus.STATUS_CANCELED


# ================================================================================================
# H5 -- the GripperActionController emulation
# ================================================================================================

def test_h5_the_gripper_controller_emulation(harness):
    harness.mock('/j1', 'jaw_simple', **{'mock.actuators.gripper.travel_s': 0.3,
                                         'mock.sites.main.decide_after_ms': 50})
    probe = harness.helper('h5_gc')
    w = h.StateWatch(probe, '/j1/eef/jaw_simple/state', depth=500)
    joints = JointWatch(probe, '/j1/eef/jaw_simple/joint_states')
    jaw = ActionClient(probe, GC, '/j1/gripper_controller/gripper_cmd')
    w.wait(lambda m: m.status == TS.READY)
    assert h.wait_until(lambda: joints.latest('finger_joint') == 0.0, 2.0)  # starts at its release target

    start = time.monotonic()
    accepted, handle = h.send_goal(jaw, gc_goal(0.7))                     # "closed": the engage target
    assert accepted
    status, result = h.result_of(handle)
    assert time.monotonic() - start >= 0.25
    assert status == GoalStatus.STATUS_SUCCEEDED
    assert result.reached_goal and not result.stalled and abs(result.position - 0.7) <= 0.001
    assert any(0.05 < x < 0.65 for x in joints.positions['finger_joint'])
    held = w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD)
    assert actuator(held, 'gripper').last_command == Command.ENGAGE
    assert actuator(held, 'gripper').binary == Binary.UNKNOWN            # not a set_bool actuator

    accepted, handle = h.send_goal(jaw, gc_goal(0.35))                   # an unnamed position
    assert h.result_of(handle)[0] == GoalStatus.STATUS_SUCCEEDED
    w.wait(lambda m: actuator(m, 'gripper').last_command == Command.TARGET)

    # a preemption: CANCELED with an empty result; the newer goal succeeds
    accepted, first = h.send_goal(jaw, gc_goal(0.7))
    time.sleep(0.05)
    accepted, second = h.send_goal(jaw, gc_goal(0.0))                    # "open": the release target
    status, result = h.result_of(first)
    assert status == GoalStatus.STATUS_CANCELED
    assert (result.position, result.effort, result.stalled, result.reached_goal) == (0.0, 0.0, False, False)
    assert h.result_of(second)[0] == GoalStatus.STATUS_SUCCEEDED
    w.wait(lambda m: actuator(m, 'gripper').last_command == Command.RELEASE)

    # a cancel: CANCELED with an empty result
    accepted, handle = h.send_goal(jaw, gc_goal(0.7))
    time.sleep(0.05)
    future = handle.cancel_goal_async()
    assert h.wait_until(future.done, 5.0)
    status, result = h.result_of(handle)
    assert status == GoalStatus.STATUS_CANCELED and not result.reached_goal and result.position == 0.0


@pytest.mark.parametrize('fail,status,stalled', [
    ('stall_succeeded', GoalStatus.STATUS_SUCCEEDED, True),
    ('stall_aborted', GoalStatus.STATUS_ABORTED, True),
    ('abort_m1', GoalStatus.STATUS_ABORTED, False),
])
def test_h5_gripper_stall_flags_and_aborts(harness, fail, status, stalled):
    harness.mock('/j2', 'jaw_simple', **{'mock.actuators.gripper.fail': fail,
                                         'mock.actuators.gripper.travel_s': 0.2})
    probe = harness.helper('h5_gc_stall')
    jaw = ActionClient(probe, GC, '/j2/gripper_controller/gripper_cmd')
    accepted, handle = h.send_goal(jaw, gc_goal(0.7, 15.0))
    assert accepted
    got, result = h.result_of(handle)
    assert got == status and result.stalled == stalled and not result.reached_goal
    assert abs(result.position - 0.35) < 1e-6                           # halfway
    assert result.effort == (15.0 if stalled else 0.0)


def test_h5_gripper_canceled_and_rejected(harness):
    harness.mock('/j3', 'jaw_simple', **{'mock.actuators.gripper.fail': 'canceled'})
    harness.mock('/j4', 'jaw_simple', **{'mock.actuators.gripper.fail': 'reject'})
    probe = harness.helper('h5_gc_refusals')
    w4 = h.StateWatch(probe, '/j4/eef/jaw_simple/state')
    canceled = ActionClient(probe, GC, '/j3/gripper_controller/gripper_cmd')
    rejected = ActionClient(probe, GC, '/j4/gripper_controller/gripper_cmd')
    accepted, handle = h.send_goal(canceled, gc_goal(0.7))
    status, result = h.result_of(handle)
    assert status == GoalStatus.STATUS_CANCELED and (result.position, result.reached_goal) == (0.0, False)
    assert h.send_goal(rejected, gc_goal(0.7))[0] is False
    m = w4.wait(lambda m: 'gripper: ENGAGE rejected: injected reject (mock)' in m.detail)
    assert actuator(m, 'gripper').last_command == Command.NONE


# ================================================================================================
# H5 -- SetBool actuators
# ================================================================================================

def test_h5_set_bool_reject_no_answer_applied_unanswered_and_unavailable(harness):
    cases = {'/b1': 'reject', '/b2': 'no_answer', '/b3': 'applied_unanswered', '/b4': 'unavailable'}
    for ns, fail in cases.items():
        harness.mock(ns, 'composite_tool', **{'mock.actuators.gripper_b.fail': fail})
    probe = harness.helper('h5_set_bool')
    watches = {ns: h.StateWatch(probe, f'{ns}/eef/composite_tool/state') for ns in cases}
    clients = {ns: probe.create_client(SetBool, f'{ns}/eef/composite_tool/actuators/gripper_b/set_engaged')
               for ns in cases}
    response = h.call(clients['/b1'], SetBool.Request(data=True))
    assert (response.success, response.message) == (False, 'injected reject (mock)')
    m = watches['/b1'].wait(lambda m: 'gripper_b: ENGAGE rejected: injected reject (mock)' in m.detail)
    assert actuator(m, 'gripper_b').last_command == Command.NONE and actuator(m, 'gripper_b').binary == Binary.RELEASED

    assert h.call(clients['/b2'], SetBool.Request(data=True), timeout=1.0) is None
    m = watches['/b2'].latest()
    assert actuator(m, 'gripper_b').last_command == Command.NONE and m.detail == ''

    assert h.call(clients['/b3'], SetBool.Request(data=True), timeout=1.0) is None
    m = watches['/b3'].wait(lambda m: actuator(m, 'gripper_b').last_command == Command.ENGAGE)
    assert actuator(m, 'gripper_b').binary == Binary.ENGAGED               # the state shows it applied

    m = watches['/b4'].wait(lambda m: m.status == TS.UNAVAILABLE)
    assert actuator(m, 'gripper_b').status == AS.UNAVAILABLE
    response = h.call(clients['/b4'], SetBool.Request(data=True))
    assert response.success is False and 'UNAVAILABLE' in response.message


def test_h5_an_end_switch_confirms_or_does_not(harness):
    harness.mock('/s1', 'composite_tool', **{'mock.actuators.binary_lock.confirm_after_ms': 100})
    harness.mock('/s2', 'composite_tool', **{'mock.actuators.binary_lock.switch_confirms': False})
    probe = harness.helper('h5_switch')
    w1 = h.StateWatch(probe, '/s1/eef/composite_tool/state', depth=500)
    w2 = h.StateWatch(probe, '/s2/eef/composite_tool/state')
    at_rest = actuator(w1.wait(lambda m: m.status == TS.READY), 'binary_lock')
    assert (at_rest.binary, at_rest.binary_confirmed) == (Binary.RELEASED, True)
    c1 = probe.create_client(SetBool, '/s1/eef/composite_tool/actuators/binary_lock/set_engaged')
    c2 = probe.create_client(SetBool, '/s2/eef/composite_tool/actuators/binary_lock/set_engaged')
    assert h.call(c1, SetBool.Request(data=True)).success
    switching = actuator(w1.wait(lambda m: actuator(m, 'binary_lock').last_command == Command.ENGAGE), 'binary_lock')
    assert (switching.binary, switching.binary_confirmed, switching.status) == (Binary.ENGAGED, False, AS.MOVING)
    confirmed = actuator(w1.wait(lambda m: actuator(m, 'binary_lock').binary_confirmed), 'binary_lock')
    assert confirmed.status == AS.IDLE and confirmed.binary == Binary.ENGAGED
    assert h.ns_of(confirmed.stamp) >= h.ns_of(confirmed.last_command_stamp) + 90_000_000
    assert h.call(c2, SetBool.Request(data=True)).success
    w2.wait(lambda m: actuator(m, 'binary_lock').last_command == Command.ENGAGE)
    time.sleep(0.5)
    entry = actuator(w2.latest(), 'binary_lock')
    assert (entry.binary, entry.binary_confirmed, entry.status) == (Binary.ENGAGED, False, AS.MOVING)
    assert w2.latest().status == TS.BUSY


def test_h5_the_gripper_command_suction_handle_is_honest(harness):
    handle_name = 'suctioncup_controller/gripper_command'
    harness.mock('/v1', 'suction_simple', **{'compat.gripper_command_suction_action': handle_name})
    harness.mock('/v2', 'suction_simple', **{'compat.gripper_command_suction_action': handle_name,
                                             'mock.actuators.vacuum.fail': 'reject'})
    probe = harness.helper('h5_suction_handle')
    w1 = h.StateWatch(probe, '/v1/eef/suction_simple/state')
    ok = ActionClient(probe, GC, '/v1/' + handle_name)
    refused = ActionClient(probe, GC, '/v2/' + handle_name)
    accepted, handle = h.send_goal(ok, gc_goal(1.0))
    status, result = h.result_of(handle)
    assert status == GoalStatus.STATUS_SUCCEEDED and result.reached_goal and result.position == 1.0
    w1.wait(lambda m: actuator(m, 'vacuum').last_command == Command.ENGAGE)
    accepted, handle = h.send_goal(ok, gc_goal(0.4))                      # below the 0.5 threshold
    status, result = h.result_of(handle)
    assert status == GoalStatus.STATUS_SUCCEEDED and result.position == 0.0
    w1.wait(lambda m: actuator(m, 'vacuum').last_command == Command.RELEASE)
    accepted, handle = h.send_goal(refused, gc_goal(1.0))
    status, result = h.result_of(handle)
    assert status == GoalStatus.STATUS_ABORTED and not result.reached_goal and result.position == 0.0


# ================================================================================================
# H6 -- the pseudo-joint echo
# ================================================================================================

def test_h6_the_pseudo_joint_echo_never_touches_a_site(harness):
    harness.mock('/', 'suction_simple', **{
        'compat.pseudo_joint_topic': '/suction/joint_states', 'compat.pseudo_joint_name': 'suctioncup_joint',
        'mock.sites.main.outcome': 'not_held', 'mock.sites.main.decide_after_ms': 50})
    harness.mock('/r6', 'suction_simple', **{
        'compat.pseudo_joint_topic': 'suction/joint_states', 'compat.pseudo_joint_name': 'suctioncup_joint',
        'mock.sites.main.outcome': 'unavailable'})
    probe = harness.helper('h6')
    w = h.StateWatch(probe, '/eef/suction_simple/state', depth=500)
    w6 = h.StateWatch(probe, '/r6/eef/suction_simple/state', depth=500)
    echo = JointWatch(probe, '/suction/joint_states')
    echo6 = JointWatch(probe, '/r6/suction/joint_states')
    assert h.wait_until(lambda: echo.latest('suctioncup_joint') == 0.0, 3.0)    # released, as the bridge
    client = probe.create_client(SetBool, '/eef/suction_simple/actuators/vacuum/set_engaged')
    assert h.call(client, SetBool.Request(data=True)).success
    assert h.wait_until(lambda: echo.latest('suctioncup_joint') == 1.0, 2.0)
    w.wait(lambda m: site(m, 'main')[0] == Grasp.NOT_HELD and actuator(m, 'vacuum').last_command == Command.ENGAGE)
    assert echo.latest('suctioncup_joint') == 1.0                    # pseudo-joint position != attached
    assert h.call(client, SetBool.Request(data=False)).success
    assert h.wait_until(lambda: echo.latest('suctioncup_joint') == 0.0, 2.0)

    client6 = probe.create_client(SetBool, '/r6/eef/suction_simple/actuators/vacuum/set_engaged')
    assert h.call(client6, SetBool.Request(data=True)).success
    assert h.wait_until(lambda: echo6.latest('suctioncup_joint') == 1.0, 2.0)
    time.sleep(0.2)
    for sample in w6.all():
        assert site(sample, 'main') == (Grasp.UNAVAILABLE, Evidence.NONE, 0)
    for sample in w.all() + w6.all():
        assert all('suctioncup_joint' not in a.joints for a in sample.actuators)
    topics, _ = h.graph_names(probe)
    assert '/eef/suction_simple/joint_states' not in topics             # the vacuum owns no joint
    assert not any('attached' in t for t in topics)


# ================================================================================================
# the scene signal
# ================================================================================================

def test_the_scene_signal_carries_the_applied_engage(harness):
    harness.mock('/r7', 'suction_simple', **{'mock.sites.main.outcome': 'not_held'})
    harness.mock('/r8', 'suction_simple', **{'mock.actuators.vacuum.fail': 'reject'})
    harness.mock('/r9', 'jaw_simple')                                     # scene_signal: tmr
    probe = harness.helper('scene_signal')
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    got7, got8 = [], []
    probe.create_subscription(Bool, '/r7/eef/suction_simple/engaged', lambda m: got7.append(m.data), latched)
    probe.create_subscription(Bool, '/r8/eef/suction_simple/engaged', lambda m: got8.append(m.data), latched)
    c7 = probe.create_client(SetBool, '/r7/eef/suction_simple/actuators/vacuum/set_engaged')
    c8 = probe.create_client(SetBool, '/r8/eef/suction_simple/actuators/vacuum/set_engaged')
    time.sleep(0.5)
    assert got7 == [] and got8 == []                                      # nothing applied yet
    assert h.call(c7, SetBool.Request(data=True)).success                 # applied, not held: still true
    assert h.wait_until(lambda: got7 == [True], 2.0)
    assert h.call(c7, SetBool.Request(data=False)).success
    assert h.wait_until(lambda: got7 == [True, False], 2.0)
    assert h.call(c8, SetBool.Request(data=True)).success is False
    time.sleep(0.3)
    assert got8 == []
    late = []
    probe.create_subscription(Bool, '/r7/eef/suction_simple/engaged', lambda m: late.append(m.data), latched)
    assert h.wait_until(lambda: late == [False], 2.0)                     # latched for a late joiner
    topics, _ = h.graph_names(probe)
    assert '/r9/eef/jaw_simple/engaged' not in topics                    # TMR is that device's source


# ================================================================================================
# the first sample
# ================================================================================================

def test_an_injected_condition_from_t0_is_in_the_first_latched_sample(harness):
    """fail_after_s 0: the sample the constructor publishes (latched) already carries the condition.
    The adapters are never spun, so that sample is the only one a subscriber can receive."""
    cases = {'/t1': ('fault', TS.FAULT, AS.FAULT), '/t2': ('unavailable', TS.UNAVAILABLE, AS.UNAVAILABLE)}
    for ns, (fail, _status, _entry) in cases.items():
        harness.mock(ns, 'composite_tool', spin=False, **{'mock.actuators.gripper_b.fail': fail})
    probe = harness.helper('first_sample')
    watches = {ns: h.StateWatch(probe, f'{ns}/eef/composite_tool/state') for ns in cases}
    for ns, (fail, status, entry) in cases.items():
        first = watches[ns].wait(lambda m: True, what='from the constructor')
        assert watches[ns].count() == 1, ns
        assert first.status == status, h.describe(first)
        assert actuator(first, 'gripper_b').status == entry
        assert f'gripper_b: injected {entry.name} (mock)' in first.detail, first.detail


# ================================================================================================
# refusals at start-up
# ================================================================================================

@pytest.mark.parametrize('device_id,params,key', [
    ('composite_tool', {'mock.actuators.gripper_b.fail': 'abort_m4'}, 'mock.actuators.gripper_b.fail'),
    ('composite_tool', {'mock.actuators.gripper_a.fail': 'blocked'}, 'mock.actuators.gripper_a.fail'),
    ('composite_tool', {'mock.actuators.ghost.fail': 'reject'}, 'mock.actuators.ghost.fail'),
    ('composite_tool', {'mock.sites.site_a.colour': 'red'}, 'mock.sites.site_a.colour'),
    ('composite_tool', {'mock.sites.site_a.outcome': 'maybe'}, 'mock.sites.site_a.outcome'),
    ('composite_tool', {'mock.sites.site_a.evidence': 'command_echo'}, 'mock.sites.site_a.evidence'),
    ('composite_tool', {'mock.sample_rate_hz': 1.0}, 'mock.sample_rate_hz'),
    ('composite_tool', {'mock.actuators.linear_axis.travel_s': 0.1}, 'mock.actuators.linear_axis.travel_s'),
    ('composite_tool', {'mock.actuators.gripper_b.switch_confirms': True},
     'mock.actuators.gripper_b.switch_confirms'),
    ('composite_tool', {'mock.actuators.rotary_axis.initial_target': 'nowhere'},
     'mock.actuators.rotary_axis.initial_target'),
    ('composite_tool', {'compat.trigger_on': '/suction/on', 'compat.trigger_off': '/suction/off'},
     'compat.trigger_on'),
    ('suction_simple', {'compat.trigger_on': '/suction/on'}, 'compat.trigger_on'),
    ('suction_simple', {'compat.unknown_endpoint': 'x'}, 'compat.unknown_endpoint'),
    ('suction_simple', {'publish_rate_hz': 0.0}, 'publish_rate_hz'),
    ('jaw_simple', {'compat.trigger_on': '/suction/on', 'compat.trigger_off': '/suction/off'},
     'compat.trigger_on'),
    ('suction_simple', {'device_id': 7}, 'device_id'),
    ('suction_simple', {'device_id': 'jaw_simple'}, 'eef.jaw_simple.actuator_ids'),
])
def test_an_adapter_refuses_a_bad_parameter_and_leaves_nothing_behind(harness, device_id, params, key):
    block = h.device_block(device_id)
    params = dict(params)
    if 'device_id' in params:               # a bad device_id itself rides in the block
        block['device_id'] = params.pop('device_id')
    with pytest.raises(Refused) as e:
        harness.mock('/rx', device_id, block, **params)
    assert e.value.key == key, e.value
    assert not harness.session.visible() or h.wait_until(lambda: not harness.session.visible(), 5.0)


def test_close_waits_for_the_goals_it_is_executing(harness):
    """An executor waits for its callbacks but not for goal-execution tasks: close() must."""
    adapter = harness.mock('/r10', 'composite_tool')
    probe = harness.helper('close_waits')
    axis = ActionClient(probe, FJT, '/r10/linear_axis_controller/follow_joint_trajectory')
    accepted, handle = h.send_goal(axis, fjt_goal([L], [0.08], 5.0, goal_tol=[0.002]))
    assert accepted
    # counted by the accepted-goal callback, which rclpy runs after it sent the goal response
    assert h.wait_until(lambda: adapter._executions == 1, 5.0), adapter._executions
    started = time.monotonic()
    adapter.close()
    assert time.monotonic() - started < 2.0
    assert adapter._executions == 0
    status, _result = h.result_of(handle, timeout=5.0)
    assert status == GoalStatus.STATUS_ABORTED                           # ended by the shutdown
