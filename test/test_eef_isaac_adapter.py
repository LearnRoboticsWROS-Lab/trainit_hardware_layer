"""H7 -- the Isaac adapter's logic without Kit (domain 86; ADR-0022 D8, EEF-D5 scenario b).

A fake stage made of plain rclpy endpoints stands in for the OmniGraph side: a VOLATILE subscriber
on the stage command topic (the stage's subscription is volatile) and a periodic std_msgs/String
"<Open|Closing|Closed> <count>" status publisher. No Isaac, no Kit, no USD.

Covered: the adapter starts released and republishes its command; no stage subscriber gives
``success: false`` and UNAVAILABLE; without a status topic the site is UNAVAILABLE / NONE (no
grasp_detection, never a grasp from the echo); the status is mapped by NAME (Closed / Closing /
Open with the gripped count; a partial Closing with a count is HELD; PENDING within the engage
timeout, then NOT_HELD or LOST); a malformed or contradictory sample reads UNKNOWN, and no sample
can raise in the callback and stop the adapter (non-ASCII digits, an over-long count, a failing
mapping); a status timeout gives UNKNOWN / NONE; a status topic that never publishes is named in
detail; the per-robot reset is a release; a rejected release does not disarm LOST; use_sim_time
stamps on the simulation clock; the start-up refusals. From the T5 Isaac run: a subscriber alone
is not the stage; a dead stage (its /clock stopped) reads UNKNOWN on the wall clock; the one-frame
"Closed 0" of a re-grip is never a false LOST.
"""
import random
import time

import pytest

import eef_harness as h
from eef_harness import actuator, site

rclpy = pytest.importorskip('rclpy', reason='the adapter tests need a sourced ROS 2 Humble')

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402

from trainit_hardware_layer.eef import isaac_eef_adapter  # noqa: E402
from trainit_hardware_layer.eef.contract import Refused  # noqa: E402
from trainit_hardware_layer.eef.core import ActuatorStatus as AS  # noqa: E402
from trainit_hardware_layer.eef.core import Binary, Capability, Command, Evidence, Grasp, Reading  # noqa: E402
from trainit_hardware_layer.eef.core import ToolStatus as TS  # noqa: E402
from trainit_hardware_layer.eef.isaac_eef_adapter import map_stage_status  # noqa: E402

CMD = 'stage/suction_cmd'
STATUS = 'stage/suction_status'
SIM = Evidence.SIM_GROUND_TRUTH
VOLATILE = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


class FakeStage:
    """The stage side of the reference cell, as plain topics."""

    def __init__(self, harness, namespace, subscribe=True, status_hz=20.0, publishing=True):
        self.node = harness.helper('fake_stage', namespace=namespace)
        self.commands = []
        self.text = 'Open 0'
        self.publishing = publishing
        self.sub = None
        if subscribe:
            self.subscribe()
        self.pub = self.node.create_publisher(String, STATUS, 10)
        self.node.create_timer(1.0 / status_hz, self._tick)

    def subscribe(self):
        self.sub = self.node.create_subscription(Bool, CMD, lambda m: self.commands.append(m.data), VOLATILE)

    def unsubscribe(self):
        self.node.destroy_subscription(self.sub)
        self.sub = None

    def _tick(self):
        if self.publishing:
            self.pub.publish(String(data=self.text))


def _client(probe, ns):
    return probe.create_client(SetBool, f'{ns}/eef/suction_simple/actuators/vacuum/set_engaged')


def test_h7_the_mapping_is_by_name():
    second = 1_000_000_000
    assert map_stage_status('Closed 1', False, 0, second) == (Reading.HELD, '')
    assert map_stage_status('Closing 2', True, 0, second) == (Reading.HELD, 'partial attachment (Closing 2)')
    assert map_stage_status('Closing 0', True, 0, second) == (Reading.UNDECIDED, '')
    assert map_stage_status('Open 0', True, second // 2, second) == (Reading.UNDECIDED, '')
    assert map_stage_status('Open 0', True, 2 * second, second) == (Reading.NOT_HELD, '')
    # Isaac 5.1 reports "Closed 0" for one frame on every attach (a re-grip included): after an
    # engage it is a transient, never a decision; with no engage it is nothing held
    assert map_stage_status('Closed 0', True, 2 * second, second) == (Reading.UNDECIDED, '')
    assert map_stage_status('Closed 0', False, 0, second) == (Reading.NOT_HELD, '')
    assert map_stage_status('Open 0', False, 0, second) == (Reading.NOT_HELD, '')
    # never by a literal int, and never a guess on a contradiction
    for text in ('2 1', '0 0', 'Closed', 'Closed -1', 'closed 1', 'Open x', 'Closed 1 extra', ''):
        assert map_stage_status(text, True, 0, second)[0] == Reading.CANNOT_TELL, text
    assert map_stage_status('Open 3', True, 0, second) == (Reading.CANNOT_TELL, "contradictory stage status 'Open 3'")
    # the count is plain ASCII digits: str.isdigit() also takes '²' and '①' (int() raises on them)
    # and '٣' (int() reads 3); more digits than int() converts raises too
    for text in ('Closed ²', 'Open ①', 'Closed ٣', 'Closed ' + '1' * 5000):
        reading, note = map_stage_status(text, True, 0, second)
        assert reading == Reading.CANNOT_TELL and note.startswith('malformed stage status '), text
        assert len(note) < 100, note                         # detail quotes a bounded part of it
    assert map_stage_status('Closed ²', True, 0, second)[1] == "malformed stage status 'Closed ²'"


def test_h7_the_mapping_never_raises():
    """A raise would escape the status callback and stop the adapter: every text has a reading."""
    rng = random.Random(20261010)
    second = 1_000_000_000
    names = ['Open', 'Closing', 'Closed', 'closed', '', '²', 'Open ']
    counts = ['0', '1', '12', '-1', '+1', '1.0', '²', '٣', '①', '1' * 4400, '', ' ', '\x00']
    tails = ['', ' x', '\n', ' ', ' 1']
    for _ in range(3000):
        text = rng.choice(names) + rng.choice([' ', '\t', ' ', '', '  ']) + rng.choice(counts) + rng.choice(tails)
        reading, note = map_stage_status(text, rng.random() < 0.5, rng.randrange(2 * second), second)
        assert isinstance(reading, Reading) and isinstance(note, str) and len(note) < 400, text[:80]


def test_h7_starts_released_republishes_and_needs_a_stage_subscriber(harness):
    adapter = harness.isaac('/i1', stage_cmd_topic=CMD)
    probe = harness.helper('h7_cmd')
    w = h.StateWatch(probe, '/i1/eef/suction_simple/state', depth=500)
    client = _client(probe, '/i1')
    m = w.wait(lambda m: m.status == TS.UNAVAILABLE)
    assert m.adapter == 'trainit_isaac_surface_gripper' and m.end_effector == '/i1/eef/suction_simple'
    assert actuator(m, 'vacuum').status == AS.UNAVAILABLE
    assert m.detail == 'vacuum: no stage subscriber on /i1/stage/suction_cmd'
    assert site(m, 'main') == (Grasp.UNAVAILABLE, Evidence.NONE, 0)       # no status topic
    response = h.call(client, SetBool.Request(data=True))
    assert (response.success, response.message) == (False, 'no stage subscriber on /i1/stage/suction_cmd')
    m = w.wait(lambda m: 'ENGAGE rejected' in m.detail)
    assert actuator(m, 'vacuum').last_command == Command.NONE

    stage = FakeStage(harness, '/i1')
    w.wait(lambda m: m.status == TS.READY, timeout=6.0)
    assert h.wait_until(lambda: len(stage.commands) >= 3, 3.0)          # republished at 2 Hz
    assert set(stage.commands) == {False}                                 # it starts released
    response = h.call(client, SetBool.Request(data=True))
    assert (response.success, response.message) == (True, 'engaged')
    assert h.wait_until(lambda: stage.commands[-1] is True, 2.0)
    n = len(stage.commands)
    assert h.wait_until(lambda: len(stage.commands) >= n + 2 and stage.commands[-1] is True, 3.0)
    m = w.wait(lambda m: actuator(m, 'vacuum').last_command == Command.ENGAGE)
    entry = actuator(m, 'vacuum')
    assert (entry.binary, entry.binary_confirmed, entry.status) == (Binary.ENGAGED, False, AS.IDLE)
    assert m.detail == ''
    time.sleep(0.3)
    for sample in w.all():                                                # never a grasp from the echo
        assert site(sample, 'main') == (Grasp.UNAVAILABLE, Evidence.NONE, 0)
    assert Capability.GRASP_DETECTION not in adapter.provided_capabilities()
    assert adapter.provided_capabilities() == {Capability.BINARY_ACTUATION, Capability.CONTINUOUS_STATE}


def test_h7_the_stage_status_drives_the_site(harness):
    adapter = harness.isaac('/i2', stage_cmd_topic=CMD, stage_status_topic=STATUS,
                            engage_timeout_ms=300, status_timeout_ms=400)
    stage = FakeStage(harness, '/i2')
    probe = harness.helper('h7_status')
    w = h.StateWatch(probe, '/i2/eef/suction_simple/state', depth=2000)
    client = _client(probe, '/i2')
    assert Capability.GRASP_DETECTION in adapter.provided_capabilities()
    w.wait(lambda m: m.status == TS.READY and site(m, 'main')[:2] == (Grasp.NOT_HELD, SIM), timeout=6.0)

    assert h.call(client, SetBool.Request(data=True)).success
    stage.text = 'Closing 0'
    w.wait(lambda m: site(m, 'main')[:2] == (Grasp.PENDING, SIM))
    stage.text = 'Closing 2'                                             # partial, with a count: HELD
    m = w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD)
    assert m.detail == 'main: partial attachment (Closing 2)' and site(m, 'main')[1] == SIM
    stage.text = 'Closed 1'
    w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD and m.detail == '')
    stage.text = 'Open 0'                                                # dropped, no release: LOST
    lost = w.wait(lambda m: site(m, 'main')[0] == Grasp.LOST)
    assert site(lost, 'main')[1] == SIM and actuator(lost, 'vacuum').last_command == Command.ENGAGE
    time.sleep(0.2)
    assert site(w.latest(), 'main')[0] == Grasp.LOST                     # latched

    assert h.call(client, SetBool.Request(data=False)).success
    assert h.wait_until(lambda: stage.commands and stage.commands[-1] is False, 2.0)
    w.wait(lambda m: site(m, 'main')[0] == Grasp.NOT_HELD and actuator(m, 'vacuum').binary == Binary.RELEASED)

    # an engage that grips nothing: PENDING within the engage timeout, then NOT_HELD (not LOST)
    assert h.call(client, SetBool.Request(data=True)).success
    engage = h.ns_of(actuator(w.wait(lambda m: actuator(m, 'vacuum').last_command == Command.ENGAGE),
                              'vacuum').last_command_stamp)
    pending = w.wait(lambda m: site(m, 'main')[0] == Grasp.PENDING and site(m, 'main')[2] >= engage)
    assert site(pending, 'main')[1] == SIM
    settled = w.wait(lambda m: site(m, 'main')[0] == Grasp.NOT_HELD and site(m, 'main')[2] >= engage)
    assert site(settled, 'main')[2] - engage >= 300_000_000 - 50_000_000

    # malformed and contradictory samples read UNKNOWN, named in detail
    stage.text = '2 1'
    m = w.wait(lambda m: site(m, 'main')[0] == Grasp.UNKNOWN)
    assert site(m, 'main')[1] == Evidence.NONE and m.detail == "main: malformed stage status '2 1'"
    stage.text = 'Open 3'
    w.wait(lambda m: m.detail == "main: contradictory stage status 'Open 3'")
    # a silent stage: UNKNOWN / NONE after the status timeout, the stamp never advanced
    stage.text = 'Open 0'
    w.wait(lambda m: site(m, 'main')[0] == Grasp.NOT_HELD)
    stage.publishing = False
    silent = w.wait(lambda m: site(m, 'main')[0] == Grasp.UNKNOWN and m.status == TS.UNAVAILABLE,
                    timeout=3.0)
    assert site(silent, 'main')[1] == Evidence.NONE
    assert silent.detail == ('vacuum: no stage status on /i2/stage/suction_status for 400 ms (a '
                             'subscriber alone is not the stage); '
                             'main: no deciding-source sample for more than 400 ms')
    stamp = site(silent, 'main')[2]
    time.sleep(0.3)
    assert site(w.latest(), 'main')[2] == stamp                         # heartbeats never re-stamp
    stage.publishing = True
    w.wait(lambda m: site(m, 'main')[0] == Grasp.NOT_HELD and m.detail == '' and m.status == TS.READY)


def test_h7_no_stage_sample_can_stop_the_adapter(harness, monkeypatch):
    """A raise in the status callback would stop the executor (in a process: exit 1): a malformed
    sample reads UNKNOWN, named in detail, the state keeps flowing, and a good sample counts again."""
    harness.isaac('/i7', stage_cmd_topic=CMD, stage_status_topic=STATUS)
    stage = FakeStage(harness, '/i7')
    probe = harness.helper('h7_malformed')
    w = h.StateWatch(probe, '/i7/eef/suction_simple/state', depth=2000)
    w.wait(lambda m: m.status == TS.READY and site(m, 'main')[:2] == (Grasp.NOT_HELD, SIM), timeout=6.0)
    for text in ('Closed ²', 'Open ①', 'Closed ٣'):
        stage.text = text
        m = w.wait(lambda m, t=text: m.detail == f'main: malformed stage status {t!r}', what=text)
        assert site(m, 'main')[:2] == (Grasp.UNKNOWN, Evidence.NONE) and m.status == TS.READY
    stage.text = 'Closed ' + '1' * 5000
    m = w.wait(lambda m: m.detail.startswith("main: malformed stage status 'Closed 111"))
    assert len(m.detail) < 120
    stage.text = 'Open 0'
    w.wait(lambda m: site(m, 'main')[:2] == (Grasp.NOT_HELD, SIM) and m.detail == '')

    def broken(*_args):
        raise RuntimeError('a defect in the mapping')
    monkeypatch.setattr(isaac_eef_adapter, 'map_stage_status', broken)   # even a defect cannot
    m = w.wait(lambda m: m.detail == "main: malformed stage status 'Open 0' (RuntimeError)")
    assert site(m, 'main')[:2] == (Grasp.UNKNOWN, Evidence.NONE)
    monkeypatch.undo()
    w.wait(lambda m: site(m, 'main')[:2] == (Grasp.NOT_HELD, SIM) and m.detail == '')


def test_h7_a_status_topic_that_never_publishes_is_named(harness):
    """A wrong stage_status_topic: nothing ever arrives. The site stays UNKNOWN / NONE with no
    stamp; the vacuum is UNAVAILABLE (a stage that does not talk is not there), and detail names
    the site once status_timeout_ms (400) has passed."""
    harness.isaac('/i6', stage_cmd_topic=CMD, stage_status_topic=STATUS)
    stage = FakeStage(harness, '/i6', publishing=False)
    probe = harness.helper('h7_unheard')
    w = h.StateWatch(probe, '/i6/eef/suction_simple/state', depth=500)
    m = w.wait(lambda m: m.detail.endswith('main: no sample from its source yet'), timeout=6.0)
    assert site(m, 'main') == (Grasp.UNKNOWN, Evidence.NONE, 0)
    # with a status topic, the stage is there only when it TALKS: the subscriber alone is not it
    assert m.status == TS.UNAVAILABLE and m.detail.startswith(
        'vacuum: no stage status on /i6/stage/suction_status for 400 ms')
    stage.publishing = True
    w.wait(lambda m: site(m, 'main')[:2] == (Grasp.NOT_HELD, SIM) and m.detail == ''
           and m.status == TS.READY)


def test_h7_the_per_robot_reset_is_a_release(harness):
    harness.isaac('/i3', stage_cmd_topic=CMD, stage_status_topic=STATUS, scene_reset_topic='scene_reset')
    stage = FakeStage(harness, '/i3')
    probe = harness.helper('h7_reset')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    reset = probe.create_publisher(Bool, '/i3/scene_reset', latched)       # as scene_manager_node
    w = h.StateWatch(probe, '/i3/eef/suction_simple/state', depth=2000)
    client = _client(probe, '/i3')
    w.wait(lambda m: m.status == TS.READY, timeout=6.0)
    assert h.call(client, SetBool.Request(data=True)).success
    stage.text = 'Closed 1'
    w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD)
    assert h.wait_until(lambda: probe.count_subscribers('/i3/scene_reset') == 1, 3.0)
    reset.publish(Bool(data=True))                                       # the rising edge
    m = w.wait(lambda m: actuator(m, 'vacuum').last_command == Command.RELEASE)
    assert actuator(m, 'vacuum').binary == Binary.RELEASED
    assert h.wait_until(lambda: stage.commands and stage.commands[-1] is False, 2.0)
    stage.text = 'Open 0'                                                # the stage opened the gripper
    w.wait(lambda m: site(m, 'main')[0] == Grasp.NOT_HELD)
    reset.publish(Bool(data=False))
    stamp = h.ns_of(actuator(w.latest(), 'vacuum').last_command_stamp)
    reset.publish(Bool(data=False))                                      # no edge: nothing recorded
    time.sleep(0.3)
    assert h.ns_of(actuator(w.latest(), 'vacuum').last_command_stamp) == stamp
    assert all(site(s, 'main')[0] != Grasp.LOST for s in w.all())


def test_h7_a_rejected_release_does_not_disarm_lost(harness):
    harness.isaac('/i4', stage_cmd_topic=CMD, stage_status_topic=STATUS)
    stage = FakeStage(harness, '/i4')
    probe = harness.helper('h7_rejected_release')
    w = h.StateWatch(probe, '/i4/eef/suction_simple/state', depth=2000)
    client = _client(probe, '/i4')
    w.wait(lambda m: m.status == TS.READY, timeout=6.0)
    assert h.call(client, SetBool.Request(data=True)).success
    stage.text = 'Closed 1'
    w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD)
    stage.unsubscribe()                                                  # the stage stops listening
    w.wait(lambda m: m.status == TS.UNAVAILABLE, timeout=6.0)
    response = h.call(client, SetBool.Request(data=False))
    assert response.success is False and 'no stage subscriber' in response.message
    m = w.wait(lambda m: 'vacuum: RELEASE rejected' in m.detail)
    assert actuator(m, 'vacuum').last_command == Command.ENGAGE
    stage.text = 'Open 0'                                                # the part falls anyway
    lost = w.wait(lambda m: site(m, 'main')[0] == Grasp.LOST)
    assert site(lost, 'main')[1] == SIM


def test_h7_use_sim_time_stamps_on_the_simulation_clock(harness):
    harness.isaac('/i5', stage_cmd_topic=CMD, stage_status_topic=STATUS, use_sim_time=True)
    stage = FakeStage(harness, '/i5')
    probe = harness.helper('h7_sim_time')
    w = h.StateWatch(probe, '/i5/eef/suction_simple/state', depth=500)
    client = _client(probe, '/i5')
    response = h.call(client, SetBool.Request(data=True))                # no /clock yet: time is 0
    assert response.success is False and 'no ROS time yet' in response.message
    clock = probe.create_publisher(Clock, '/clock', 10)
    sim = [1000 * 1_000_000_000]

    def tick():
        sim[0] += 10_000_000
        clock.publish(Clock(clock=Time(sec=sim[0] // 1_000_000_000, nanosec=sim[0] % 1_000_000_000)))
    probe.create_timer(0.01, tick)
    w.wait(lambda m: m.status == TS.READY and site(m, 'main')[0] == Grasp.NOT_HELD, timeout=6.0)
    assert h.call(client, SetBool.Request(data=True)).success
    stage.text = 'Closed 1'
    m = w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD)
    stamp = site(m, 'main')[2]
    command_stamp = h.ns_of(actuator(m, 'vacuum').last_command_stamp)
    wall = time.time_ns()
    assert 1000 * 10 ** 9 < command_stamp < stamp < 1100 * 10 ** 9 < wall   # the sim clock, not wall


@pytest.mark.parametrize('device_id,params,key', [
    ('composite_tool', {'stage_cmd_topic': CMD}, 'eef.composite_tool'),
    ('jaw_simple', {'stage_cmd_topic': CMD}, 'eef.jaw_simple'),
    ('suction_simple', {}, 'stage_cmd_topic'),
    ('suction_simple', {'stage_cmd_topic': CMD, 'engage_timeout_ms': 5000}, 'engage_timeout_ms'),
    ('suction_simple', {'stage_cmd_topic': CMD, 'status_timeout_ms': 600}, 'status_timeout_ms'),
    ('suction_simple', {'stage_cmd_topic': CMD, 'republish_rate_hz': 0.0}, 'republish_rate_hz'),
    ('suction_simple', {'stage_cmd_topic': 'bad topic!'}, 'stage_cmd_topic'),
    ('suction_simple', {'stage_cmd_topic': CMD, 'mock.sites.main.outcome': 'held'}, None),
])
def test_h7_isaac_start_up_refusals(harness, device_id, params, key):
    if key is None:            # a mock.* key is not the isaac adapter's: a warning, not a refusal
        adapter = harness.isaac('/iy', device_id, **params)
        assert adapter.end_effector == '/iy/eef/suction_simple'
        return
    with pytest.raises(Refused) as e:
        harness.isaac('/ix', device_id, **params)
    assert e.value.key == key, e.value


def test_h7_actuation_feedback_is_refused_on_isaac(harness):
    block = h.device_block('suction_simple', changes={'actuators.vacuum.actuation_feedback': True})
    with pytest.raises(Refused) as e:
        harness.isaac('/ix', 'suction_simple', block, stage_cmd_topic=CMD)
    assert e.value.key == 'eef.suction_simple.actuators.vacuum.actuation_feedback'


def test_h7_a_subscriber_alone_is_not_the_stage(harness):
    """T5 found it: any subscriber on the stage command topic (a recorder, an echo) made the
    adapter answer success with no stage. With a status topic the stage must also TALK."""
    harness.isaac('/i8', stage_cmd_topic=CMD, stage_status_topic=STATUS)
    stage = FakeStage(harness, '/i8', publishing=False)           # subscribes, never reports
    probe = harness.helper('h7_mute_stage')
    w = h.StateWatch(probe, '/i8/eef/suction_simple/state', depth=500)
    client = _client(probe, '/i8')
    w.wait(lambda m: m.status == TS.UNAVAILABLE, timeout=6.0)
    response = h.call(client, SetBool.Request(data=True))
    assert response.success is False
    assert response.message.startswith('no stage status on /i8/stage/suction_status for 400 ms')
    assert actuator(w.latest(), 'vacuum').last_command == Command.NONE  # nothing applied
    stage.publishing = True
    w.wait(lambda m: m.status == TS.READY, timeout=6.0)
    assert h.call(client, SetBool.Request(data=True)).success


def test_h7_a_dead_stage_reads_unknown_even_with_its_clock_stopped(harness):
    """T5 found it: a stage that dies stops /clock, and with it every sim-time silence: the adapter
    kept publishing its last HELD with a frozen stamp, which a consumer on the same frozen clock
    judges fresh. Silence is measured on the WALL clock too."""
    harness.isaac('/i9', stage_cmd_topic=CMD, stage_status_topic=STATUS, use_sim_time=True)
    stage = FakeStage(harness, '/i9')
    probe = harness.helper('h7_dead_stage')
    w = h.StateWatch(probe, '/i9/eef/suction_simple/state', depth=2000)
    client = _client(probe, '/i9')
    clock = probe.create_publisher(Clock, '/clock', 10)
    sim = [1000 * 1_000_000_000]
    running = [True]

    def tick():
        if running[0]:
            sim[0] += 10_000_000
            clock.publish(Clock(clock=Time(sec=sim[0] // 1_000_000_000, nanosec=sim[0] % 1_000_000_000)))
    probe.create_timer(0.01, tick)
    w.wait(lambda m: m.status == TS.READY and site(m, 'main')[0] == Grasp.NOT_HELD, timeout=6.0)
    assert h.call(client, SetBool.Request(data=True)).success
    stage.text = 'Closed 1'
    held = w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD)
    running[0] = False                       # the stage dies: no /clock, no status
    stage.publishing = False
    gone = w.wait(lambda m: site(m, 'main')[0] == Grasp.UNKNOWN, timeout=3.0)
    assert site(gone, 'main')[1] == Evidence.NONE and site(gone, 'main')[2] == site(held, 'main')[2]
    assert gone.status == TS.UNAVAILABLE                                  # and the stage is gone
    time.sleep(0.5)
    assert all(site(m, 'main')[0] != Grasp.HELD for m in w.since(gone))


def test_h7_a_re_grip_after_a_slip_is_never_a_false_lost(harness):
    """T5 found it: after the engage window, a slip (Closing 0) then a re-grip passes through the
    one-frame "Closed 0" Isaac reports on every attach; read as NOT_HELD it latched a LOST while
    the stage held the part again. "Closed 0" after an engage decides nothing."""
    harness.isaac('/i10', stage_cmd_topic=CMD, stage_status_topic=STATUS, engage_timeout_ms=300)
    stage = FakeStage(harness, '/i10', status_hz=50.0)
    probe = harness.helper('h7_regrip')
    w = h.StateWatch(probe, '/i10/eef/suction_simple/state', depth=4000)
    client = _client(probe, '/i10')
    w.wait(lambda m: m.status == TS.READY and site(m, 'main')[0] == Grasp.NOT_HELD, timeout=6.0)
    assert h.call(client, SetBool.Request(data=True)).success
    stage.text = 'Closed 1'
    held = w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD)
    time.sleep(0.5)                                                       # past the engage window
    for text, dwell in (('Closing 0', 0.15), ('Closed 0', 0.1), ('Closed 1', 0.3)):
        stage.text = text
        time.sleep(dwell)
    w.wait(lambda m: site(m, 'main')[0] == Grasp.HELD and site(m, 'main')[2] > site(held, 'main')[2])
    assert all(site(m, 'main')[0] != Grasp.LOST for m in w.since(held))
    # a real loss still latches: the gripper gives up (Open 0) after the window
    stage.text = 'Open 0'
    w.wait(lambda m: site(m, 'main')[0] == Grasp.LOST)
