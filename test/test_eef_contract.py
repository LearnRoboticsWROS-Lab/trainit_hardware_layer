"""Conformance of the hardware layer's ``eef`` parser (ADR-0022 D8; vNext M2.6 spec section 4).

1. The four byte copies in test/data/m2_6_contract are the pinned bytes.
2. ``contract.py``'s normalised dump of the pinned fixture equals the pinned expected dump, byte for
   byte; so does the dump of the pinned reference normaliser (the copy IS the reference).
3. A mutation table (every refusal row of the reference, each with the key it must give, plus the
   variants that must be accepted unchanged) and a systematic sweep (every leaf key deleted, given
   a value of every type, given an unknown sibling) give the SAME outcome in ``contract.py`` and in
   the reference: the same refusal key and reason, or the same dump.
4. The rclpy paths give the same dump: parameter overrides, and a ``--params-file`` (how launch
   hands a node its parameters).
5. ``device_for_adapter``: an adapter reads the same block for its own device, with or without
   ``eef.ids``, and refuses what is not its device.
"""
import copy
import importlib.util

import pytest

import eef_harness as h
from trainit_hardware_layer.eef import contract


def _reference():
    spec = importlib.util.spec_from_file_location('normalize_eef_reference',
                                                  h.pinned('normalize_eef.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REF = _reference()
EXPECTED = h.pinned('eef_conformance.expected.txt').read_bytes().decode('utf-8')


def outcome(module, flat):
    """('refused', key, message), ('ok', dump) or ('crash', type) of one parser on one block."""
    try:
        normalized = module.parse(copy.deepcopy(flat))
    except Exception as e:  # noqa: BLE001 -- each parser raises its own Refused
        if type(e).__name__ != 'Refused':
            return ('crash', type(e).__name__)
        return ('refused', e.key, str(e))
    return ('ok', module.dump(normalized))


def same_outcome(ours, reference):
    # The reference defines an outcome for every input (its first copy raised TypeError on a list
    # in sites.<s>.grip / .engage / .release; fixed at the source, README.md): a crash is never
    # an outcome, in either parser.
    return reference[0] != 'crash' and ours == reference


# ------------------------------------------------------------------------------------------------
# 1, 2: the bytes and the dump
# ------------------------------------------------------------------------------------------------

@pytest.mark.parametrize('name', sorted(h.PINS))
def test_byte_copies_are_the_pinned_bytes(name):
    assert h.sha256(h.DATA / name) == h.PINS[name]


def test_fixture_dump_equals_the_pinned_expected_dump():
    flat = contract.load_params(str(h.pinned('eef_conformance.yaml')))
    assert contract.dump(contract.parse(flat)) == EXPECTED


def test_the_reference_copy_gives_the_same_dump():
    flat = REF.load_params(str(h.pinned('eef_conformance.yaml')))
    assert REF.dump(REF.parse(flat)) == EXPECTED


def test_both_loaders_read_the_same_flat_block():
    path = str(h.pinned('eef_conformance.yaml'))
    assert contract.load_params(path) == REF.load_params(path) == h.fixture_flat()


def test_a_legacy_parameter_set_is_no_contract():
    flat = {'gripper_action': '/gripper_controller/gripper_cmd', 'gripper_cmd_topic': '/x',
            'bt_delay': 8}
    assert contract.parse(flat) == REF.parse(flat) == {}
    assert contract.dump({}) == ''


# ------------------------------------------------------------------------------------------------
# 3: the mutation table and the sweep
# ------------------------------------------------------------------------------------------------

C = 'eef.composite_tool.'
A = C + 'actuators.'
S = C + 'sites.'
V = C + 'verification.'


def _set(key, value):
    return lambda f: f.__setitem__(key, value)


def _del(key):
    return lambda f: f.pop(key)


def _all(*ops):
    def run(f):
        for op in ops:
            op(f)
    return run


# Keys the fixture states at their default value: dropping them changes nothing.
DEFAULTED = [C + 'command_timeout_ms', A + 'linear_axis.goal_time_tolerance_s',
             A + 'rotary_axis.goal_time_tolerance_s', A + 'wrist_mechanism.goal_time_tolerance_s',
             A + 'binary_lock.confirm_timeout_ms',
             'eef.suction_simple.verification.accepted_evidence'] + [
    f'eef.{d}.verification.{k}' for d in ('composite_tool', 'suction_simple', 'jaw_simple', 'jaw_binary')
    for k in ('timeout_ms', 'max_age_ms', 'clock_tolerance_ms')]

ALL_IDS = ['composite_tool', 'suction_simple', 'jaw_simple', 'jaw_binary']

MUTATIONS = [
    # the device list and the legacy keys
    ('ids-missing', _del('eef.ids'), 'eef.ids'),
    ('ids-empty', _set('eef.ids', []), 'eef.ids'),
    ('ids-not-strings', _set('eef.ids', [1, 2]), 'eef.ids'),
    ('ids-scalar', _set('eef.ids', 'composite_tool'), 'eef.ids'),
    ('legacy-gripper-action', _set('gripper_action', 'gripper_controller/gripper_cmd'), 'gripper_action'),
    ('legacy-gripper-cmd-topic', _set('gripper_cmd_topic', '/isaac_gripper_cmd'), 'gripper_cmd_topic'),
    ('legacy-close-position', _set('gripper_close_position', 1.0), 'gripper_close_position'),
    ('legacy-open-position', _set('gripper_open_position', 0.0), 'gripper_open_position'),
    ('ids-duplicate', _set('eef.ids', ALL_IDS + ['jaw_simple']), 'eef.ids'),
    ('id-eef-is-reserved', _set('eef.ids', ['composite_tool', 'eef']), 'eef.ids'),
    ('id-not-a-token', _set('eef.ids', ALL_IDS[:3] + ['9jaw']), 'eef.ids'),
    # the device
    ('actuator-ids-missing', _del(C + 'actuator_ids'), C + 'actuator_ids'),
    ('site-ids-missing', _del(C + 'site_ids'), C + 'site_ids'),
    ('actuator-ids-duplicate', _set(C + 'actuator_ids', ['linear_axis', 'linear_axis']), C + 'actuator_ids'),
    ('actuator-id-not-a-token', _set(C + 'actuator_ids', ['linear-axis']), C + 'actuator_ids'),
    ('site-ids-empty', _set(C + 'site_ids', []), C + 'site_ids'),
    ('actuator-and-site-share-a-token', _set(C + 'site_ids', ['site_a', 'gripper_b']), 'eef.composite_tool'),
    ('command-timeout-zero', _set(C + 'command_timeout_ms', 0), C + 'command_timeout_ms'),
    ('command-timeout-double', _set(C + 'command_timeout_ms', 10000.0), C + 'command_timeout_ms'),
    ('command-timeout-bool', _set(C + 'command_timeout_ms', True), C + 'command_timeout_ms'),
    ('scene-signal-missing', _del(C + 'scene_signal'), C + 'scene_signal'),
    ('scene-signal-unknown', _set(C + 'scene_signal', 'stage'), C + 'scene_signal'),
    ('scene-signal-on-two-sites', _set(C + 'scene_signal', 'adapter'), C + 'scene_signal'),
    # actuators
    ('interface-missing', _del(A + 'linear_axis.command_interface'), A + 'linear_axis.command_interface'),
    ('interface-unknown', _set(A + 'linear_axis.command_interface', 'joint_trajectory'),
     A + 'linear_axis.command_interface'),
    ('joints-duplicate', _set(A + 'wrist_mechanism.joints', ['robot_a_wrist_pitch_joint'] * 2),
     A + 'wrist_mechanism.joints'),
    ('joints-shared', _set(A + 'gripper_b.joints', ['robot_a_jaw_a_joint']), A + 'gripper_b.joints'),
    ('joints-empty', _set(A + 'gripper_b.joints', []), A + 'gripper_b.joints'),
    ('set-bool-positions-without-joints', _del(A + 'gripper_b.joints'), A + 'gripper_b.joint_positions'),
    ('set-bool-joints-without-positions', _all(_del(A + 'gripper_b.joint_positions.engaged'),
                                               _del(A + 'gripper_b.joint_positions.released')),
     A + 'gripper_b.joint_positions'),
    ('set-bool-one-position', _del(A + 'gripper_b.joint_positions.released'), A + 'gripper_b.joint_positions'),
    ('set-bool-position-length', _set(A + 'gripper_b.joint_positions.engaged', [0.0, 0.01]),
     A + 'gripper_b.joint_positions'),
    ('set-bool-position-string', _set(A + 'gripper_b.joint_positions.engaged', ['x']),
     A + 'gripper_b.joint_positions'),
    ('actuation-feedback-string', _set(A + 'binary_lock.actuation_feedback', 'true'),
     A + 'binary_lock.actuation_feedback'),
    ('confirm-timeout-zero', _set(A + 'binary_lock.confirm_timeout_ms', 0), A + 'binary_lock.confirm_timeout_ms'),
    ('action-missing', _del(A + 'gripper_a.action'), A + 'gripper_a.action'),
    ('action-absolute', _set(A + 'gripper_a.action', '/gripper_a_controller/gripper_cmd'), A + 'gripper_a.action'),
    ('action-private', _set(A + 'gripper_a.action', '~/gripper_cmd'), A + 'gripper_a.action'),
    ('action-empty-token', _set(A + 'gripper_a.action', 'gripper_a_controller//gripper_cmd'),
     A + 'gripper_a.action'),
    ('action-shared', _set(A + 'rotary_axis.action', 'linear_axis_controller/follow_joint_trajectory'),
     A + 'rotary_axis.action'),
    ('fjt-joints-missing', _del(A + 'linear_axis.joints'), A + 'linear_axis.joints'),
    ('gc-targets-missing', _all(_del(A + 'gripper_a.targets.open'), _del(A + 'gripper_a.targets.closed')),
     A + 'gripper_a.targets'),
    ('gc-two-joints', _set(A + 'gripper_a.joints', ['robot_a_jaw_a_joint', 'robot_a_extra_joint']),
     A + 'gripper_a.joints'),
    ('gc-no-closed-target', _all(_del(A + 'gripper_a.targets.closed'), _set(A + 'gripper_a.targets.shut', 0.0)),
     A + 'gripper_a.targets'),
    ('gc-target-not-a-number', _set(A + 'gripper_a.targets.open', 'wide'), A + 'gripper_a.targets.open'),
    ('gc-target-a-list', _set(A + 'gripper_a.targets.open', [0.02]), A + 'gripper_a.targets.open'),
    # an adapter tells ENGAGE / RELEASE / TARGET apart by the goal's position: values are distinct
    ('gc-open-equals-closed', _set(A + 'gripper_a.targets.open', 0.0), A + 'gripper_a.targets'),
    ('gc-named-target-equals-closed', _set('eef.jaw_simple.actuators.gripper.targets.pinch', 0.7),
     'eef.jaw_simple.actuators.gripper.targets'),
    ('gc-int-target-equals-a-double', _set(A + 'gripper_a.targets.pinch', 0), A + 'gripper_a.targets'),
    ('gc-negative-zero-equals-zero', _set(A + 'gripper_a.targets.pinch', -0.0), A + 'gripper_a.targets'),
    ('result-policy-missing', _del(A + 'gripper_a.result_policy'), A + 'gripper_a.result_policy'),
    ('result-policy-any-result', _set(A + 'gripper_a.result_policy', 'any_result'), A + 'gripper_a.result_policy'),
    ('max-effort-zero', _set(A + 'gripper_a.max_effort', 0.0), A + 'gripper_a.max_effort'),
    ('max-effort-negative', _set(A + 'gripper_a.max_effort', -1), A + 'gripper_a.max_effort'),
    ('fjt-target-length', _set(A + 'wrist_mechanism.targets.present', [0.3]), A + 'wrist_mechanism.targets.present'),
    ('fjt-target-scalar', _set(A + 'linear_axis.targets.home', 0.0), A + 'linear_axis.targets.home'),
    ('fjt-targets-missing', _all(_del(A + 'linear_axis.targets.home'), _del(A + 'linear_axis.targets.extended')),
     A + 'linear_axis.targets'),
    ('goal-tolerance-missing', _del(A + 'linear_axis.goal_tolerance'), A + 'linear_axis.goal_tolerance'),
    ('goal-tolerance-zero', _set(A + 'linear_axis.goal_tolerance', [0.0]), A + 'linear_axis.goal_tolerance'),
    ('goal-tolerance-length', _set(A + 'wrist_mechanism.goal_tolerance', [0.01]), A + 'wrist_mechanism.goal_tolerance'),
    ('duration-missing', _del(A + 'linear_axis.duration_s'), A + 'linear_axis.duration_s'),
    ('duration-zero', _set(A + 'linear_axis.duration_s', 0), A + 'linear_axis.duration_s'),
    ('goal-time-tolerance-zero', _set(A + 'linear_axis.goal_time_tolerance_s', 0.0),
     A + 'linear_axis.goal_time_tolerance_s'),
    ('path-tolerance-zero', _set(A + 'rotary_axis.path_tolerance', [0.0]), A + 'rotary_axis.path_tolerance'),
    ('path-tolerance-length', _set(A + 'rotary_axis.path_tolerance', [0.2, 0.2]), A + 'rotary_axis.path_tolerance'),
    # sites
    ('grip-missing', _del(S + 'site_a.grip'), S + 'site_a.grip'),
    ('grip-unknown', _set(S + 'site_a.grip', 'gripper_c'), S + 'site_a.grip'),
    ('grip-fjt', _set(S + 'site_a.grip', 'linear_axis'), S + 'site_a.grip'),
    ('one-actuator-grips-two-sites', _all(_set(S + 'site_b.grip', 'gripper_a'), _set(S + 'site_b.engage', 'closed'),
                                          _set(S + 'site_b.release', 'open')), S + 'site_b.grip'),
    ('engage-missing', _del(S + 'site_a.engage'), S + 'site_a.engage'),
    ('release-missing', _del(S + 'site_a.release'), S + 'site_a.release'),
    ('engage-not-a-target-of-the-grip', _set(S + 'site_a.engage', 'engage'), S + 'site_a.engage'),
    ('release-not-a-target-of-the-grip', _set(S + 'site_b.release', 'open'), S + 'site_b.release'),
    ('release-equals-engage', _set(S + 'site_a.release', 'closed'), S + 'site_a.release'),
    # a set_bool grip's polarity is the adapter's: its site engages with 'engage', never swapped
    ('set-bool-site-swaps-engage-and-release', _all(_set(S + 'site_b.engage', 'release'),
                                                    _set(S + 'site_b.release', 'engage')), S + 'site_b.engage'),
    ('suction-site-swaps-engage-and-release', _all(_set('eef.suction_simple.sites.main.engage', 'release'),
                                                   _set('eef.suction_simple.sites.main.release', 'engage')),
     'eef.suction_simple.sites.main.engage'),
    ('set-bool-engage-release-is-refused-at-release-first', _set(S + 'site_b.engage', 'release'), S + 'site_b.release'),
    ('gripper-command-grips-no-site', _all(_set(S + 'site_a.grip', 'binary_lock'), _set(S + 'site_a.engage', 'engage'),
                                           _set(S + 'site_a.release', 'release')), A + 'gripper_a'),
    # verification
    ('require-grasp-missing', _del(V + 'require_grasp_confirmation'), V + 'require_grasp_confirmation'),
    ('require-release-a-string', _set(V + 'require_release_confirmation', 'false'), V + 'require_release_confirmation'),
    ('evidence-command-echo', _set(V + 'accepted_evidence', ['sensor', 'command_echo']), V + 'accepted_evidence'),
    ('evidence-sim-attachment', _set(V + 'accepted_evidence', ['sim_attachment']), V + 'accepted_evidence'),
    ('evidence-duplicate', _set(V + 'accepted_evidence', ['sensor', 'sensor']), V + 'accepted_evidence'),
    ('evidence-empty', _set(V + 'accepted_evidence', []), V + 'accepted_evidence'),
    ('timeout-zero', _set(V + 'timeout_ms', 0), V + 'timeout_ms'),
    ('max-age-a-double', _set(V + 'max_age_ms', 500.0), V + 'max_age_ms'),
    ('clock-tolerance-a-string', _set(V + 'clock_tolerance_ms', '100'), V + 'clock_tolerance_ms'),
    # unknown keys
    ('unknown-actuator-key', _set(A + 'linear_axis.max_velocity', 0.1), A + 'linear_axis.max_velocity'),
    ('unknown-device-key', _set(C + 'color', 'red'), C + 'color'),
    ('undeclared-device', _set('eef.ghost.actuator_ids', ['x']), 'eef.ghost.actuator_ids'),
    ('set-bool-has-no-targets', _set(A + 'gripper_b.targets.open', 0.02), A + 'gripper_b.targets.open'),
    ('gripper-command-has-no-goal-tolerance', _set(A + 'gripper_a.goal_tolerance', [0.001]),
     A + 'gripper_a.goal_tolerance'),
    ('unknown-verification-key', _set(V + 'source', 'sensor'), V + 'source'),
    # accepted, with the fixture's own dump
    ('ok-an-int-where-a-double-is-expected', _all(_set(A + 'gripper_a.max_effort', 20),
                                                  _set(A + 'linear_axis.duration_s', 1)), 'ok'),
    ('ok-an-int-array-where-doubles-are-expected', _set(A + 'gripper_b.joint_positions.engaged', [0]), 'ok'),
    ('ok-a-scalar-goal-tolerance', _set(A + 'linear_axis.goal_tolerance', 0.002), 'ok'),
    ('ok-defaults-dropped', _all(*[_del(k) for k in DEFAULTED]), 'ok'),
    ('ok-no-gripper-action', _del('gripper_action'), 'ok'),
]


LIST_IN_A_NAME = [
    # The first reference copy raised TypeError on these; both parsers now refuse them alike.
    ('list-grip', _set(S + 'site_a.grip', ['gripper_a']), S + 'site_a.grip'),
    ('list-engage', _set(S + 'site_a.engage', ['closed']), S + 'site_a.engage'),
    ('list-release', _set(S + 'site_b.release', ['release']), S + 'site_b.release'),
    # A name token is ASCII in every parser (ROS names are): a non-ASCII letter is refused.
    ('non-ascii-device-id', _set('eef.ids', ALL_IDS[:3] + ['jaw_bïnary']), 'eef.ids'),
    ('non-ascii-actuator-id', _set(C + 'actuator_ids', ['linéar_axis']), C + 'actuator_ids'),
]


@pytest.mark.parametrize('name,mutate,expected', MUTATIONS + LIST_IN_A_NAME,
                         ids=[m[0] for m in MUTATIONS + LIST_IN_A_NAME])
def test_mutation_gives_the_reference_outcome(name, mutate, expected):
    flat = h.fixture_flat()
    mutate(flat)
    ours = outcome(contract, flat)
    reference = outcome(REF, flat)
    assert same_outcome(ours, reference), (ours, reference)
    if expected == 'ok':
        assert ours == ('ok', EXPECTED)
    else:
        assert ours[:2] == ('refused', expected), ours


def test_a_distinct_third_gripper_target_is_accepted_by_both():
    flat = h.fixture_flat()
    flat[A + 'gripper_a.targets.pinch'] = 0.01
    ours, reference = outcome(contract, flat), outcome(REF, flat)
    assert same_outcome(ours, reference) and ours[0] == 'ok', (ours, reference)
    assert 'eef.composite_tool.actuators.gripper_a.targets.pinch = 0.01\n' in ours[1]


SWEEP_VALUES = [True, False, 0, 1, -1, 2.5, -2.5, 'x', '', 'composite_tool', ['x'], [], [1], [1.5],
                [True], [-1.0], ['sensor', 'sensor'], ['open', 'closed']]


def test_sweep_every_key_gives_the_reference_outcome():
    """Every leaf key deleted, given every value type, and given an unknown sibling."""
    base = h.fixture_flat()
    cases, differences, refused, accepted, crashes = 0, [], set(), 0, []
    for key in sorted(base):
        variants = [('delete', _del(key)), ('sibling', _set(key + '_x', 1))]
        variants += [(f'value {v!r}', _set(key, v)) for v in SWEEP_VALUES]
        if isinstance(base[key], list) and base[key]:
            variants.append(('drop last', _set(key, base[key][:-1])))
            variants.append(('repeat first', _set(key, [base[key][0]] + base[key])))
        for what, mutate in variants:
            flat = copy.deepcopy(base)
            mutate(flat)
            ours, reference = outcome(contract, flat), outcome(REF, flat)
            cases += 1
            if 'crash' in (ours[0], reference[0]):
                crashes.append((key, what, ours, reference))
            if not same_outcome(ours, reference):
                differences.append((key, what, ours, reference))
            if ours[0] == 'refused':
                refused.add(ours[1])
            else:
                accepted += 1
    assert not differences, differences[:5]
    # The sweep must reach far: a refusal under (nearly) every key, and some accepted variants.
    assert cases > 2000
    assert len(refused) > 80 and accepted > 20, (len(refused), accepted)
    # Neither parser ever crashes: every input has an outcome.
    assert not crashes, crashes[:5]


# ------------------------------------------------------------------------------------------------
# 4: the rclpy paths
# ------------------------------------------------------------------------------------------------

def test_rclpy_parameter_overrides_give_the_same_dump(harness):
    node = harness.helper('contract_reader', allow_undeclared_parameters=True,
                          automatically_declare_parameters_from_overrides=True,
                          parameter_overrides=harness.overrides(h.fixture_flat()))
    flat = contract.params_from_node(node)
    assert contract.dump(contract.parse(flat)) == EXPECTED


def test_a_params_file_gives_the_same_dump(harness):
    node = harness.helper('contract_file_reader', allow_undeclared_parameters=True,
                          automatically_declare_parameters_from_overrides=True,
                          cli_args=['--ros-args', '--params-file', str(h.pinned('eef_conformance.yaml'))])
    flat = contract.params_from_node(node)
    assert flat['gripper_action'] == ''
    assert contract.dump(contract.parse(flat)) == EXPECTED


# ------------------------------------------------------------------------------------------------
# 5: an adapter's own device
# ------------------------------------------------------------------------------------------------

def test_an_adapter_reads_the_same_block_for_its_own_device():
    flat = h.fixture_flat()
    whole = contract.device_for_adapter(flat, 'composite_tool')
    own = contract.device_for_adapter(contract.device_params(flat, 'composite_tool'), 'composite_tool')
    assert whole == own
    assert own.actuator_ids == ('linear_axis', 'rotary_axis', 'wrist_mechanism', 'gripper_a',
                                'gripper_b', 'binary_lock')
    assert [own.actuators[a].command_interface for a in own.actuator_ids] == [
        'follow_joint_trajectory'] * 3 + ['gripper_command', 'set_bool', 'set_bool']
    assert own.site_ids == ('site_a', 'site_b')
    assert [own.sites[s].grip for s in own.site_ids] == ['gripper_a', 'gripper_b']
    assert own.actuators['wrist_mechanism'].targets == {'present': (0.3, -0.3), 'stow': (0.0, 0.0)}
    assert own.actuators['gripper_a'].max_effort == 20.0
    assert own.actuators['rotary_axis'].path_tolerance == (0.2,)
    assert own.actuators['linear_axis'].path_tolerance is None
    assert own.verification.accepted_evidence == ('sensor',)
    assert own.default_site is None
    simple = contract.device_for_adapter(contract.device_params(flat, 'suction_simple'), 'suction_simple')
    assert simple.default_site.grip == 'vacuum'
    assert simple.site_gripped_by('vacuum').id == 'main'


@pytest.mark.parametrize('device_id,params,key', [
    (None, None, 'device_id'),
    ('', None, 'device_id'),
    ('eef', None, 'device_id'),
    ('bad-id', None, 'device_id'),
    ('ghost', 'whole', 'device_id'),
    ('composite_tool', 'foreign', 'eef.suction_simple.actuator_ids'),
    ('composite_tool', 'broken', 'eef.composite_tool.scene_signal'),
])
def test_an_adapter_refuses_what_is_not_its_device(device_id, params, key):
    flat = h.fixture_flat()
    if params is None or params == 'whole':
        block = flat
    elif params == 'foreign':     # its own keys plus another device's, without eef.ids
        block = {**contract.device_params(flat, 'composite_tool'),
                 **contract.device_params(flat, 'suction_simple')}
    else:
        block = contract.device_params(flat, 'composite_tool')
        block[C + 'scene_signal'] = 'adapter'
    with pytest.raises(contract.Refused) as e:
        contract.device_for_adapter(block, device_id)
    assert e.value.key == key


def test_identity_comes_from_the_namespace():
    assert contract.identity('/r1', 'composite_tool') == '/r1/eef/composite_tool'
    assert contract.identity('/', 'main') == '/eef/main'
    assert contract.identity('', 'main') == '/eef/main'
    assert contract.identity('/cell/r2', 'main') == '/cell/r2/eef/main'


# ------------------------------------------------------------------------------------------------
# determinism: the dump and a refusal are byte-identical in separate processes (any hash seed)
# ------------------------------------------------------------------------------------------------

_DUMP_SCRIPT = '''
import sys
sys.path.insert(0, sys.argv[1])
from trainit_hardware_layer.eef import contract
flat = contract.load_params(sys.argv[2])
sys.stdout.write(contract.dump(contract.parse(flat)))
flat["eef.composite_tool.zz_unknown"] = 1
flat["eef.composite_tool.aa_unknown"] = 1
try:
    contract.parse(flat)
except contract.Refused as e:
    sys.stdout.write("REFUSED " + str(e) + "\\n")
'''


def test_the_dump_is_byte_identical_in_separate_processes(tmp_path):
    import os
    import subprocess
    import sys
    from pathlib import Path
    root = str(Path(__file__).resolve().parent.parent)
    outputs = []
    for seed in ('1', '2', '12345'):
        env = dict(os.environ, PYTHONHASHSEED=seed)
        done = subprocess.run([sys.executable, '-I', '-c', _DUMP_SCRIPT, root,
                               str(h.pinned('eef_conformance.yaml'))],
                              env=env, capture_output=True, check=True)
        outputs.append(done.stdout)
    assert outputs[0] == outputs[1] == outputs[2]
    assert outputs[0] == (EXPECTED + 'REFUSED eef.composite_tool.aa_unknown: unknown key\n').encode('utf-8')
