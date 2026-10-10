#!/usr/bin/env python3
"""Reference normaliser of the ADR-0022 `eef` parameter block (vNext M2.6).

This is the SPECIFICATION in executable form. TMR's C++ parser and the hardware layer's Python
parser must accept exactly what this accepts, refuse exactly what it refuses (with the same
refusal key), and produce the same normalised dump, byte for byte, for the conformance fixture.

Input: a ROS 2 parameter file (the `/**: ros__parameters:` form) or the flattened parameter
dictionary {dotted_name: value} a node sees. Output: `dump()` -- one line per leaf, sorted, as
`<dotted key> = <value>`; strings raw, bools true/false, ints decimal, doubles '%.6g', arrays
comma-joined. Defaults are filled in; nothing optional and absent is printed.

Usage: normalize_eef.py <params.yaml>   prints the dump, or 'REFUSED <key>: <why>' and exits 2.
"""
import sys

import yaml

INTERFACES = ('set_bool', 'gripper_command', 'follow_joint_trajectory')
RESULT_POLICIES = ('succeeded', 'succeeded_or_stalled')
SCENE_SIGNALS = ('adapter', 'tmr', 'none')
ACCEPTED_EVIDENCE = ('sensor', 'sim_ground_truth', 'actuation_inference')
LEGACY_KEYS = ('gripper_cmd_topic', 'gripper_close_position', 'gripper_open_position')


class Refused(Exception):
    """A malformed contract: the runner exits with code 2 (ADR-0022 D5)."""

    def __init__(self, key, why):
        super().__init__(f'{key}: {why}')
        self.key = key


def flatten(tree, prefix=''):
    out = {}
    for k, v in tree.items():
        name = f'{prefix}.{k}' if prefix else str(k)
        if isinstance(v, dict):
            out.update(flatten(v, name))
        else:
            out[name] = v
    return out


def load_params(path):
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    for node in doc.values():
        if isinstance(node, dict) and 'ros__parameters' in node:
            return flatten(node['ros__parameters'])
    raise SystemExit(f'{path}: no ros__parameters')


def _is_token(s):
    # a ROS name token: [A-Za-z][A-Za-z0-9_]*, ASCII only ; 'eef' is reserved as an id inside the block
    return (isinstance(s, str) and s.isascii() and s[:1].isalpha()
            and all(c.isalnum() or c == '_' for c in s))


def _relative_action(s):
    # a relative ROS name: tokens separated by '/', no leading '/' or '~'
    return isinstance(s, str) and s and not s.startswith(('/', '~')) and all(
        _is_token(t) for t in s.split('/'))


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _nums(v):
    return isinstance(v, list) and len(v) > 0 and all(_num(x) for x in v)


def _strs(v):
    return isinstance(v, list) and len(v) > 0 and all(isinstance(x, str) for x in v)


def parse(params):
    """Return the normalised contract {key: value} for every device, or raise Refused.

    `params` is the flat {dotted: value} map. Returns {} when there is no `eef.ids` (LEGACY).
    """
    eef = {k[4:]: v for k, v in params.items() if k.startswith('eef.')}
    if not eef:
        return {}
    ids = eef.get('ids')
    if not _strs(ids):
        raise Refused('eef.ids', 'a non-empty string array is required')
    # legacy and canonical keys mixed
    if params.get('gripper_action', ''):
        raise Refused('gripper_action', 'a legacy key next to an eef block')
    for k in LEGACY_KEYS:
        if k in params:
            raise Refused(k, 'a legacy key next to an eef block')
    if len(set(ids)) != len(ids):
        raise Refused('eef.ids', 'duplicate device id')
    out = {'ids': list(ids)}
    known = {'ids'}
    for dev in ids:
        if not _is_token(dev) or dev == 'eef':
            raise Refused(f'eef.ids', f'bad device id {dev!r}')
        p = f'{dev}.'

        def get(key, default=None, required=False, _p=p):
            full = _p + key
            if full in eef:
                known.add(full)
                return eef[full]
            if required:
                raise Refused('eef.' + full, 'required')
            return default

        act_ids = get('actuator_ids', required=True)
        site_ids = get('site_ids', required=True)
        if not _strs(act_ids) or len(set(act_ids)) != len(act_ids) or not all(_is_token(a) for a in act_ids):
            raise Refused(f'eef.{dev}.actuator_ids', 'non-empty, unique ROS name tokens required')
        if not _strs(site_ids) or len(set(site_ids)) != len(site_ids) or not all(_is_token(s) for s in site_ids):
            raise Refused(f'eef.{dev}.site_ids', 'non-empty, unique ROS name tokens required')
        if set(act_ids) & set(site_ids):
            raise Refused(f'eef.{dev}', 'one token used as both an actuator and a site id')
        cto = get('command_timeout_ms', 10000)
        if not isinstance(cto, int) or isinstance(cto, bool) or cto <= 0:
            raise Refused(f'eef.{dev}.command_timeout_ms', 'a positive integer')
        scene = get('scene_signal', required=True)
        if scene not in SCENE_SIGNALS:
            raise Refused(f'eef.{dev}.scene_signal', f'one of {SCENE_SIGNALS}')
        if scene != 'none' and len(site_ids) != 1:
            raise Refused(f'eef.{dev}.scene_signal', 'only a single-site device has a scene signal')
        out[f'{dev}.actuator_ids'] = list(act_ids)
        out[f'{dev}.site_ids'] = list(site_ids)
        out[f'{dev}.command_timeout_ms'] = cto
        out[f'{dev}.scene_signal'] = scene
        actions = {}
        joints_seen = {}
        interfaces = {}
        targets_of = {}
        for a in act_ids:
            q = f'actuators.{a}.'
            iface = get(q + 'command_interface', required=True)
            if iface not in INTERFACES:
                raise Refused(f'eef.{dev}.{q}command_interface', f'one of {INTERFACES}')
            interfaces[a] = iface
            out[f'{dev}.{q}command_interface'] = iface
            joints = get(q + 'joints')
            if joints is not None:
                if not _strs(joints) or len(set(joints)) != len(joints):
                    raise Refused(f'eef.{dev}.{q}joints', 'a non-empty array of unique names')
                for j in joints:
                    if j in joints_seen:
                        raise Refused(f'eef.{dev}.{q}joints', f'joint {j} shared with {joints_seen[j]}')
                    joints_seen[j] = a
                out[f'{dev}.{q}joints'] = list(joints)
            if iface == 'set_bool':
                jp_e = get(q + 'joint_positions.engaged')
                jp_r = get(q + 'joint_positions.released')
                if (jp_e is None) != (jp_r is None) or (joints is None) != (jp_e is None):
                    raise Refused(f'eef.{dev}.{q}joint_positions', 'joints and both joint_positions, or none')
                if joints is not None:
                    if not _nums(jp_e) or not _nums(jp_r) or len(jp_e) != len(joints) or len(jp_r) != len(joints):
                        raise Refused(f'eef.{dev}.{q}joint_positions', 'one number per joint')
                    out[f'{dev}.{q}joint_positions.engaged'] = [float(x) for x in jp_e]
                    out[f'{dev}.{q}joint_positions.released'] = [float(x) for x in jp_r]
                fb = get(q + 'actuation_feedback', False)
                if not isinstance(fb, bool):
                    raise Refused(f'eef.{dev}.{q}actuation_feedback', 'a bool')
                cto_a = get(q + 'confirm_timeout_ms', 2000)
                if not isinstance(cto_a, int) or isinstance(cto_a, bool) or cto_a <= 0:
                    raise Refused(f'eef.{dev}.{q}confirm_timeout_ms', 'a positive integer')
                out[f'{dev}.{q}actuation_feedback'] = fb
                out[f'{dev}.{q}confirm_timeout_ms'] = cto_a
                targets_of[a] = {'engage', 'release'}
                continue
            action = get(q + 'action', required=True)
            if not _relative_action(action):
                raise Refused(f'eef.{dev}.{q}action', 'a relative ROS name')
            if action in actions:
                raise Refused(f'eef.{dev}.{q}action', f'shared with {actions[action]}')
            actions[action] = a
            out[f'{dev}.{q}action'] = action
            if joints is None:
                raise Refused(f'eef.{dev}.{q}joints', 'required')
            tnames = sorted({k.split('.')[-1] for k in eef if k.startswith(p + q + 'targets.')})
            if not tnames:
                raise Refused(f'eef.{dev}.{q}targets', 'required')
            targets_of[a] = set(tnames)
            if iface == 'gripper_command':
                if len(joints) != 1:
                    raise Refused(f'eef.{dev}.{q}joints', 'exactly one driven joint')
                if 'open' not in tnames or 'closed' not in tnames:
                    raise Refused(f'eef.{dev}.{q}targets', 'open and closed are required')
                for t in tnames:
                    v = get(q + 'targets.' + t)
                    if not _num(v):
                        raise Refused(f'eef.{dev}.{q}targets.{t}', 'a number')
                    out[f'{dev}.{q}targets.{t}'] = float(v)
                # an adapter tells ENGAGE / RELEASE / TARGET apart by the goal's position
                values = [out[f'{dev}.{q}targets.{t}'] for t in tnames]
                if len(set(values)) != len(values):
                    raise Refused(f'eef.{dev}.{q}targets', 'distinct values (a command is told by its position)')
                rp = get(q + 'result_policy', required=True)
                if rp not in RESULT_POLICIES:
                    raise Refused(f'eef.{dev}.{q}result_policy', f'one of {RESULT_POLICIES}')
                out[f'{dev}.{q}result_policy'] = rp
                me = get(q + 'max_effort')
                if me is not None:
                    if not _num(me) or me <= 0:
                        raise Refused(f'eef.{dev}.{q}max_effort', 'a number > 0')
                    out[f'{dev}.{q}max_effort'] = float(me)
                continue
            # follow_joint_trajectory
            n = len(joints)
            for t in tnames:
                v = get(q + 'targets.' + t)
                if not _nums(v) or len(v) != n:
                    raise Refused(f'eef.{dev}.{q}targets.{t}', f'{n} numbers')
                out[f'{dev}.{q}targets.{t}'] = [float(x) for x in v]
            gt = get(q + 'goal_tolerance', required=True)
            gt = [gt] * n if _num(gt) else gt
            if not _nums(gt) or len(gt) != n or any(x <= 0 for x in gt):
                raise Refused(f'eef.{dev}.{q}goal_tolerance', f'{n} numbers > 0 (or one scalar)')
            out[f'{dev}.{q}goal_tolerance'] = [float(x) for x in gt]
            d = get(q + 'duration_s', required=True)
            if not _num(d) or d <= 0:
                raise Refused(f'eef.{dev}.{q}duration_s', 'a number > 0')
            out[f'{dev}.{q}duration_s'] = float(d)
            gtt = get(q + 'goal_time_tolerance_s', 0.5)
            if not _num(gtt) or gtt <= 0:
                raise Refused(f'eef.{dev}.{q}goal_time_tolerance_s', 'a number > 0')
            out[f'{dev}.{q}goal_time_tolerance_s'] = float(gtt)
            pt = get(q + 'path_tolerance')
            if pt is not None:
                pt = [pt] * n if _num(pt) else pt
                if not _nums(pt) or len(pt) != n or any(x <= 0 for x in pt):
                    raise Refused(f'eef.{dev}.{q}path_tolerance', f'{n} numbers > 0 (or one scalar)')
                out[f'{dev}.{q}path_tolerance'] = [float(x) for x in pt]
        grips = {}
        for s in site_ids:
            q = f'sites.{s}.'
            grip = get(q + 'grip', required=True)
            if not isinstance(grip, str) or grip not in interfaces:
                raise Refused(f'eef.{dev}.{q}grip', 'an actuator of this device')
            if interfaces[grip] == 'follow_joint_trajectory':
                raise Refused(f'eef.{dev}.{q}grip', 'a follow_joint_trajectory actuator never grips')
            if grip in grips:
                raise Refused(f'eef.{dev}.{q}grip', f'already grips {grips[grip]}')
            grips[grip] = s
            eng = get(q + 'engage', required=True)
            rel = get(q + 'release', required=True)
            for name, t in (('engage', eng), ('release', rel)):
                if not isinstance(t, str) or t not in targets_of[grip]:
                    raise Refused(f'eef.{dev}.{q}{name}', f'a target of {grip}')
            if eng == rel:
                raise Refused(f'eef.{dev}.{q}release', 'differs from engage')
            if interfaces[grip] == 'set_bool' and eng != 'engage':
                raise Refused(f'eef.{dev}.{q}engage', 'engage for a set_bool grip (its polarity is the adapter\'s)')
            out[f'{dev}.{q}grip'] = grip
            out[f'{dev}.{q}engage'] = eng
            out[f'{dev}.{q}release'] = rel
        for a, iface in interfaces.items():
            if iface == 'gripper_command' and a not in grips:
                raise Refused(f'eef.{dev}.actuators.{a}', 'a gripper_command actuator must grip a site')
        v = 'verification.'
        for key in ('require_grasp_confirmation', 'require_release_confirmation'):
            b = get(v + key, required=True)
            if not isinstance(b, bool):
                raise Refused(f'eef.{dev}.{v}{key}', 'a bool')
            out[f'{dev}.{v}{key}'] = b
        ae = get(v + 'accepted_evidence', ['sensor', 'sim_ground_truth'])
        if not _strs(ae) or any(x not in ACCEPTED_EVIDENCE for x in ae) or len(set(ae)) != len(ae):
            raise Refused(f'eef.{dev}.{v}accepted_evidence', f'unique values of {ACCEPTED_EVIDENCE}')
        out[f'{dev}.{v}accepted_evidence'] = list(ae)
        for key, default in (('timeout_ms', 3000), ('max_age_ms', 500), ('clock_tolerance_ms', 100)):
            x = get(v + key, default)
            if not isinstance(x, int) or isinstance(x, bool) or x <= 0:
                raise Refused(f'eef.{dev}.{v}{key}', 'a positive integer')
            out[f'{dev}.{v}{key}'] = x
    unknown = sorted(set(eef) - known)
    if unknown:
        raise Refused('eef.' + unknown[0], 'unknown key')
    return out


def fmt(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return '%.6g' % v
    if isinstance(v, list):
        return ','.join(fmt(x) for x in v)
    return str(v)


def dump(contract):
    return ''.join(f'eef.{k} = {fmt(contract[k])}\n' for k in sorted(contract))


if __name__ == '__main__':
    try:
        sys.stdout.write(dump(parse(load_params(sys.argv[1]))))
    except Refused as e:
        print(f'REFUSED {e}')
        sys.exit(2)
