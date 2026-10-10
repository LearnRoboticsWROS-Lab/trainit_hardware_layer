"""The ADR-0022 ``eef`` parameter block, as an adapter reads it (vNext M2.6).

This is the hardware layer's port of the reference normaliser of the vNext M2.6 forensics
(``normalize_eef.py``). It must stay BEHAVIOURALLY IDENTICAL to it: accept exactly what it
accepts, refuse exactly what it refuses with the same refusal key (and the same reason text), and
produce the same normalised dump, byte for byte. TMR's C++ parser is held to the same rule, so
the runner and the adapter of one device can never read one block two ways. The proof is the
conformance test (``test/test_eef_contract.py``): the pinned fixture's dump equals the pinned
expected file, and a mutation table plus a systematic sweep give the same outcome here and in the
pinned byte copy of the reference. No code here imports anything from another repository.
There is no exception to that rule: a list where ``sites.<s>.grip``, ``.engage`` or ``.release``
expects a name is refused by both (the reference's first copy raised TypeError there; fixed at
the source), and a name token is ASCII in both.

The block (ADR-0022 D6 emission; one entry per device id):

- ``eef.ids`` [string];
- per device ``<id>``: ``actuator_ids``, ``site_ids``, ``command_timeout_ms`` (10000),
  ``scene_signal`` (adapter | tmr | none, required);
- ``actuators.<a>.*`` by ``command_interface``:

  - ``set_bool``: ``joints``? with ``joint_positions.engaged/released``? (both or neither),
    ``actuation_feedback`` (false), ``confirm_timeout_ms`` (2000);
  - ``gripper_command``: ``action``, ``joints`` (exactly one), ``targets.<name>`` (open and closed
    required; pairwise distinct values, since an adapter tells ENGAGE, RELEASE and TARGET apart by
    a goal's position), ``result_policy`` (succeeded | succeeded_or_stalled), ``max_effort``? (> 0);
  - ``follow_joint_trajectory``: ``action``, ``joints``, ``targets.<name>`` [one per joint],
    ``goal_tolerance`` (> 0, a scalar or one per joint), ``duration_s`` (> 0),
    ``goal_time_tolerance_s`` (0.5), ``path_tolerance``? (> 0);

- ``sites.<s>.grip`` / ``.engage`` / ``.release`` (a ``set_bool`` grip engages with ``engage``: its
  polarity is the adapter's, never the block's);
- ``verification.require_grasp_confirmation`` / ``require_release_confirmation`` (required),
  ``accepted_evidence`` ([sensor, sim_ground_truth]), ``timeout_ms`` (3000), ``max_age_ms`` (500),
  ``clock_tolerance_ms`` (100).

On top of the normalised flat form this module builds a typed, read-only view (``Device``,
``Actuator``, ``Site``, ``Verification``) for the adapters. The view adds no rule: everything it
holds was validated by ``parse``.
"""
from __future__ import annotations

import array
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Tuple, Union

import yaml

INTERFACES = ('set_bool', 'gripper_command', 'follow_joint_trajectory')
RESULT_POLICIES = ('succeeded', 'succeeded_or_stalled')
SCENE_SIGNALS = ('adapter', 'tmr', 'none')
ACCEPTED_EVIDENCE = ('sensor', 'sim_ground_truth', 'actuation_inference')
LEGACY_KEYS = ('gripper_cmd_topic', 'gripper_close_position', 'gripper_open_position')


class Refused(Exception):
    """A malformed contract: the runner exits with code 2 (ADR-0022 D5), and so does an adapter."""

    def __init__(self, key: str, why: str):
        super().__init__(f'{key}: {why}')
        self.key = key
        self.why = why


# ------------------------------------------------------------------------------------------------
# The parser: a port of normalize_eef.py, kept line for line in its checks and their order.
# ------------------------------------------------------------------------------------------------

def flatten(tree: Mapping[str, Any], prefix: str = '') -> Dict[str, Any]:
    """{'a': {'b': 1}} -> {'a.b': 1}: the flat form a node sees (launch_ros flattens the same way)."""
    out = {}
    for k, v in tree.items():
        name = f'{prefix}.{k}' if prefix else str(k)
        if isinstance(v, dict):
            out.update(flatten(v, name))
        else:
            out[name] = v
    return out


def load_params(path: str) -> Dict[str, Any]:
    """The flat parameter map of the first node entry of a ROS 2 parameter file.

    (The reference exits on a file with no ``ros__parameters``; a library raises ValueError.)
    """
    with open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    for node in doc.values():
        if isinstance(node, dict) and 'ros__parameters' in node:
            return flatten(node['ros__parameters'])
    raise ValueError(f'{path}: no ros__parameters')


def _is_token(s: Any) -> bool:
    # a ROS name token: [A-Za-z][A-Za-z0-9_]*, ASCII only ; 'eef' is reserved as an id inside the block
    return (isinstance(s, str) and s.isascii() and s[:1].isalpha()
            and all(c.isalnum() or c == '_' for c in s))


def _relative_action(s: Any) -> bool:
    # a relative ROS name: tokens separated by '/', no leading '/' or '~'
    return isinstance(s, str) and s and not s.startswith(('/', '~')) and all(
        _is_token(t) for t in s.split('/'))


def _num(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _nums(v: Any) -> bool:
    return isinstance(v, list) and len(v) > 0 and all(_num(x) for x in v)


def _strs(v: Any) -> bool:
    return isinstance(v, list) and len(v) > 0 and all(isinstance(x, str) for x in v)


def is_token(s: Any) -> bool:
    """True for a ROS name token as the block uses it (the ids of devices, actuators, sites)."""
    return _is_token(s)


def parse(params: Mapping[str, Any]) -> Dict[str, Any]:
    """Return the normalised contract {key: value} for every device, or raise Refused.

    `params` is the flat {dotted: value} map. Returns {} when there is no `eef.*` key (LEGACY).
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
    out: Dict[str, Any] = {'ids': list(ids)}
    known = {'ids'}
    for dev in ids:
        if not _is_token(dev) or dev == 'eef':
            raise Refused('eef.ids', f'bad device id {dev!r}')
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
        if not _strs(act_ids) or len(set(act_ids)) != len(act_ids) or not all(
                _is_token(a) for a in act_ids):
            raise Refused(f'eef.{dev}.actuator_ids', 'non-empty, unique ROS name tokens required')
        if not _strs(site_ids) or len(set(site_ids)) != len(site_ids) or not all(
                _is_token(s) for s in site_ids):
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
        actions: Dict[str, str] = {}
        joints_seen: Dict[str, str] = {}
        interfaces: Dict[str, str] = {}
        targets_of: Dict[str, set] = {}
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
                    raise Refused(f'eef.{dev}.{q}joint_positions',
                                  'joints and both joint_positions, or none')
                if joints is not None:
                    if not _nums(jp_e) or not _nums(jp_r) or len(jp_e) != len(joints) or len(
                            jp_r) != len(joints):
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
        grips: Dict[str, str] = {}
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
                raise Refused(f'eef.{dev}.actuators.{a}',
                              'a gripper_command actuator must grip a site')
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


def fmt(v: Any) -> str:
    """One dump value: strings raw, bools true/false, ints decimal, doubles '%.6g', arrays joined."""
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return '%.6g' % v
    if isinstance(v, list):
        return ','.join(fmt(x) for x in v)
    return str(v)


def dump(contract: Mapping[str, Any]) -> str:
    """The normalised dump: one '<dotted key> = <value>' line per leaf, sorted."""
    return ''.join(f'eef.{k} = {fmt(contract[k])}\n' for k in sorted(contract))


# ------------------------------------------------------------------------------------------------
# Reading the block from a node's parameters
# ------------------------------------------------------------------------------------------------

def plain(value: Any) -> Any:
    """A parameter value as the parser expects it: arrays become lists, scalars stay as they are.

    rclpy hands arrays over as lists, tuples or ``array.array`` depending on where they came from
    (a parameter file, an override, a parameter service); the parser only accepts lists.
    """
    if isinstance(value, (array.array, tuple)):
        return list(value)
    return value


def params_from_node(node: Any) -> Dict[str, Any]:
    """The flat map ``parse`` reads, from a node declared with automatic parameter overrides.

    It holds every ``eef.*`` parameter, plus the legacy keys the parser refuses next to a block
    (so an adapter refuses the same mixed block the runner refuses).
    """
    out = {}
    for name, param in node.get_parameters_by_prefix('eef').items():
        out[f'eef.{name}'] = plain(param.value)
    for key in ('gripper_action',) + LEGACY_KEYS:
        if node.has_parameter(key):
            out[key] = plain(node.get_parameter(key).value)
    return out


# ------------------------------------------------------------------------------------------------
# The typed view an adapter works with
# ------------------------------------------------------------------------------------------------

Target = Union[float, Tuple[float, ...]]


@dataclass(frozen=True)
class Actuator:
    """One commandable part of a device, as the contract declares it."""

    id: str
    command_interface: str
    joints: Tuple[str, ...] = ()
    # set_bool
    joint_positions_engaged: Tuple[float, ...] = ()
    joint_positions_released: Tuple[float, ...] = ()
    actuation_feedback: bool = False
    confirm_timeout_ms: int = 2000
    # gripper_command and follow_joint_trajectory
    action: str = ''
    targets: Mapping[str, Target] = field(default_factory=dict)
    # gripper_command
    result_policy: str = ''
    max_effort: Optional[float] = None
    # follow_joint_trajectory
    goal_tolerance: Tuple[float, ...] = ()
    duration_s: float = 0.0
    goal_time_tolerance_s: float = 0.0
    path_tolerance: Optional[Tuple[float, ...]] = None

    @property
    def target_names(self) -> Tuple[str, ...]:
        if self.command_interface == 'set_bool':
            return ('engage', 'release')
        return tuple(sorted(self.targets))


@dataclass(frozen=True)
class Site:
    """Where the device holds an object: its grip actuator and that actuator's two targets."""

    id: str
    grip: str
    engage: str
    release: str


@dataclass(frozen=True)
class Verification:
    require_grasp_confirmation: bool
    require_release_confirmation: bool
    accepted_evidence: Tuple[str, ...]
    timeout_ms: int
    max_age_ms: int
    clock_tolerance_ms: int


@dataclass(frozen=True)
class Device:
    """One end-effector device of the block, validated by ``parse``."""

    id: str
    actuator_ids: Tuple[str, ...]
    site_ids: Tuple[str, ...]
    command_timeout_ms: int
    scene_signal: str
    actuators: Mapping[str, Actuator]
    sites: Mapping[str, Site]
    verification: Verification

    def site_gripped_by(self, actuator_id: str) -> Optional[Site]:
        """The site whose grip actuator this is, or None (an actuator grips at most one site)."""
        for s in self.site_ids:
            if self.sites[s].grip == actuator_id:
                return self.sites[s]
        return None

    @property
    def default_site(self) -> Optional[Site]:
        """The only site of a single-site device (the compatibility endpoints map onto it)."""
        return self.sites[self.site_ids[0]] if len(self.site_ids) == 1 else None


def _device(normalized: Mapping[str, Any], dev: str) -> Device:
    g = normalized.get
    actuators = {}
    for a in normalized[f'{dev}.actuator_ids']:
        q = f'{dev}.actuators.{a}.'
        iface = normalized[q + 'command_interface']
        targets = {k[len(q + 'targets.'):]: (tuple(v) if isinstance(v, list) else v)
                   for k, v in normalized.items() if k.startswith(q + 'targets.')}
        pt = g(q + 'path_tolerance')
        actuators[a] = Actuator(
            id=a,
            command_interface=iface,
            joints=tuple(g(q + 'joints', ())),
            joint_positions_engaged=tuple(g(q + 'joint_positions.engaged', ())),
            joint_positions_released=tuple(g(q + 'joint_positions.released', ())),
            actuation_feedback=bool(g(q + 'actuation_feedback', False)),
            confirm_timeout_ms=int(g(q + 'confirm_timeout_ms', 2000)),
            action=g(q + 'action', ''),
            targets=targets,
            result_policy=g(q + 'result_policy', ''),
            max_effort=g(q + 'max_effort'),
            goal_tolerance=tuple(g(q + 'goal_tolerance', ())),
            duration_s=float(g(q + 'duration_s', 0.0)),
            goal_time_tolerance_s=float(g(q + 'goal_time_tolerance_s', 0.0)),
            path_tolerance=tuple(pt) if pt is not None else None,
        )
    sites = {}
    for s in normalized[f'{dev}.site_ids']:
        q = f'{dev}.sites.{s}.'
        sites[s] = Site(id=s, grip=normalized[q + 'grip'], engage=normalized[q + 'engage'],
                        release=normalized[q + 'release'])
    v = f'{dev}.verification.'
    verification = Verification(
        require_grasp_confirmation=normalized[v + 'require_grasp_confirmation'],
        require_release_confirmation=normalized[v + 'require_release_confirmation'],
        accepted_evidence=tuple(normalized[v + 'accepted_evidence']),
        timeout_ms=normalized[v + 'timeout_ms'],
        max_age_ms=normalized[v + 'max_age_ms'],
        clock_tolerance_ms=normalized[v + 'clock_tolerance_ms'],
    )
    return Device(
        id=dev,
        actuator_ids=tuple(normalized[f'{dev}.actuator_ids']),
        site_ids=tuple(normalized[f'{dev}.site_ids']),
        command_timeout_ms=normalized[f'{dev}.command_timeout_ms'],
        scene_signal=normalized[f'{dev}.scene_signal'],
        actuators=actuators,
        sites=sites,
        verification=verification,
    )


def devices(normalized: Mapping[str, Any]) -> Dict[str, Device]:
    """The typed devices of a normalised contract, by id, in ``eef.ids`` order."""
    return {dev: _device(normalized, dev) for dev in normalized.get('ids', [])}


def device_for_adapter(params: Mapping[str, Any], device_id: Any) -> Device:
    """The device an adapter serves, read from the SAME block TMR reads (ADR-0022 D8).

    An adapter is started with ``device_id: <id>`` plus the ``eef.<id>.*`` keys of its own device
    (launch_ros flattens the nested dict). It may also be handed the whole block, ``eef.ids``
    included; then every device is validated, exactly as the runner validates them. Without
    ``eef.ids`` the block is read as ``eef.ids: [<device_id>]``, so a key of any other device is an
    unknown key and is refused.
    """
    if device_id is None or device_id == '':
        raise Refused('device_id', 'required: the id of the device this adapter serves')
    if not _is_token(device_id) or device_id == 'eef':
        raise Refused('device_id', f'bad device id {device_id!r}')
    block = dict(params)
    if 'eef.ids' not in block:
        block['eef.ids'] = [device_id]
    normalized = parse(block)
    if device_id not in normalized['ids']:
        raise Refused('device_id', f'{device_id!r} is not in eef.ids {normalized["ids"]}')
    return _device(normalized, device_id)


def device_params(flat: Mapping[str, Any], device_id: str) -> Dict[str, Any]:
    """The raw ``eef.<id>.*`` keys of one device of a flat block: what a launch file hands an adapter."""
    pre = f'eef.{device_id}.'
    return {k: v for k, v in flat.items() if k.startswith(pre)}


def identity(node_namespace: str, device_id: str) -> str:
    """The fully-qualified device identity ``<robot namespace>/eef/<id>`` (``/eef/<id>`` at the root).

    Computed from the adapter's own node namespace, never configured (ADR-0022 D3, D7).
    """
    ns = (node_namespace or '/').rstrip('/')
    return f'{ns}/eef/{device_id}'
