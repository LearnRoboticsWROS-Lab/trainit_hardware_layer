"""The real camera backend (D0.4): the source-information document, its discovery helpers, the launch's contract.

No device and no ROS graph: the discovery helpers read a FAKE sysfs / proc tree, the launch's driver parameters and
remaps are pure functions. The live D455 run is the owner-guided D0-C acceptance.
"""
import importlib.util
import json
import xml.dom.minidom
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from trainit_hardware_layer.camera import dds_transport as dds
from trainit_hardware_layer.camera import source_info as si

ROOT = Path(__file__).resolve().parents[1]
PROFILE = yaml.safe_load((ROOT / 'profiles' / 'camera' / 'realsense_d455.yaml').read_text())


@pytest.fixture(autouse=True)
def _hermetic_launch_environment(monkeypatch, tmp_path):
    """The launch writes its generated DDS profile under $ROS_HOME: never the developer's own ~/.ros here."""
    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros_home'))
    for name in (*dds.USER_PROFILE_ENVS, 'RMW_IMPLEMENTATION'):
        monkeypatch.delenv(name, raising=False)


def _launch_module():
    path = ROOT / 'launch' / 'realsense_camera.launch.py'
    spec = importlib.util.spec_from_file_location('realsense_camera_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _doc(**blocks):
    return si.build(backend='real', published_wall_utc_ns=1, **blocks)


# ---- the document ----------------------------------------------------------------------------------------------

def test_a_document_round_trips_and_keeps_only_known_blocks():
    doc = _doc(device={'origin': 'DISCOVERED', 'serial_number': '000000000000'},
               deployment={'origin': 'DECLARED', 'namespace': '/camera'},
               timestamping={'origin': 'OBSERVED', 'clock_domains': {'color': 'global_time', 'depth': 'global_time'},
                             'stamp_origin': 'DRIVER_GLOBAL_TIME'},
               attachment=None)                                   # unknown is ABSENT, never a placeholder
    assert doc['schema'] == 'trainit-camera-source/1.0' and 'attachment' not in doc
    assert si.parse(dict(doc)) == doc


@pytest.mark.parametrize('mutate, needle', [
    (lambda d: d.update(schema='trainit-camera-source/2.0'), 'schema'),
    (lambda d: d.pop('backend'), 'backend'),
    (lambda d: d.update(published_wall_utc_ns=0), 'published_wall_utc_ns'),
    (lambda d: d.update(published_wall_utc_ns=True), 'published_wall_utc_ns'),
    (lambda d: d.update(device={'origin': 'DECLARED'}), 'device.origin'),       # a device fact is DISCOVERED
    (lambda d: d.update(deployment={'origin': 'DISCOVERED'}), 'deployment.origin'),
    (lambda d: d.update(gossip={'origin': 'OBSERVED'}), 'unknown block'),       # a typo is not a new fact
    (lambda d: d.update(device={'origin': 'DISCOVERED', 'x': float('nan')}), 'JSON value'),
    (lambda d: d.update(timestamping={'origin': 'OBSERVED', 'clock_domains': {'color': 'hardware_clock'},
                                      'stamp_origin': 'DRIVER_GLOBAL_TIME'}), 'stamp_origin'),
])
def test_bad_documents_are_refused(mutate, needle):
    doc = _doc()
    mutate(doc)
    with pytest.raises(si.SourceInfoError, match=needle):
        si.parse(doc)


def test_a_newer_minor_version_may_carry_blocks_this_reader_does_not_know():
    doc = dict(_doc(), schema='trainit-camera-source/1.3', network={'origin': 'OBSERVED', 'link': 'x'})
    assert si.parse(doc)['network'] == {'origin': 'OBSERVED', 'link': 'x'}


@pytest.mark.parametrize('domains, origin', [
    ({'color': 'global_time', 'depth': 'global_time'}, 'DRIVER_GLOBAL_TIME'),
    ({'color': 'system_time', 'depth': 'system_time'}, 'RECEIPT'),
    ({'color': 'hardware_clock', 'depth': 'hardware_clock'}, None),       # the driver re-bases it: no claim
    ({'color': 'global_time', 'depth': 'system_time'}, None),             # mixed: no claim
    ({'color': 'MIXED_OR_UNKNOWN'}, None),
    ({}, None),
])
def test_the_stamp_origin_is_claimed_only_when_every_stream_establishes_it(domains, origin):
    assert si.stamp_origin(domains) == origin


# ---- discovery helpers (fake sysfs / proc) -------------------------------------------------------------------------

def _usb_device(directory: Path, **attributes):
    directory.mkdir(parents=True)
    for name, value in attributes.items():
        (directory / name).write_text(f'{value}\n')


def _fake_sysfs(tmp_path, with_hub=True):
    root = tmp_path / 'sys'
    usb = root / 'devices' / 'pci0000:00' / '0000:00:08.3' / 'usb8'
    usb.mkdir(parents=True)
    if with_hub:
        _usb_device(usb / '8-1', idVendor='0bda', idProduct='0411', product='4-Port USB 3.0 Hub', speed='5000',
                    bmAttributes='e0')
        camera = usb / '8-1' / '8-1.4'
    else:
        camera = usb / '8-1'
    _usb_device(camera, idVendor='8086', idProduct='0b5c', busnum='8', speed='5000', version=' 3.20',
                serial='ASIC-SERIAL-NOT-THE-CAMERA')
    video = camera / f'{camera.name}:1.0' / 'video4linux' / 'video2'
    video.mkdir(parents=True)
    port = '/sys/' + str(video.relative_to(root))
    return root, port


def test_the_usb_attachment_behind_a_hub_is_discovered_from_sysfs(tmp_path):
    root, port = _fake_sysfs(tmp_path)
    att = si.usb_attachment(port, sysfs_root=root)
    assert att == {'origin': 'DISCOVERED', 'via': 'sysfs (from the physical port the driver reports)', 'kind': 'USB',
                   'port_id': '8-1.4', 'bus': 8,
                   'speed_mbps': 5000, 'usb_version': '3.20',
                   'hub': {'port_id': '8-1', 'vendor_product': '0bda:0411', 'product': '4-Port USB 3.0 Hub',
                           'speed_mbps': 5000, 'reports_self_powered': True}}
    assert 'serial' not in str(att).lower()       # the USB descriptor's serial is the ASIC's, not the camera's


def test_a_root_port_attachment_says_so(tmp_path):
    root, port = _fake_sysfs(tmp_path, with_hub=False)
    att = si.usb_attachment(port, sysfs_root=root)
    assert att['port_id'] == '8-1' and att['hub'] is None


@pytest.mark.parametrize('port', ['', None, 'usb-0000:00:14.0-1', '/dev/video2', '/sys/devices/nowhere/video9',
                                  '/sys/../etc/8-1', '/sys/devices/../../etc/8-1'])
def test_an_unknown_attachment_is_absent_never_guessed(tmp_path, port):
    root, _ = _fake_sysfs(tmp_path)
    assert si.usb_attachment(port, sysfs_root=root) is None


def test_unreadable_attributes_are_left_out(tmp_path):
    root, port = _fake_sysfs(tmp_path)
    (root / Path(port).relative_to('/sys')).parents[2].joinpath('speed').write_text('fast\n')
    assert 'speed_mbps' not in si.usb_attachment(port, sysfs_root=root)


def test_host_facts_carry_a_boot_hash_and_no_address(tmp_path):
    proc = tmp_path / 'proc'
    (proc / 'sys' / 'kernel' / 'random').mkdir(parents=True)
    (proc / 'sys' / 'kernel' / 'random' / 'boot_id').write_text('6f1c0d3e-0000-4000-8000-000000000000\n')
    facts = si.host_facts(proc_root=proc)
    import hashlib
    assert facts['boot_id_sha256'] == hashlib.sha256(b'6f1c0d3e-0000-4000-8000-000000000000').hexdigest()
    assert facts['origin'] == 'DISCOVERED' and {'hostname', 'machine', 'kernel'} <= set(facts)
    assert not {'ip', 'address', 'machine_id', 'mac'} & set(facts)
    assert 'boot_id_sha256' not in si.host_facts(proc_root=tmp_path / 'absent')      # unknown is absent


def test_the_library_is_read_from_the_running_process_map():
    maps = ('7f00-7f01 r-xp 00000000 08:02 1 /opt/ros/humble/lib/librealsense2_camera.so\n'
            '7f02-7f03 r-xp 00000000 08:02 2 /opt/ros/humble/lib/x86_64-linux-gnu/librealsense2.so.2.58.4\n'
            '7f04-7f05 r--p 00000000 08:02 2 /opt/ros/humble/lib/x86_64-linux-gnu/librealsense2.so.2.58.4\n')
    assert si.library_in_process(maps) == {'name': 'librealsense2', 'version': '2.58.4',
                                           'path': '/opt/ros/humble/lib/x86_64-linux-gnu/librealsense2.so.2.58.4'}
    assert si.library_in_process('7f00-7f01 r-xp 0 0 0 /usr/lib/libc.so.6\n') is None


# ---- the real backend: profile, launch, standalone description ------------------------------------------------------

def test_the_real_backend_declares_its_source_descriptor_and_its_launch():
    real = PROFILE['backends']['real']
    assert real['source_descriptor'] == {'backend': 'real', 'intrinsics_kind': 'DEVICE_REPORTED',
                                         'stamp_origin': 'DRIVER_GLOBAL_TIME',
                                         'registration_basis': 'driver_align_depth'}
    assert real['launch'] == 'realsense_camera.launch.py' and (ROOT / 'launch' / real['launch']).is_file()
    assert 'remote' not in real                    # where it runs is deployment, not backend


def test_the_launch_remaps_only_colour_and_the_aligned_depth_onto_the_contract():
    launch = _launch_module()
    pairs = dict(launch.contract_remappings('/camera', '/camera/driver'))
    assert pairs == {'/camera/driver/color/image_raw': '/camera/color/image_raw',
                     '/camera/driver/color/camera_info': '/camera/color/camera_info',
                     '/camera/driver/aligned_depth_to_color/image_raw': '/camera/depth/image_rect_raw',
                     '/camera/driver/aligned_depth_to_color/camera_info': '/camera/depth/camera_info'}
    assert not any('driver/depth/image_rect_raw' in k for k in pairs)          # raw depth never on a contract name
    assert not any('points' in k or 'points' in v for k, v in pairs.items())  # the driver's cloud: not the contract's


def test_the_launch_states_every_stream_and_the_timestamp_and_qos_choices():
    params = _launch_module().driver_parameters(PROFILE, serial_no='123456789012')
    assert params['serial_no'] == '123456789012' and isinstance(params['serial_no'], str)
    assert params['camera_name'] == PROFILE['frames']['prefix']
    assert (params['enable_color'], params['enable_depth']) == (True, True)
    # the driver's defaults also open both infrared streams and the IMU: with those on, the D455 dropped off the
    # bus on the development machine (D0.4 spike); the contract needs colour + depth only
    assert not any(params[k] for k in ('enable_infra', 'enable_infra1', 'enable_infra2', 'enable_gyro',
                                       'enable_accel'))
    color, depth = PROFILE['sensor']['color'], PROFILE['sensor']['depth']
    assert params['rgb_camera.color_profile'] == f'{color["width"]}x{color["height"]}x{color["fps"]}'
    assert params['depth_module.depth_profile'] == f'{depth["width"]}x{depth["height"]}x{depth["fps"]}'
    assert (params['rgb_camera.color_format'], params['depth_module.depth_format']) == ('RGB8', 'Z16')
    assert params['align_depth.enable'] is True and params['pointcloud.enable'] is False
    assert params['rgb_camera.global_time_enabled'] is True and params['depth_module.global_time_enabled'] is True
    assert params['publish_tf'] is False                                      # the description owns the frames
    assert (params['color_qos'], params['depth_qos']) == ('SENSOR_DATA', 'SENSOR_DATA')    # BEST_EFFORT
    assert params['pointcloud.pointcloud_qos'] == 'SENSOR_DATA'


def _setup_actions(**arguments):
    from launch import LaunchContext
    launch = _launch_module()
    context = LaunchContext()
    for name, default, _ in launch.ARGUMENTS:
        context.launch_configurations[name] = arguments.get(name, default)
    return launch._setup(context)


def _params(node):
    """A launch Node's first parameter dict with its (normalised) keys and string values resolved."""
    from launch import LaunchContext
    from launch.utilities import perform_substitutions

    def plain(value):
        if isinstance(value, tuple) and value and all(hasattr(v, 'perform') for v in value):
            return perform_substitutions(LaunchContext(), list(value))
        return value
    # launch_ros normalises a value to a substitution of its YAML text: read it back as YAML
    return {plain(k): yaml.safe_load(plain(v)) if isinstance(plain(v), str) else plain(v)
            for k, v in node._Node__parameters[0].items()}


def _node(actions, executable):
    found = [a for a in actions if getattr(a, 'node_executable', None) == executable]
    assert len(found) == 1, [getattr(a, 'node_executable', a) for a in actions]
    return found[0]


def test_the_launch_builds_the_driver_its_source_info_and_optionally_the_description():
    actions = _setup_actions(namespace='bench_cam/', host_id='bench')
    driver = _node(actions, 'realsense2_camera_node')
    assert driver._Node__node_namespace == '/bench_cam' and driver._Node__node_name == 'driver'   # normalised
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    remaps = {perform_substitutions(LaunchContext(), a): perform_substitutions(LaunchContext(), b)
              for a, b in driver._Node__remappings}
    assert remaps == dict(_launch_module().contract_remappings('/bench_cam', '/bench_cam/driver'))
    assert _params(driver)['publish_tf'] is False
    info = _node(actions, 'realsense_source_info')
    params = _params(info)
    assert json.loads(params.pop('dds_transport'))['mode'] in dds.MODES
    assert params == {'profile': 'realsense_d455', 'namespace': '/bench_cam', 'driver_name': 'driver',
                      'host_id': 'bench'}
    assert not [a for a in actions if getattr(a, 'node_executable', None) == 'robot_state_publisher']
    assert not [a for a in _setup_actions(source_info='false')
                if getattr(a, 'node_executable', None) == 'realsense_source_info']


def test_the_standalone_description_is_refused_for_another_camera_model(monkeypatch):
    from trainit_hardware_layer import profiles
    other = dict(PROFILE, description=dict(PROFILE['description'], macro='sensor_d435'))
    monkeypatch.setattr(profiles, 'load', lambda role, model: other)
    with pytest.raises(RuntimeError, match='D455'):
        _setup_actions(standalone_description='true')


# ---- the source-info node's evidence rules (pure) and process choice -----------------------------------------------

def _window(n, domain='global_time', start=10_000_000_000, step=33_333_333, ts_offset_ns=0):
    meta = [(start + i * step, domain, (start + i * step + ts_offset_ns) / 1e6) for i in range(n)]
    heads = [start + i * step for i in range(n)]
    return meta, heads


def _evaluate(metadata, headers, warmup=2, observe=6):
    from trainit_hardware_layer.camera.realsense_source_info import evaluate_timestamping
    return evaluate_timestamping(metadata, headers, warmup=warmup, observe=observe, via='test')


def test_the_stamp_origin_is_claimed_when_every_stream_and_the_contract_headers_establish_it():
    meta, heads = _window(10)
    block = _evaluate({'color': meta, 'depth': meta}, {'color': heads, 'depth': heads})
    assert block['stamp_origin'] == 'DRIVER_GLOBAL_TIME'
    assert block['clock_domains'] == {'color': 'global_time', 'depth': 'global_time'}
    assert block['frames_observed'] == {'color': 6, 'depth': 6}
    assert block['header_evidence']['max_abs_color_header_minus_frame_timestamp_us'] == 0.0
    assert si.parse(_doc(timestamping=block))['timestamping'] == block


@pytest.mark.parametrize('case', ['depth silent', 'depth short', 'colour short', 'warm-up domain only',
                                  'mixed after warm-up', 'headers not the metadata clock', 'depth headers unrelated',
                                  'no contract headers', 'bad metadata'])
def test_the_stamp_origin_is_withheld_without_complete_evidence(case):
    meta, heads = _window(10)
    metadata, headers = {'color': meta, 'depth': meta}, {'color': heads, 'depth': heads}
    if case == 'depth silent':
        metadata['depth'] = []
    elif case == 'depth short':
        metadata['depth'] = meta[:5]                     # warm-up 2 + 3 observed < 6
    elif case == 'colour short':
        metadata['color'] = meta[:7]
    elif case == 'warm-up domain only':
        metadata['color'] = _window(2, 'global_time')[0] + _window(8, 'hardware_clock')[0]
    elif case == 'mixed after warm-up':
        metadata['color'] = meta[:5] + _window(5, 'system_time', start=meta[5][0])[0]
    elif case == 'headers not the metadata clock':
        metadata['color'] = _window(10, ts_offset_ns=5_000_000)[0]          # 5 ms apart
    elif case == 'depth headers unrelated':
        headers['depth'] = [h + 1_000_000 for h in heads]
    elif case == 'no contract headers':
        headers = {'color': [], 'depth': []}
    elif case == 'bad metadata':
        metadata['depth'] = [(h, None, None) for h, _, _ in meta]
    block = _evaluate(metadata, headers)
    assert 'stamp_origin' not in block, block
    assert si.parse(_doc(timestamping=block))                            # still a valid, honest document


def _fake_process(proc, pid, argv, domain=None, maps='7f r-xp 0 0 0 /x/librealsense2.so.2.58.4\n'):
    d = proc / str(pid)
    d.mkdir(parents=True)
    (d / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in argv) + b'\0')
    if domain is not None:
        (d / 'environ').write_bytes(f'PATH=/usr/bin\0ROS_DOMAIN_ID={domain}\0'.encode())
    if maps is not None:
        (d / 'maps').write_text(maps)


DRIVER_ARGV = ['/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node', '--ros-args', '-r', '__node:=driver',
               '-r', '__ns:=/camera', '--params-file', '/tmp/x.yaml']


def test_the_one_driver_process_of_this_namespace_and_domain_is_read(tmp_path):
    from trainit_hardware_layer.camera.realsense_source_info import driver_process_maps
    proc = tmp_path / 'proc'
    _fake_process(proc, 100, DRIVER_ARGV, domain=88)
    _fake_process(proc, 101, DRIVER_ARGV, domain=10, maps='another domain\n')          # another domain's driver
    _fake_process(proc, 102, [a.replace('/camera', '/other') for a in DRIVER_ARGV], domain=88)
    _fake_process(proc, 103, ['/usr/bin/gdb', '--args'] + DRIVER_ARGV, domain=88)        # not the driver itself
    assert 'librealsense2.so.2.58.4' in driver_process_maps('/camera', 'driver', proc_root=proc, domain_id=88)


@pytest.mark.parametrize('twin', ['same domain', 'unreadable environ'])
def test_two_candidate_driver_processes_make_the_library_unknown(tmp_path, twin):
    from trainit_hardware_layer.camera.realsense_source_info import driver_process_maps
    proc = tmp_path / 'proc'
    _fake_process(proc, 100, DRIVER_ARGV, domain=88)
    _fake_process(proc, 200, DRIVER_ARGV, domain=88 if twin == 'same domain' else None)
    assert driver_process_maps('/camera', 'driver', proc_root=proc, domain_id=88) is None


def test_an_unreadable_map_makes_the_library_unknown(tmp_path):
    from trainit_hardware_layer.camera.realsense_source_info import driver_process_maps
    proc = tmp_path / 'proc'
    _fake_process(proc, 100, DRIVER_ARGV, domain=88, maps=None)
    assert driver_process_maps('/camera', 'driver', proc_root=proc, domain_id=88) is None


def test_the_standalone_description_roots_the_nominal_d455_frames_at_its_mount():
    xacro = pytest.importorskip('xacro')
    pytest.importorskip('ament_index_python')
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        get_package_share_directory('realsense2_description')
    except PackageNotFoundError:
        pytest.skip('realsense2_description is not installed')
    urdf = xacro.process_file(str(ROOT / 'urdf' / 'd455_standalone.urdf.xacro'), mappings={'name': 'camera'}).toxml()
    dom = xml.dom.minidom.parseString(urdf)
    parent = {j.getElementsByTagName('child')[0].getAttribute('link'):
              j.getElementsByTagName('parent')[0].getAttribute('link') for j in dom.getElementsByTagName('joint')}
    chain, link = [], 'camera_color_optical_frame'
    while link in parent:
        chain.append(link)
        link = parent[link]
    assert link == 'camera_mount'
    assert chain[-1] == 'camera_bottom_screw_frame' and 'camera_link' in chain


# ---- the driver's DDS transport for large images (dds_transport.py + the launch) -----------------------------------

FASTDDS = 'rmw_fastrtps_cpp'


def _configure(tmp_path, requested='auto', rmw=FASTDDS, environ=None, shm=16 << 30, pointcloud=False):
    return dds.configure(requested, PROFILE, pointcloud=pointcloud, rmw=rmw, environ=environ or {},
                         shm_bytes=shm, directory=tmp_path / 'fastdds')


def test_the_segment_is_computed_from_the_profile_and_holds_sixteen_framesets():
    colour = PROFILE['sensor']['color']
    frameset = dds.frameset_bytes(PROFILE)
    assert frameset == colour['width'] * colour['height'] * 5                  # RGB8 + aligned Z16 on the colour grid
    assert frameset == 2_035_200                                                # 848x480: 1.2 MB + 0.8 MB
    assert dds.segment_bytes(frameset) == (32 * dds.MIB, False)                 # the size measured live (D0-C)
    depth = PROFILE['sensor']['depth']
    assert dds.frameset_bytes(PROFILE, pointcloud=True) == frameset + depth['width'] * depth['height'] * 16
    big = dict(PROFILE, sensor={'color': dict(colour, width=1280, height=720), 'depth': depth})
    assert dds.segment_bytes(dds.frameset_bytes(big))[0] >= dds.FRAMESETS_HELD * 1280 * 720 * 5   # any resolution


@pytest.mark.parametrize('shm, expected', [
    (64 * dds.MIB, (16 * dds.MIB, True)),               # a container's default /dev/shm: capped at 1/4
    (1 * dds.MIB, (dds.FASTDDS_DEFAULT_SEGMENT_BYTES, True)),     # never below the Fast DDS default
    (None, (32 * dds.MIB, False)),                      # unknown /dev/shm: no cap
])
def test_the_segment_is_capped_by_the_hosts_shared_memory(shm, expected):
    assert dds.segment_bytes(dds.frameset_bytes(PROFILE), shm) == expected


def test_on_fast_dds_the_driver_alone_gets_a_generated_profile(tmp_path):
    decision, env = _configure(tmp_path)
    assert decision == {'rmw': FASTDDS, 'mode': 'APPLIED', 'shm_segment_bytes': 32 * dds.MIB,
                        'frameset_bytes': 2_035_200, 'framesets_held': 16, 'capped_by_dev_shm': False,
                        'profile_sha256': decision['profile_sha256']}
    path = Path(env[dds.PROFILE_ENV])
    assert list(env) == [dds.PROFILE_ENV] and path.parent == tmp_path / 'fastdds'
    ns = {'f': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    root = ET.fromstring(path.read_text(encoding='utf-8'))
    kinds = {d.find('f:type', ns).text: d for d in root.iter('{%s}transport_descriptor' % ns['f'])}
    assert set(kinds) == {'SHM', 'UDPv4'}                       # other hosts keep receiving over UDP
    assert int(kinds['SHM'].find('f:segment_size', ns).text) == 32 * dds.MIB
    participant = root.find('f:participant', ns)
    assert participant.get('is_default_profile') == 'true'
    assert participant.find('f:rtps/f:useBuiltinTransports', ns).text == 'false'
    assert _configure(tmp_path) == (decision, env)               # content-addressed: the same file, reused
    assert len(list((tmp_path / 'fastdds').iterdir())) == 1
    assert si.parse(_doc(deployment={'origin': 'DECLARED', 'namespace': '/camera', 'dds_transport': decision}))


@pytest.mark.parametrize('variable', ['FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'])
def test_a_users_fast_dds_profile_is_never_overridden(tmp_path, variable):
    decision, env = _configure(tmp_path, environ={variable: '/somewhere/their_profile.xml'})
    assert decision == {'rmw': FASTDDS, 'mode': 'USER_PROFILE', 'variable': variable}       # no path recorded
    assert env == {} and not (tmp_path / 'fastdds').exists()


@pytest.mark.parametrize('requested, rmw, mode', [('auto', 'rmw_cyclonedds_cpp', 'OTHER_RMW'),
                                                  ('off', FASTDDS, 'OFF')])
def test_another_middleware_or_off_leaves_the_driver_alone(tmp_path, requested, rmw, mode):
    decision, env = _configure(tmp_path, requested=requested, rmw=rmw)
    assert decision == {'rmw': rmw, 'mode': mode} and env == {}
    assert not (tmp_path / 'fastdds').exists()


def test_an_unknown_middleware_still_gets_the_profile_and_records_no_rmw(tmp_path):
    decision, env = _configure(tmp_path, rmw=None)               # harmless on another middleware: it ignores it
    assert decision['mode'] == 'APPLIED' and 'rmw' not in decision and dds.PROFILE_ENV in env
    assert 'not established' in decision['note'] and 'not established' in dds.describe(decision)


def test_an_unwritable_profile_directory_is_recorded_not_raised(tmp_path):
    (tmp_path / 'fastdds').write_text('a file where the directory should be')
    decision, env = dds.configure('auto', PROFILE, pointcloud=False, rmw=FASTDDS, environ={},
                                  shm_bytes=None, directory=tmp_path / 'fastdds' / 'sub')
    assert decision['mode'] == 'NOT_APPLIED' and 'could not be written' in decision['reason'] and env == {}


def test_a_bad_request_is_refused(tmp_path):
    with pytest.raises(ValueError, match='dds_transport'):
        _configure(tmp_path, requested='on')


def test_the_profile_directory_follows_ros_home():
    assert dds.profile_directory({'ROS_HOME': '/r'}) == Path('/r/trainit/fastdds')
    assert dds.profile_directory({}).parts[-3:] == ('.ros', 'trainit', 'fastdds')


def _driver_env(node):
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    return {perform_substitutions(LaunchContext(), k): perform_substitutions(LaunchContext(), v)
            for k, v in (node.process_description.additional_env or [])}


def test_the_launch_applies_the_profile_to_the_driver_only_and_hands_the_decision_on(monkeypatch, tmp_path):
    import rclpy
    monkeypatch.setattr(rclpy, 'get_rmw_implementation_identifier', lambda: FASTDDS)
    actions = _setup_actions(standalone_description='true')
    env = _driver_env(_node(actions, 'realsense2_camera_node'))
    assert Path(env[dds.PROFILE_ENV]).parent == tmp_path / 'ros_home' / 'trainit' / 'fastdds'
    for other in ('realsense_source_info', 'robot_state_publisher'):
        assert dds.PROFILE_ENV not in _driver_env(_node(actions, other))       # the publisher alone needs it
    decision = json.loads(_params(_node(actions, 'realsense_source_info'))['dds_transport'])
    assert decision['mode'] == 'APPLIED' and decision['shm_segment_bytes'] == 32 * dds.MIB
    assert len([a for a in actions if type(a).__name__ == 'LogInfo']) == 1
    off = _setup_actions(dds_transport='off')
    assert _driver_env(_node(off, 'realsense2_camera_node')) == {}
    assert json.loads(_params(_node(off, 'realsense_source_info'))['dds_transport'])['mode'] == 'OFF'


# ---- what reached a local subscriber (contract_delivery, pure) -----------------------------------------------------

def test_every_frameset_delivered_is_a_fraction_of_one():
    from trainit_hardware_layer.camera.realsense_source_info import contract_delivery
    meta, heads = _window(30)
    delivery = contract_delivery({'color': meta, 'depth': meta}, {'color': heads, 'depth': heads})
    assert delivery['color'] == delivery['depth'] == {'driver_framesets': 30, 'received': 30, 'fraction': 1.0}


def test_lost_images_are_counted_against_the_drivers_framesets():
    from trainit_hardware_layer.camera.realsense_source_info import contract_delivery
    meta, heads = _window(30)
    depth = heads[::3]                                   # two of every three depth images lost in transport
    delivery = contract_delivery({'color': meta, 'depth': meta}, {'color': heads, 'depth': depth})
    assert delivery['depth'] == {'driver_framesets': 28, 'received': 10, 'fraction': 0.357}
    assert contract_delivery({'color': meta}, {'color': heads, 'depth': []})['depth'] == {
        'driver_framesets': 30, 'received': 0, 'fraction': 0.0}
    assert contract_delivery({'color': [], 'depth': meta}, {'color': heads, 'depth': heads}) is None


def test_a_header_whose_metadata_was_lost_is_not_counted_so_the_fraction_never_exceeds_one():
    from trainit_hardware_layer.camera.realsense_source_info import contract_delivery
    meta, heads = _window(30)
    reference = meta[:10] + meta[11:]                    # one colour metadata message lost (best-effort, live)
    delivery = contract_delivery({'color': reference}, {'color': heads, 'depth': heads})
    assert delivery['depth'] == {'driver_framesets': 29, 'received': 29, 'fraction': 1.0}


def test_the_timestamping_block_carries_the_delivery_and_stays_a_valid_document():
    meta, heads = _window(10)
    block = _evaluate({'color': meta, 'depth': meta}, {'color': heads, 'depth': heads[::2]})
    assert block['contract_delivery']['depth']['received'] == 5
    assert si.parse(_doc(timestamping=block))


def test_a_failed_profile_write_leaves_no_temporary_file(tmp_path, monkeypatch):
    def refuse(src, dst):
        raise OSError(28, 'No space left on device')
    monkeypatch.setattr(dds.os, 'replace', refuse)
    decision, env = _configure(tmp_path)
    assert decision['mode'] == 'NOT_APPLIED' and env == {}
    assert list((tmp_path / 'fastdds').iterdir()) == []


@pytest.mark.parametrize('shm, needle', [(64 * dds.MIB, 'capped by /dev/shm'),
                                         (16 * dds.MIB, 'a slow subscriber will lose images'),
                                         (1 * dds.MIB, 'below ONE frameset')])
def test_the_launch_log_says_how_much_a_capped_segment_still_holds(tmp_path, shm, needle):
    decision, _ = _configure(tmp_path, shm=shm)
    assert decision['capped_by_dev_shm'] is True and needle in dds.describe(decision)


@pytest.mark.parametrize('variable, value, mode', [
    ('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp', 'OTHER_RMW'),                   # e.g. a parent launch's choice
    ('FASTRTPS_DEFAULT_PROFILES_FILE', '/their/profile.xml', 'USER_PROFILE'),
])
def test_the_launch_never_overrides_a_choice_made_in_its_environment(monkeypatch, variable, value, mode):
    import rclpy
    monkeypatch.setattr(rclpy, 'get_rmw_implementation_identifier', lambda: FASTDDS)
    monkeypatch.setenv(variable, value)               # Humble's LaunchContext.environment IS the process environment
    actions = _setup_actions()
    assert _driver_env(_node(actions, 'realsense2_camera_node')) == {}
    assert json.loads(_params(_node(actions, 'realsense_source_info'))['dds_transport'])['mode'] == mode


def test_a_stream_with_fewer_than_two_headers_is_measured_over_the_whole_reference():
    from trainit_hardware_layer.camera.realsense_source_info import contract_delivery
    meta, heads = _window(30)
    delivery = contract_delivery({'color': meta}, {'color': heads, 'depth': heads[5:6]})
    assert delivery['depth'] == {'driver_framesets': 30, 'received': 1, 'fraction': 0.033}


# ---- the driver's clock mapping, watched after start-up (ClockWatch, pure) ----------------------------------------

def _watch(offsets, **kw):
    from trainit_hardware_layer.camera.realsense_source_info import ClockWatch
    watch, events = ClockWatch(**kw), []
    for i, offset in enumerate(offsets):
        header_ns = 1_000_000_000_000 + i * 33_333_333
        event = watch.update(header_ns, header_ns / 1e6 - offset)
        if event:
            events.append((i, event[0], event[1]))
    return watch, events


def test_a_steady_mapping_with_jitter_raises_nothing():
    jitter = [-27.5 + ((i * 7) % 5 - 2) * 0.5 for i in range(600)]          # +-1 ms, 20 s
    jitter += [-27.5 + 10.4] * 540                     # the measured worst healthy episode: +10.4 ms for 18 s
    watch, events = _watch(jitter)
    assert events == [] and watch.baseline == pytest.approx(-27.5, abs=1.0)


def test_a_mapping_running_ahead_is_reported_once_then_its_recovery():
    healthy = [-27.5] * 60
    ramp = [-27.5 + 0.5 * i for i in range(1, 200)]       # 15 ms/s ahead, as measured in D0-C
    rebased = [-27.5] * 60                                # librealsense re-bases its fit
    watch, events = _watch(healthy + ramp + rebased)
    assert [e[1] for e in events] == ['AHEAD', 'OK']
    first = events[0]
    assert first[0] == 60 + 40 + 15 - 1 and first[2]['departure_ms'] > 20       # sustained, not a single frame
    assert events[1][2]['worst_departure_ms'] == pytest.approx(99.5, abs=0.6)    # signed: ahead is positive
    assert watch.ahead_events == 1 and watch.state == 'OK'


def test_a_brief_spike_or_flapping_at_the_threshold_is_ignored():
    assert _watch([-27.5] * 40 + [100.0] * 10 + [-27.5] * 40)[1] == []
    flapping = [-27.5] * 40 + [-27.5 + (22 if i % 2 else 18) for i in range(200)]
    assert _watch(flapping)[1] == []


def test_late_arrivals_are_behind_never_ahead():
    watch, events = _watch([-27.5] * 40 + [-60.0] * 40 + [-27.5] * 40)
    assert [e[1] for e in events] == ['BEHIND', 'OK'] and watch.ahead_events == 0
    assert events[1][2]['worst_departure_ms'] == pytest.approx(-32.5)


def test_a_header_after_its_own_arrival_is_ahead_whatever_the_baseline():
    watch, events = _watch([200.0] * 40 + [150.0] * 40)     # start-up while the fit was already wrong
    assert watch.baseline == 200.0 and [e[1] for e in events] == ['AHEAD']


def test_unusable_metadata_never_poisons_the_baseline():
    from trainit_hardware_layer.camera.realsense_source_info import ClockWatch
    watch = ClockWatch(baseline_frames=3)
    for header_ns, arrival in ((1_000_000_000, float('nan')), (0, 1.0), (1_000_000_000, float('inf'))):
        assert watch.update(header_ns, arrival) is None
    assert watch.collected == [] and watch.baseline is None
