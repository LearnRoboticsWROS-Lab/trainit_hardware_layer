"""The camera contract's registered-depth clause (contracts/camera.md) and the mock's conformance.

C1 (pure) the clause is in the contract, the mock's source descriptor is declared in the profile, and the mock
   code publishes nothing in the depth optical frame.
C2 (ROS, domain 86) a live mock_camera: rgb8 colour; 16UC1 millimetre depth at the profile's plane distance;
   depth image, depth camera_info and cloud all in the colour optical frame; the two camera_infos identical
   but for nothing; depth on the colour pixel grid; the contract's QoS (BEST_EFFORT images, RELIABLE info).
"""
import struct
import time
from pathlib import Path

from trainit_hardware_layer import profiles

ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / 'contracts' / 'camera.md'
MOCK = ROOT / 'trainit_hardware_layer' / 'camera' / 'mock_camera_node.py'


def test_the_contract_states_the_registered_depth_clause():
    text = CONTRACT.read_text()
    assert '## Registered depth (normative' in text
    for phrase in ('colour optical frame', 'geometrically identical to `<ns>/color/camera_info`',
                   '`16UC1` = millimetres, `32FC1` = metres', 'Raw depth never masquerades as registered depth'):
        assert phrase in text, phrase


def _source_profile():
    # The SOURCE profile under test (profiles.load prefers an installed share, which may be older).
    import yaml
    return yaml.safe_load((ROOT / 'profiles' / 'camera' / 'realsense_d455.yaml').read_text())


def test_the_mock_source_descriptor_is_declared():
    descriptor = profiles.backend(_source_profile(), 'mock')['source_descriptor']
    assert descriptor == {'backend': 'mock', 'intrinsics_kind': 'NOMINAL_PROFILE', 'stamp_origin': 'HEADER',
                          'registration_basis': 'same_pixel_grid'}


def test_the_isaac_source_descriptor_is_declared():
    descriptor = profiles.backend(_source_profile(), 'isaac')['source_descriptor']
    notes = descriptor.pop('intrinsics_notes')
    assert descriptor == {'backend': 'isaac', 'intrinsics_kind': 'SYNTHETIC_RENDER', 'stamp_origin': 'HEADER',
                          'registration_basis': 'same_render_product', 'sim_source': 'isaac'}
    assert 'pixel-edge' in notes


def test_the_mock_never_publishes_in_the_depth_optical_frame():
    assert 'depth_optical' not in MOCK.read_text()


def test_a_live_mock_conforms(harness):
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from trainit_hardware_layer.camera.mock_camera_node import MockCamera

    prof = profiles.load('camera', 'realsense_d455')
    ns, colour_frame = prof['topics']['namespace'], prof['frames']['color_optical']
    best_effort = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST,
                             depth=5)
    reliable = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=5)
    got = {}
    helper = harness.helper()
    for name, kind, qos in (('color/image_raw', Image, best_effort), ('depth/image_rect_raw', Image, best_effort),
                            ('depth/color/points', PointCloud2, best_effort),
                            ('color/camera_info', CameraInfo, reliable), ('depth/camera_info', CameraInfo, reliable)):
        helper.create_subscription(kind, f'{ns}/{name}', lambda m, n=name: got.setdefault(n, m), qos)
    harness._add(MockCamera(context=harness.ctx, parameter_overrides=harness.overrides(
        {'rate_hz': 20.0, 'plane_distance_m': 1.25})))
    deadline = time.monotonic() + 10.0
    while len(got) < 5 and time.monotonic() < deadline:
        time.sleep(0.05)
    assert len(got) == 5, sorted(got)

    colour, depth = got['color/image_raw'], got['depth/image_rect_raw']
    info_c, info_d = got['color/camera_info'], got['depth/camera_info']
    assert colour.encoding == 'rgb8' and depth.encoding == '16UC1'
    assert {colour.header.frame_id, depth.header.frame_id, got['depth/color/points'].header.frame_id,
            info_c.header.frame_id, info_d.header.frame_id} == {colour_frame}
    assert (depth.width, depth.height) == (colour.width, colour.height) == (info_c.width, info_c.height)
    for field in ('width', 'height', 'distortion_model', 'd', 'k', 'r', 'p', 'binning_x', 'binning_y', 'roi'):
        a, b = getattr(info_d, field), getattr(info_c, field)
        assert (list(a) == list(b)) if field in ('d', 'k', 'r', 'p') else (a == b), field
    assert struct.unpack_from('<H', bytes(depth.data))[0] == 1250          # millimetres, unchanged convention
    assert depth.step == depth.width * 2 and not depth.is_bigendian


def test_the_contract_still_names_both_optical_frames_in_the_tf_tree():
    text = CONTRACT.read_text()
    assert '<prefix>_depth_optical_frame' in text and '<prefix>_color_optical_frame' in text
    assert _source_profile()['frames']['depth_optical'] == 'camera_depth_optical_frame'
