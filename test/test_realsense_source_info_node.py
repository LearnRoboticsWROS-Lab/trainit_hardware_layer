"""The real backend's source-information NODE against a FAKE driver (private domain 86, no device).

The fake serves what realsense2_camera 4.58.4 serves and the node reads: ``<ns>/driver/device_info``,
``<ns>/driver/get_parameters``, ``<ns>/driver/<stream>/metadata`` (JSON with ``clock_domain`` / ``frame_timestamp``)
and the CONTRACT images. Whether the real driver publishes exactly these is the owner's D0-C check.
"""
import json
import os
import signal
import subprocess
import threading
import time

import pytest

pytest.importorskip('realsense2_camera_msgs', reason='needs ros-humble-realsense2-camera-msgs')

from trainit_hardware_layer.camera import source_info as si  # noqa: E402

NS = '/camera'
STEP_NS = 33_333_333


class FakeDriver:
    def __init__(self, harness, *, device_info=True, depth_metadata=True, header_offset_ns=0,
                 clock_domain='global_time', arrival_drift_ms_after=None):
        from rcl_interfaces.msg import ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters
        from realsense2_camera_msgs.msg import Metadata
        from realsense2_camera_msgs.srv import DeviceInfo
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        self.node = harness.helper('driver', namespace=NS)
        self.values = {'enable_color': True, 'align_depth.enable': True, 'rgb_camera.color_profile': '848x480x30',
                       'rgb_camera.global_time_enabled': True, 'publish_tf': False}

        def parameters(request, response):
            for name in request.names:
                value = self.values.get(name)
                if isinstance(value, bool):
                    response.values.append(ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))
                elif isinstance(value, str):
                    response.values.append(ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value))
                else:
                    response.values.append(ParameterValue(type=ParameterType.PARAMETER_NOT_SET))
            return response

        def info(request, response):
            response.device_name, response.serial_number = 'realsense_d455', '000000000455'
            response.firmware_version, response.usb_type_descriptor = '0.0.0.0', '3.2'
            response.physical_port = '/dev/not-a-sysfs-path'
            return response
        self.node.create_service(GetParameters, 'driver/get_parameters', parameters)
        if device_info:
            self.node.create_service(DeviceInfo, 'driver/device_info', info)
        meta = {s: self.node.create_publisher(Metadata, f'driver/{s}/metadata', qos_profile_sensor_data)
                for s in ('color', 'depth')}
        images = {'color': self.node.create_publisher(Image, f'{NS}/color/image_raw', qos_profile_sensor_data),
                  'depth': self.node.create_publisher(Image, f'{NS}/depth/image_rect_raw', qos_profile_sensor_data)}
        self.tick = 0

        def publish():
            self.tick += 1
            stamp_ns = 1_793_000_000_000_000_000 + self.tick * STEP_NS
            for stream in ('color', 'depth'):
                if stream == 'color' or depth_metadata:
                    m = Metadata()
                    m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
                    drift = 0.0                     # from frame N on, the header runs ahead 2 ms per frame
                    if arrival_drift_ms_after is not None and self.tick > arrival_drift_ms_after:
                        drift = 2.0 * (self.tick - arrival_drift_ms_after)
                    m.json_data = json.dumps({'clock_domain': clock_domain,
                                              'frame_timestamp': (stamp_ns - header_offset_ns) / 1e6,
                                              'time_of_arrival': stamp_ns / 1e6 + 27.0 - drift})
                    meta[stream].publish(m)
                img = Image(height=1, width=1, encoding='rgb8' if stream == 'color' else '16UC1',
                            step=3 if stream == 'color' else 2, data=b'\0' * (3 if stream == 'color' else 2))
                img.header.stamp.sec, img.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
                images[stream].publish(img)
        self.node.create_timer(0.01, publish)


def run_node(harness, return_node=False, **params):
    from trainit_hardware_layer.camera.realsense_source_info import RealSenseSourceInfo
    values = {'namespace': NS, 'host_id': 'bench', 'warmup_frames': 3, 'observe_frames': 10, 'timeout_s': 6.0}
    values.update(params)
    node = RealSenseSourceInfo(context=harness.ctx, parameter_overrides=harness.overrides(values))
    harness._add(node, spin=False)                    # it spins itself (its own executor), then is destroyed
    received = []
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String
    listener = harness.helper('source_info_listener')
    listener.create_subscription(String, f'{NS}/source_info', lambda m: received.append(m.data), QoSProfile(
        depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    done = threading.Event()
    result = {}

    def target():
        result['doc'] = node.run()
        done.set()
    threading.Thread(target=target, daemon=True).start()
    assert done.wait(20.0), 'the source-info node did not finish within its deadline'
    import eef_harness
    assert eef_harness.wait_until(lambda: received, 5.0), 'no latched document received'
    published = json.loads(received[-1])
    assert published == result['doc']
    return (si.parse(published), node) if return_node else si.parse(published)


TRANSPORT = {'mode': 'APPLIED', 'rmw': 'rmw_fastrtps_cpp', 'shm_segment_bytes': 33554432}


def test_the_node_publishes_the_full_document_from_the_drivers_surfaces(harness):
    FakeDriver(harness)
    doc = run_node(harness, dds_transport=json.dumps(TRANSPORT))
    assert doc['backend'] == 'real'
    assert doc['device'] == {'origin': 'DISCOVERED', 'via': 'driver/device_info (realsense2_camera)',
                             'name': 'realsense_d455', 'serial_number': '000000000455',
                             'firmware_version': '0.0.0.0', 'usb_type_descriptor': '3.2'}
    assert 'attachment' not in doc                     # the port is no sysfs path: unknown, never guessed
    assert doc['deployment'] == {'origin': 'DECLARED', 'namespace': NS, 'host_id': 'bench',
                                 'dds_transport': TRANSPORT}
    assert doc['configuration']['parameters']['align_depth.enable'] is True
    assert 'depth_module.depth_profile' not in doc['configuration']['parameters']        # not set: left out
    t = doc['timestamping']
    assert t['clock_domains'] == {'color': 'global_time', 'depth': 'global_time'}
    assert t['stamp_origin'] == 'DRIVER_GLOBAL_TIME'
    assert t['header_evidence']['max_abs_color_header_minus_frame_timestamp_us'] <= 1.0
    assert doc['host']['boot_id_sha256'] == si.boot_id_sha256()
    delivery = t['contract_delivery']
    assert delivery['color']['fraction'] >= 0.9 and delivery['depth']['fraction'] >= 0.9   # tiny images: none lost


def test_without_a_device_service_and_with_silent_depth_metadata_the_document_says_less_never_more(harness):
    FakeDriver(harness, device_info=False, depth_metadata=False)
    doc = run_node(harness, service_wait_s=1.0)
    assert 'device' not in doc
    assert doc['timestamping']['clock_domains']['depth'] == 'NOT_OBSERVED'
    assert 'stamp_origin' not in doc['timestamping']


def test_headers_that_are_not_the_metadata_clock_withhold_the_stamp_origin(harness):
    FakeDriver(harness, header_offset_ns=5_000_000)
    doc = run_node(harness)
    assert doc['timestamping']['clock_domains']['color'] == 'global_time'
    assert 'stamp_origin' not in doc['timestamping']


def test_a_malformed_transport_decision_is_left_out(harness):
    FakeDriver(harness)
    doc = run_node(harness, dds_transport='not json')
    assert 'dds_transport' not in doc['deployment']


@pytest.mark.parametrize('gap_s', [0.0, 0.005, 0.02, 0.1])
def test_the_installed_node_stops_cleanly_on_the_double_sigint_of_a_terminal_ctrl_c(tmp_path, gap_s):
    """Ctrl+C under ros2 launch reaches the node twice (terminal + launch): it must still exit 0, never -2."""
    import eef_harness as h
    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix('trainit_hardware_layer'), 'lib', 'trainit_hardware_layer',
                       'realsense_source_info')
    env = dict(os.environ, ROS_DOMAIN_ID=str(h.DOMAIN_ID), ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(tmp_path / 'ros_log'), PYTHONUNBUFFERED='1')
    process = subprocess.Popen([exe, '--ros-args', '-p', 'namespace:=/no_camera_here', '-p', 'timeout_s:=1.0',
                                '-p', 'service_wait_s:=0.2'], env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 20
        line = ''
        while 'source_info on' not in line and time.monotonic() < deadline:
            line = process.stdout.readline()          # the document is published: the node now spins
        assert 'source_info on' in line, 'the node never published its document'
        process.send_signal(signal.SIGINT)
        time.sleep(gap_s)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        assert process.wait(timeout=15) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_the_node_keeps_watching_the_drivers_clock_mapping_after_publishing(harness):
    FakeDriver(harness, arrival_drift_ms_after=120)        # healthy for ~1.2 s, then the header runs ahead
    _, node = run_node(harness, return_node=True)
    node.watch_clock()
    deadline = time.monotonic() + 10
    while node.clock_watch.ahead_events == 0 and time.monotonic() < deadline:
        node.executor_.spin_once(timeout_sec=0.05)
    assert node.clock_watch.baseline is not None and abs(node.clock_watch.baseline + 27.0) < 0.5
    assert node.clock_watch.ahead_events == 1 and node.clock_watch.state == 'AHEAD'
