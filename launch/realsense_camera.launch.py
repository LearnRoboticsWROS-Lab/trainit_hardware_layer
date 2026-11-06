"""The REAL camera backend: a RealSense D4xx through ``realsense2_camera``, remapped onto the camera contract.

    ros2 launch trainit_hardware_layer realsense_camera.launch.py                       # in a cell
    ros2 launch trainit_hardware_layer realsense_camera.launch.py standalone_description:=true host_id:=bench

What it starts, on the host the camera is plugged into (the ACQUISITION host):

- the driver, as ``<ns>/driver``: EVERY stream stated explicitly (colour + depth only: the driver's own defaults
  also open both infrared streams and the IMU), depth ALIGNED to colour, librealsense global time on, image QoS
  ``SENSOR_DATA`` (BEST_EFFORT, the contract's), the driver's TF OFF (the description owns the frames);
- the remaps onto the contract (contracts/camera.md): ``<ns>/color/image_raw``, ``<ns>/color/camera_info``,
  ``<ns>/depth/image_rect_raw`` and ``<ns>/depth/camera_info`` from the driver's ALIGNED depth -- its raw depth stays
  inside ``<ns>/driver`` and never reaches a contract name ("Registered depth", clause 5). The driver's own point
  cloud (``pointcloud:=true``) stays inside ``<ns>/driver`` too: it is unorganised and in the DEPTH optical frame,
  not the contract's organised, colour-registered cloud (measured 2026-10-30; contracts/camera.md, ``real``);
- ``realsense_source_info``: the latched ``<ns>/source_info`` document (device, USB attachment, host, software,
  effective configuration, observed clock domain);
- the driver's DDS transport for large images (``dds_transport:=auto``, the default): on Fast DDS the driver process
  alone gets a generated profile whose shared-memory segment holds the configured framesets (the 512 KiB default
  loses most 0.8-1.2 MB images on one host; ``camera/dds_transport.py``). A Fast DDS profile the user already set, or
  another middleware, is left alone; the decision is logged and recorded in the source information;
- with ``standalone_description:=true`` only: ``robot_state_publisher`` on ``urdf/d455_standalone.urdf.xacro``
  (the camera on its own, nominal frames rooted at ``<prefix>_mount``). In a cell, the cell's description publishes
  the camera frames and this stays off.

The resolution and frame rate come from the hardware profile (``sensor.color``); the frame prefix and the namespace
too. Two cameras of one profile need distinct frame prefixes, which this launch does not set yet (D0.4 debt).
Nothing here is a deployment topology: the same launch runs on a workstation or on an edge computer, and the recorder
may run on the same host or another one (ROS 2 / DDS carries the contract).
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from trainit_hardware_layer import profiles
from trainit_hardware_layer.camera import dds_transport

ARGUMENTS = (
    ('profile', 'realsense_d455', 'the hardware camera profile'),
    ('namespace', '', 'the contract namespace (default: the profile\'s topics.namespace)'),
    ('serial_no', '', 'select the device by serial number (default: the first one found)'),
    ('usb_port_id', '', 'select the device by USB port id, e.g. 8-1.4'),
    ('host_id', '', 'a DECLARED logical id of this acquisition host, recorded in the source information'),
    ('standalone_description', 'false', 'publish the camera\'s own nominal frames (no cell description)'),
    ('pointcloud', 'false', 'also the driver\'s own cloud, inside <ns>/driver only (not the contract\'s cloud)'),
    ('source_info', 'true', 'publish the latched <ns>/source_info document'),
    ('dds_transport', 'auto', 'auto: size the driver\'s Fast DDS shared-memory segment for its images; off: defaults'),
)


def _true(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def contract_remappings(namespace, driver):
    """(driver topic, contract topic) pairs: colour and the ALIGNED depth; never the raw depth, never the driver's
    cloud (unorganised, depth optical frame: not the contract's cloud)."""
    pairs = [(f'{driver}/color/image_raw', f'{namespace}/color/image_raw'),
             (f'{driver}/color/camera_info', f'{namespace}/color/camera_info'),
             (f'{driver}/aligned_depth_to_color/image_raw', f'{namespace}/depth/image_rect_raw'),
             (f'{driver}/aligned_depth_to_color/camera_info', f'{namespace}/depth/camera_info')]
    return pairs


def driver_parameters(prof, *, serial_no='', usb_port_id='', pointcloud=False):
    color, depth = prof['sensor']['color'], prof['sensor']['depth']
    return {
        'camera_name': prof['frames']['prefix'],             # the frame prefix: <prefix>_color_optical_frame
        'serial_no': str(serial_no), 'usb_port_id': str(usb_port_id),
        'enable_color': True, 'enable_depth': True,
        'enable_infra': False, 'enable_infra1': False, 'enable_infra2': False,
        'enable_gyro': False, 'enable_accel': False,
        'rgb_camera.color_profile': f'{color["width"]}x{color["height"]}x{color["fps"]}',
        'rgb_camera.color_format': 'RGB8',
        'depth_module.depth_profile': f'{depth["width"]}x{depth["height"]}x{depth["fps"]}',
        'depth_module.depth_format': 'Z16',
        'align_depth.enable': True,
        'pointcloud.enable': bool(pointcloud),
        'rgb_camera.global_time_enabled': True, 'depth_module.global_time_enabled': True,
        'publish_tf': False,
        'color_qos': 'SENSOR_DATA', 'depth_qos': 'SENSOR_DATA',
        'color_info_qos': 'DEFAULT', 'depth_info_qos': 'DEFAULT',
        'pointcloud.pointcloud_qos': 'SENSOR_DATA',          # the driver's own cloud: BEST_EFFORT like the images
    }


def _rmw_identifier():
    """The middleware this launch (and so the driver it starts) runs with, else ``None``."""
    try:
        import rclpy
        return rclpy.get_rmw_implementation_identifier()
    except Exception:  # noqa: BLE001 -- unknown: dds_transport treats it as possibly Fast DDS (harmless elsewhere)
        return None


def driver_transport(requested, prof, pointcloud, environ):
    """(decision, driver environment) for this host: the facts the launch reads, handed to ``dds_transport``.

    ``environ`` is the LAUNCH CONTEXT's environment (what the driver inherits, including a parent launch's
    ``SetEnvironmentVariable``): its ``RMW_IMPLEMENTATION`` wins over the middleware this process loaded.
    """
    rmw = environ.get('RMW_IMPLEMENTATION') or _rmw_identifier()
    return dds_transport.configure(requested, prof, pointcloud=pointcloud, rmw=rmw, environ=environ,
                                   shm_bytes=dds_transport.dev_shm_bytes(),
                                   directory=dds_transport.profile_directory(environ))


def _setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    prof = profiles.load('camera', get('profile'))
    namespace = get('namespace') or prof['topics']['namespace']
    if not namespace.startswith('/'):
        namespace = '/' + namespace
    namespace = namespace.rstrip('/')
    driver_name = 'driver'
    pointcloud = _true(get('pointcloud'))
    transport, driver_env = driver_transport(get('dds_transport'), prof, pointcloud, dict(context.environment))
    actions = [LogInfo(msg=f'[{namespace}] DDS transport: {dds_transport.describe(transport)}'),
               Node(package='realsense2_camera', executable='realsense2_camera_node', namespace=namespace,
                    name=driver_name, output='screen', additional_env=driver_env,
                    parameters=[driver_parameters(prof, serial_no=get('serial_no'), usb_port_id=get('usb_port_id'),
                                                  pointcloud=pointcloud)],
                    remappings=contract_remappings(namespace, f'{namespace}/{driver_name}'))]
    if _true(get('source_info')):
        actions.append(Node(package='trainit_hardware_layer', executable='realsense_source_info',
                            name=f'{prof["frames"]["prefix"]}_source_info', output='screen',
                            parameters=[{'profile': get('profile'), 'namespace': namespace,
                                         'driver_name': driver_name, 'host_id': get('host_id'),
                                         'dds_transport': json.dumps(transport, sort_keys=True)}]))
    if _true(get('standalone_description')):
        import xacro
        if prof['description'].get('macro') != 'sensor_d455':
            raise RuntimeError(f'standalone_description: urdf/d455_standalone.urdf.xacro describes a D455, the profile '
                               f'{get("profile")!r} is another camera')
        path = os.path.join(get_package_share_directory('trainit_hardware_layer'), 'urdf',
                            'd455_standalone.urdf.xacro')
        urdf = xacro.process_file(path, mappings={'name': prof['frames']['prefix']}).toxml()
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            name=f'{prof["frames"]["prefix"]}_standalone_description', output='screen',
                            parameters=[{'robot_description': urdf}]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in ARGUMENTS]
                             + [OpaqueFunction(function=_setup)])
