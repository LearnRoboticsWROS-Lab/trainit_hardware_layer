#!/usr/bin/env python3
"""The REAL camera backend's source information: a RealSense D4xx behind ``realsense2_camera``.

Runs on the ACQUISITION host, next to the driver (``launch/realsense_camera.launch.py`` starts both). It asks the
driver's own stable surfaces -- never its terminal output -- and publishes one ``trainit-camera-source/1.0``
document, latched, on ``<ns>/source_info`` (contracts/camera.md, "Source information"):

- ``device``         DISCOVERED  the driver's ``device_info`` service (name as the driver reports it, serial, firmware);
- ``attachment``     DISCOVERED  sysfs, from the physical port the driver reports (link speed, bcdUSB, the hub if any);
- ``host``           DISCOVERED  this host (hostname, machine, kernel, the boot-id hash);
- ``deployment``     DECLARED    the launch's ``host_id``, the contract namespace and its DDS transport decision for the
  driver (``dds_transport``: ``camera/dds_transport.py``);
- ``software``       DISCOVERED  the driver package's installed version (ament index);
- ``library``        OBSERVED    the librealsense the ONE running driver process of this namespace / domain has mapped;
- ``configuration``  DISCOVERED  the driver's effective parameters (profiles, alignment, global time, QoS);
- ``timestamping``   OBSERVED    a start-up window: each stream's per-frame clock domain (the driver's metadata), and
  whether the CONTRACT image headers are that clock (colour header == its metadata ``frame_timestamp``; aligned
  depth header == a colour header). ``stamp_origin`` is claimed only when all of it holds. The same window measures
  ``contract_delivery``: how many of the driver's framesets reached THIS local subscriber on the contract topics (a
  loss here is the transport's, not the device's, and is logged as a warning).

After publishing, the node keeps WATCHING the driver's clock mapping (``ClockWatch``): per colour frame, the header
(global time) minus the host time the frame reached librealsense (``time_of_arrival`` in the driver's metadata) is
constant while the mapping is healthy (about -27 ms on the development workstation). A sustained departure from the
start-up value is logged as a warning, and its end as a recovery: librealsense 2.58.4's global-time fit was seen to
run up to 0.53 s ahead for about a minute before re-basing itself (D0-C, 2026-10-30). The published document stays
the start-up snapshot; a recorder on the same host refuses such frames itself (CLOCK_MISMATCH).

Every step is bounded by ONE overall deadline; a step that fails (no service, no message package, a timeout)
leaves its block out and is logged -- the document is still published (unknown is absent). The node then stays up so
the latched document remains available. RealSense specifics live here, in the hardware layer: no recorder ever talks
to the driver.
"""
import json
import math
import os
import signal
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import String

from trainit_hardware_layer import profiles
from trainit_hardware_layer.camera import source_info

LATCHED_QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
# The driver parameters that make up the effective configuration.
CONFIG_PARAMETERS = ('enable_color', 'enable_depth', 'rgb_camera.color_profile', 'rgb_camera.color_format',
                     'depth_module.depth_profile', 'depth_module.depth_format', 'align_depth.enable',
                     'pointcloud.enable', 'rgb_camera.global_time_enabled', 'depth_module.global_time_enabled',
                     'publish_tf', 'color_qos', 'depth_qos', 'color_info_qos', 'depth_info_qos',
                     'pointcloud.pointcloud_qos', 'enable_sync')
STREAMS = ('color', 'depth')
CONTRACT_IMAGES = {'color': 'color/image_raw', 'depth': 'depth/image_rect_raw'}
HEADER_TOLERANCE_US = 1000.0        # a contract header "is" the frame's metadata timestamp within 1 ms
MIN_DEPTH_SHARED = 0.9              # fraction of aligned-depth headers that must equal a colour header
DELIVERY_WARNING = 0.95             # below this fraction of framesets delivered locally, the transport loses images
# ClockWatch: departure from the start-up header-minus-arrival offset. Measured on the D455 over 45 min (2026-10-30,
# 8217 frames): healthy departures reached p99 9.0 ms, max 10.4 ms, with +5..+10 ms episodes lasting ~18 s.
THRESHOLD_MS = 20.0


def _package_version(package):
    try:
        from ament_index_python.packages import get_package_share_directory
        text = (Path(get_package_share_directory(package)) / 'package.xml').read_text()
        return text.split('<version>')[1].split('</version>')[0].strip()
    except Exception:  # noqa: BLE001 -- absent, never invented
        return None


def driver_process_maps(namespace, name, proc_root='/proc', domain_id=None):
    """The memory map of THE running ``realsense2_camera_node`` launched as ``<namespace>/<name>`` on this ROS domain,
    or ``None`` unless exactly one such process is found and readable (a stale twin, another domain's driver or an
    unreadable candidate makes the answer unknown, never a guess)."""
    domain = str(os.environ.get('ROS_DOMAIN_ID', '') or '0') if domain_id is None else str(domain_id)
    found = []
    for cmdline in Path(proc_root).glob('[0-9]*/cmdline'):
        try:
            args = cmdline.read_bytes().split(b'\0')
        except OSError:
            continue
        if not args or os.path.basename(args[0]) != b'realsense2_camera_node':
            continue
        if f'__node:={name}'.encode() not in args or f'__ns:={namespace}'.encode() not in args:
            continue
        try:
            environ = (cmdline.parent / 'environ').read_bytes().split(b'\0')
            theirs = next((e.split(b'=', 1)[1] for e in environ if e.startswith(b'ROS_DOMAIN_ID=')), b'') or b'0'
            if theirs.decode() != domain:
                continue
        except OSError:
            pass                                            # not readable: kept as a candidate, ambiguity decides
        found.append(cmdline.parent / 'maps')
    if len(found) != 1:
        return None
    try:
        return found[0].read_text()
    except OSError:
        return None


def contract_delivery(metadata, headers):
    """How many of the driver's framesets reached the contract subscriber, per stream (pure), else ``None``.

    The reference is the driver's colour metadata: one stamp per frameset, the stamp the contract colour AND aligned
    depth headers carry. Per stream, over the span between its first and last received header (the whole reference
    span when fewer than two arrived): the framesets the driver produced in that span, and how many of THOSE were
    received. A header whose metadata message was itself lost cannot be checked and is not counted, so the fraction
    never exceeds 1 -- and a lost metadata message makes it slightly OPTIMISTIC. It is measured once, at start-up,
    by this node's own subscriber on the acquisition host: it shows what the transport delivers there, not what a
    recorder (another process, maybe another host) received.
    """
    reference = sorted({h for h, _, _ in metadata.get('color', [])})
    if not reference:
        return None
    out = {'reference': 'the driver\'s colour metadata stamps (one per frameset)',
           'subscriber': 'the source node on the acquisition host, at start-up (not the recorder)'}
    for stream in STREAMS:
        got = sorted(set(headers.get(stream, [])))
        low, high = (got[0], got[-1]) if len(got) >= 2 else (reference[0], reference[-1])
        produced = [h for h in reference if low <= h <= high]
        received = len(set(produced) & set(got))
        produced = len(produced)
        out[stream] = {'driver_framesets': produced, 'received': received,
                       'fraction': round(received / produced, 3) if produced else None}
    return out


def evaluate_timestamping(metadata, headers, *, warmup, observe, via):
    """The ``timestamping`` block from a start-up window (pure; the evidence rules live here).

    ``metadata[stream]``: ``[(header_ns, clock_domain, frame_timestamp_ms)]`` of the driver's metadata messages in
    arrival order; ``headers[stream]``: the CONTRACT images' header stamps (ns) in arrival order. A stream with fewer
    than ``observe`` metadata frames after the warm-up is ``NOT_OBSERVED``; ``stamp_origin`` is claimed only when every
    stream's window is one mapped clock domain AND the contract headers are shown to be that clock.
    """
    domains, window = {}, {}
    for stream in STREAMS:
        window[stream] = metadata.get(stream, [])[warmup:warmup + observe]
        values = {d for _, d, _ in window[stream]}
        if len(window[stream]) < observe:
            domains[stream] = 'NOT_OBSERVED'
        elif len(values) == 1 and None not in values:
            domains[stream] = values.pop()
        else:
            domains[stream] = 'MIXED_OR_UNKNOWN'
    colour_ts = {h: ts for h, _, ts in window['color'] if isinstance(ts, (int, float))}
    colour_heads = [h for h in headers.get('color', []) if h in colour_ts]
    deltas = [abs(h - colour_ts[h] * 1e6) / 1e3 for h in colour_heads]
    start = min(colour_ts) if colour_ts else None
    depth_heads = [h for h in headers.get('depth', []) if start is not None and h >= start]
    all_colour = set(headers.get('color', []))
    shared = sum(1 for h in depth_heads if h in all_colour)
    evidence = {'color_headers_matched': len(colour_heads), 'depth_headers': len(depth_heads),
                'depth_headers_equal_to_a_color_header': shared}
    if deltas:
        evidence['max_abs_color_header_minus_frame_timestamp_us'] = round(max(deltas), 3)
    block = {'origin': 'OBSERVED', 'via': via, 'window': 'start-up snapshot after a warm-up (not re-observed)',
             'warmup_frames': warmup, 'clock_domains': domains,
             'frames_observed': {s: len(window[s]) for s in STREAMS}, 'header_evidence': evidence,
             'device_timestamp': 'per frame on the driver metadata topics; not on the camera contract'}
    delivery = contract_delivery(metadata, headers)
    if delivery is not None:
        block['contract_delivery'] = delivery
    origin = source_info.stamp_origin(domains)
    headers_are_that_clock = (len(colour_heads) >= max(1, observe // 2) and max(deltas) <= HEADER_TOLERANCE_US
                              and depth_heads and shared >= MIN_DEPTH_SHARED * len(depth_heads))
    if origin is not None and headers_are_that_clock:
        block['stamp_origin'] = origin
    return block


class ClockWatch:
    """Is the driver's device-to-host clock mapping still the one seen at start-up? (pure; fed per colour frame)

    ``offset`` = header (global time, ms) minus the host time the frame reached librealsense (``time_of_arrival``,
    ms): negative while the mapping is healthy (the exposure precedes the arrival). States, each entered after
    ``sustain`` consecutive frames:

    - ``AHEAD`` (dangerous): the offset is above the start-up baseline by more than ``threshold_ms``, or above 0 (a
      header AFTER the frame's own arrival is never this host's time, whatever the baseline);
    - ``BEHIND`` (benign): below the baseline by more than ``threshold_ms`` (later arrivals or a mapping running
      behind: a capture may only be later than its trigger);
    - ``OK``: back within ``threshold_ms``. Its event carries the worst SIGNED departure of the episode.

    The baseline is the median of the first ``baseline_frames`` usable offsets; a non-finite arrival or a zero
    header is ignored.
    """

    def __init__(self, threshold_ms=THRESHOLD_MS, sustain=15, baseline_frames=30):
        self.threshold_ms, self.sustain, self.baseline_frames = threshold_ms, sustain, baseline_frames
        self.collected, self.baseline, self.state = [], None, 'OK'
        self.candidate, self.streak, self.worst_ms, self.ahead_events = None, 0, 0.0, 0

    def _classify(self, offset):
        departure = offset - self.baseline
        if departure > self.threshold_ms or offset > 0:
            return 'AHEAD', departure
        if departure < -self.threshold_ms:
            return 'BEHIND', departure
        return 'OK', departure

    def update(self, header_ns, arrival_ms):
        """``(state, details)`` when the state changes, else ``None``."""
        if header_ns <= 0 or not math.isfinite(arrival_ms):
            return None
        offset = header_ns / 1e6 - arrival_ms
        if self.baseline is None:
            self.collected.append(offset)
            if len(self.collected) >= self.baseline_frames:
                self.baseline = sorted(self.collected)[len(self.collected) // 2]
            return None
        state, departure = self._classify(offset)
        if self.state != 'OK' and abs(departure) > abs(self.worst_ms):
            self.worst_ms = departure
        if state == self.state:
            self.candidate, self.streak = None, 0
            return None
        self.streak = self.streak + 1 if state == self.candidate else 1
        self.candidate = state
        if self.streak < self.sustain:
            return None
        details = {'offset_ms': round(offset, 1), 'baseline_ms': round(self.baseline, 1),
                   'departure_ms': round(departure, 1)}
        if state == 'OK':
            details['worst_departure_ms'] = round(self.worst_ms, 1)
        else:
            self.worst_ms = departure
            self.ahead_events += state == 'AHEAD'
        self.state, self.candidate, self.streak = state, None, 0
        return state, details


def _parameter_value(value):
    return {1: value.bool_value, 2: value.integer_value, 3: value.double_value, 4: value.string_value}.get(value.type)


def _stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class RealSenseSourceInfo(Node):

    def __init__(self, **node_kwargs):
        super().__init__('realsense_source_info', **node_kwargs)
        self.declare_parameter('profile', 'realsense_d455')
        self.declare_parameter('namespace', '')             # the contract namespace (default: the profile's)
        self.declare_parameter('driver_name', 'driver')     # the driver node, inside the contract namespace
        self.declare_parameter('host_id', '')               # DECLARED logical id of this acquisition host
        self.declare_parameter('warmup_frames', 15)         # frames skipped before observing (start-up domains)
        self.declare_parameter('observe_frames', 30)        # frames per stream whose clock domain is observed
        self.declare_parameter('timeout_s', 30.0)           # ONE overall deadline for every step
        self.declare_parameter('service_wait_s', 5.0)       # at most this per service (an absent one costs no more)
        self.declare_parameter('dds_transport', '')         # the launch's DDS transport decision (JSON), if any
        prof = profiles.load('camera', self.get_parameter('profile').value)
        namespace = self.get_parameter('namespace').value or prof['topics']['namespace']
        self.namespace = '/' + namespace.strip('/')
        self.driver_name = self.get_parameter('driver_name').value
        self.driver = f'{self.namespace}/{self.driver_name}'
        self.host_id = self.get_parameter('host_id').value
        self.warmup = int(self.get_parameter('warmup_frames').value)
        self.observe = int(self.get_parameter('observe_frames').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.service_wait_s = float(self.get_parameter('service_wait_s').value)
        self.dds_transport = self._declared_transport(self.get_parameter('dds_transport').value)
        self.publisher = self.create_publisher(String, f'{self.namespace}/source_info', LATCHED_QOS)
        self.executor_ = SingleThreadedExecutor(context=self.context)    # this node's own: never the global one
        self.executor_.add_node(self)
        self.deadline = None
        self.document = None

    def _declared_transport(self, text):
        if not text:
            return None
        try:
            decision = json.loads(text)
        except ValueError:
            decision = None
        if not isinstance(decision, dict) or not decision.get('mode'):
            self.get_logger().warn(f'dds_transport parameter is not a transport decision: left out ({text!r})')
            return None
        return decision

    def _remaining(self):
        return max(0.0, self.deadline - time.monotonic())

    def _call(self, srv_type, name, request):
        client = self.create_client(srv_type, name)
        try:
            if not client.wait_for_service(timeout_sec=min(self.service_wait_s, self._remaining())):
                return None
            future = client.call_async(request)
            self.executor_.spin_until_future_complete(future, timeout_sec=self._remaining())
            return future.result() if future.done() else None
        finally:
            self.destroy_client(client)

    def _step(self, name, function, *args):
        """One discovery step: its block, or None (logged) -- a failed step never stops the document."""
        try:
            return function(*args)
        except ImportError as exc:
            self.get_logger().warn(f'{name}: {exc} (is ros-humble-realsense2-camera installed?): no {name} block')
        except Exception as exc:  # noqa: BLE001 -- unknown is absent
            self.get_logger().warn(f'{name}: {exc!r}: no {name} block')
        return None

    def device_block(self):
        from realsense2_camera_msgs.srv import DeviceInfo
        reply = self._call(DeviceInfo, f'{self.driver}/device_info', DeviceInfo.Request())
        if reply is None:
            self.get_logger().warn(f'{self.driver}/device_info did not answer: no device block')
            return None
        device = {'origin': 'DISCOVERED', 'via': f'{self.driver_name}/device_info (realsense2_camera)'}
        for key, field in (('name', 'device_name'), ('serial_number', 'serial_number'),
                           ('firmware_version', 'firmware_version'), ('usb_type_descriptor', 'usb_type_descriptor'),
                           ('sensors', 'sensors'), ('physical_port', 'physical_port')):
            value = getattr(reply, field, '')
            if value:
                device[key] = value
        return device

    def configuration_block(self):
        from rcl_interfaces.srv import GetParameters
        request = GetParameters.Request()
        request.names = list(CONFIG_PARAMETERS)
        reply = self._call(GetParameters, f'{self.driver}/get_parameters', request)
        if reply is None or len(reply.values) != len(request.names):
            self.get_logger().warn(f'{self.driver}/get_parameters did not answer every name: no configuration block')
            return None
        values = {n: _parameter_value(v) for n, v in zip(request.names, reply.values)}
        return {'origin': 'DISCOVERED', 'via': f'{self.driver_name} parameters',
                'parameters': {n: v for n, v in values.items() if v is not None}}

    def timestamping_block(self):
        from realsense2_camera_msgs.msg import Metadata
        from sensor_msgs.msg import Image
        metadata = {s: [] for s in STREAMS}
        headers = {s: [] for s in STREAMS}

        def on_metadata(stream):
            def callback(msg):
                try:
                    data = json.loads(msg.json_data)
                    entry = (_stamp_ns(msg.header), data.get('clock_domain'), data.get('frame_timestamp'))
                except (ValueError, AttributeError):
                    entry = (_stamp_ns(msg.header), None, None)
                metadata[stream].append(entry)
            return callback
        subs = [self.create_subscription(Metadata, f'{self.driver}/{s}/metadata', on_metadata(s),
                                         qos_profile_sensor_data) for s in STREAMS]
        subs += [self.create_subscription(Image, f'{self.namespace}/{t}', lambda m, s=s: headers[s].append(
            _stamp_ns(m.header)), qos_profile_sensor_data) for s, t in CONTRACT_IMAGES.items()]
        need = self.warmup + self.observe
        try:
            while self._remaining() > 0 and (any(len(v) < need for v in metadata.values())
                                             or any(len(v) < need for v in headers.values())):
                self.executor_.spin_once(timeout_sec=min(0.1, self._remaining()))
        finally:
            for sub in subs:
                self.destroy_subscription(sub)
        if not any(metadata.values()):
            self.get_logger().warn(f'no frame metadata on {self.driver}/<stream>/metadata: no timestamping block')
            return None
        return evaluate_timestamping(metadata, headers, warmup=self.warmup, observe=self.observe,
                                     via=f'{self.driver_name}/<stream>/metadata + the contract image headers')

    def run(self):
        self.deadline = time.monotonic() + self.timeout_s
        device = self._step('device', self.device_block)
        attachment = self._step('attachment', source_info.usb_attachment, (device or {}).pop('physical_port', None))
        maps = self._step('library', driver_process_maps, self.namespace, self.driver_name)
        library = source_info.library_in_process(maps) if maps else None
        deployment = {'origin': 'DECLARED', 'namespace': self.namespace}
        if self.host_id:
            deployment['host_id'] = self.host_id
        if self.dds_transport is not None:
            deployment['dds_transport'] = self.dds_transport
        driver_version = _package_version('realsense2_camera')
        software = None if driver_version is None else {
            'origin': 'DISCOVERED', 'via': 'ament index of the acquisition host',
            'driver': {'package': 'realsense2_camera', 'version': driver_version}}
        configuration = self._step('configuration', self.configuration_block)
        timestamping = self._step('timestamping', self.timestamping_block)
        self.document = source_info.build(
            backend='real', published_wall_utc_ns=time.time_ns(), device=device, attachment=attachment,
            host=source_info.host_facts(), deployment=deployment, software=software,
            library=None if library is None else dict(library, origin='OBSERVED', via='driver process memory map'),
            configuration=configuration, timestamping=timestamping)
        self.publisher.publish(String(data=json.dumps(self.document, sort_keys=True)))
        d, a, t = self.document.get('device', {}), self.document.get('attachment') or {}, \
            self.document.get('timestamping', {})
        self.get_logger().info(
            f'source_info on {self.namespace}/source_info: {d.get("name")} serial {d.get("serial_number")} fw '
            f'{d.get("firmware_version")}, USB {a.get("port_id")} {a.get("speed_mbps")} Mb/s '
            f'{"via hub " + str(a["hub"].get("vendor_product")) if a.get("hub") else "root port" if a else "?"}, '
            f'clock domains {t.get("clock_domains")}, stamp origin {t.get("stamp_origin")}')
        self._warn_on_loss(t.get('contract_delivery'))
        return self.document

    def watch_clock(self):
        """Keep comparing the colour headers with their host arrival times (``ClockWatch``); log the changes."""
        try:
            from realsense2_camera_msgs.msg import Metadata
        except ImportError:
            self.get_logger().warn('no realsense2_camera_msgs: the driver clock mapping is not watched')
            return
        self.clock_watch = ClockWatch()

        def on_metadata(msg):
            try:                                            # a callback never raises (it would stop spin())
                arrival = json.loads(msg.json_data).get('time_of_arrival')
                if not isinstance(arrival, (int, float)) or isinstance(arrival, bool):
                    return
                event = self.clock_watch.update(_stamp_ns(msg.header), float(arrival))
                if event:
                    self._log_clock_event(*event)
            except Exception:  # noqa: BLE001
                return
        self._clock_subscription = self.create_subscription(Metadata, f'{self.driver}/color/metadata', on_metadata,
                                                            qos_profile_sensor_data)

    def _log_clock_event(self, state, details):
        where = (f'{self.namespace}: header - host arrival = {details["offset_ms"]} ms, '
                 f'{details["departure_ms"]:+} ms from start-up ({details["baseline_ms"]} ms)')
        if state == 'AHEAD':
            self.get_logger().warn(
                f'{where}: the driver\'s device-clock mapping runs AHEAD of this host: stamps are not this host\'s '
                'time; a same-host recorder refuses frames stamped after their receipt (CLOCK_MISMATCH), smaller '
                'departures pass unnoticed by it')
        elif state == 'BEHIND':
            self.get_logger().info(f'{where}: stamps run behind their usual offset (later arrivals or the mapping '
                                   'behind): captures may be later than their trigger')
        else:
            self.get_logger().info(f'{where}: the driver\'s device-clock mapping is back within '
                                   f'{self.clock_watch.threshold_ms:g} ms of start-up (worst departure '
                                   f'{details["worst_departure_ms"]:+} ms)')

    def _warn_on_loss(self, delivery):
        for stream in STREAMS:
            entry = (delivery or {}).get(stream) or {}
            if entry.get('fraction') is not None and entry['fraction'] < DELIVERY_WARNING:
                self.get_logger().warn(
                    f'{self.namespace}: only {entry["received"]} of {entry["driver_framesets"]} {stream} framesets '
                    'reached a subscriber ON THIS HOST at start-up: images are lost in transport or not produced on '
                    f'the contract (deployment.dds_transport {(self.dds_transport or {}).get("mode")}; '
                    'contracts/camera.md, "Large images on one host")')


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseSourceInfo()
    # Ctrl+C in a terminal reaches this process twice (from the terminal and from ros2 launch). The first one stops
    # the node; a second one must not end it "killed by SIGINT" (exit code -2) wherever it lands.
    try:
        try:
            node.run()
            node.watch_clock()
            node.executor_.spin()                           # keep the latched document available
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            node.destroy_node()
            rclpy.try_shutdown()
            signal.signal(signal.SIGINT, signal.SIG_IGN)   # try_shutdown restores rclpy's previous handler
    except KeyboardInterrupt:
        pass                                                # the second one landed before the guard: still a stop


if __name__ == '__main__':
    main()
