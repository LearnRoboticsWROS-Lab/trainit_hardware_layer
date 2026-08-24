#!/usr/bin/env python3
"""Mock camera adapter -- realises contracts/camera.md with no simulator and no device.

It publishes a synthetic, static scene so the perception pipeline can be wired, launched
and debugged without a GPU or hardware. It is NOT a renderer and will never become one:
if you need images that mean something, use ``mode:=isaac``.

What it IS good for: proving the contract is satisfied -- topic names, encodings, frame
ids, QoS. Most vision-pipeline wiring bugs are exactly those, and they are far cheaper
to find here than against a running simulator.

QoS matches the contract: BEST_EFFORT for image and points, RELIABLE for camera_info.
Getting that wrong is silent -- the subscription simply never matches.
"""
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from trainit_hardware_layer import profiles

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=5)
INFO_QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=5)


class MockCamera(Node):

    def __init__(self):
        super().__init__('mock_camera')
        self.declare_parameter('profile', 'realsense_d455')
        self.declare_parameter('namespace', '')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('plane_distance_m', 1.0)

        prof = profiles.load('camera', self.get_parameter('profile').value)
        ns = self.get_parameter('namespace').value or prof['topics']['namespace']
        colour, depth = prof['sensor']['color'], prof['sensor']['depth']

        self.w, self.h = int(colour['width']), int(colour['height'])
        self.color_frame = prof['frames']['color_optical']
        self.depth_frame = prof['frames']['depth_optical']
        self.dist_mm = int(self.get_parameter('plane_distance_m').value * 1000)
        self.fx = self.fy = (self.w / 2.0) / math.tan(math.radians(colour['hfov_deg']) / 2.0)
        self.cx, self.cy = self.w / 2.0, self.h / 2.0

        self.pub_rgb = self.create_publisher(Image, f'{ns}/color/image_raw', SENSOR_QOS)
        self.pub_depth = self.create_publisher(Image, f'{ns}/depth/image_rect_raw', SENSOR_QOS)
        self.pub_pts = self.create_publisher(PointCloud2, f'{ns}/depth/color/points', SENSOR_QOS)
        self.pub_ci_c = self.create_publisher(CameraInfo, f'{ns}/color/camera_info', INFO_QOS)
        self.pub_ci_d = self.create_publisher(CameraInfo, f'{ns}/depth/camera_info', INFO_QOS)

        self._rgb = bytes(bytearray([90, 110, 130] * (self.w * self.h)))
        self._depth = struct.pack('<H', self.dist_mm) * (self.w * self.h)
        self._cloud = self._build_cloud()

        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        self.get_logger().info(
            f"mock camera on '{ns}' -- {self.w}x{self.h}, flat plane at "
            f"{self.dist_mm/1000:.2f} m, frames {self.color_frame} / {self.depth_frame}. "
            f"Synthetic: proves the wiring, not the scene.")

    def _build_cloud(self) -> bytes:
        """A subsampled flat plane at plane_distance, in OPTICAL axes (+Z forward)."""
        z = self.dist_mm / 1000.0
        out = bytearray()
        for v in range(0, self.h, 8):
            for u in range(0, self.w, 8):
                out += struct.pack('<fff', (u - self.cx) * z / self.fx,
                                   (v - self.cy) * z / self.fy, z)
                out += struct.pack('<BBBB', 130, 110, 90, 0)
        return bytes(out)

    def _info(self, stamp, frame) -> CameraInfo:
        ci = CameraInfo()
        ci.header.stamp, ci.header.frame_id = stamp, frame
        ci.width, ci.height, ci.distortion_model = self.w, self.h, 'plumb_bob'
        ci.d = [0.0] * 5
        ci.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def _tick(self):
        now = self.get_clock().now().to_msg()

        rgb = Image(height=self.h, width=self.w, encoding='rgb8',
                    is_bigendian=0, step=self.w * 3, data=self._rgb)
        rgb.header.stamp, rgb.header.frame_id = now, self.color_frame
        self.pub_rgb.publish(rgb)

        d = Image(height=self.h, width=self.w, encoding='16UC1',
                  is_bigendian=0, step=self.w * 2, data=self._depth)
        d.header.stamp, d.header.frame_id = now, self.depth_frame
        self.pub_depth.publish(d)

        n = len(self._cloud) // 16
        pc = PointCloud2(
            height=1, width=n, is_dense=True, is_bigendian=False,
            point_step=16, row_step=16 * n, data=self._cloud,
            fields=[PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)])
        pc.header.stamp, pc.header.frame_id = now, self.depth_frame
        self.pub_pts.publish(pc)

        self.pub_ci_c.publish(self._info(now, self.color_frame))
        self.pub_ci_d.publish(self._info(now, self.depth_frame))


def main(args=None):
    rclpy.init(args=args)
    node = MockCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
