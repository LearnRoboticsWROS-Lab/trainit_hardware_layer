# Contract — camera (RGB-D)

Modelled on the `realsense2_camera` wrapper, because the real deployment runs it. The
names below are **the contract**: every backend remaps onto them, including the real
driver. Do not "use whatever the driver publishes" — driver defaults differ between
versions and between the apt package and the Isaac ROS container image.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `<ns>/color/image_raw` | `sensor_msgs/Image` | `rgb8` |
| `<ns>/color/camera_info` | `sensor_msgs/CameraInfo` | intrinsics; `frame_id` = the colour **optical** frame |
| `<ns>/depth/image_rect_raw` | `sensor_msgs/Image` | `16UC1`, **millimetres** — the RealSense convention |
| `<ns>/depth/camera_info` | `sensor_msgs/CameraInfo` | |
| `<ns>/depth/color/points` | `sensor_msgs/PointCloud2` | organised, registered to colour |

`<ns>` defaults to `/camera`. A cell with two cameras gives each its own namespace and
its own frame prefix; nothing else changes.

## QoS — the one that wastes an afternoon

Image and point-cloud topics are **`BEST_EFFORT`**. If one side is `RELIABLE` and the
other `BEST_EFFORT` the subscription silently never matches: no error, no data, and it
looks like a network fault. It is not. `camera_info` is `RELIABLE`.

This bites hardest in `real` mode, where the camera is on a **different machine** (a
Jetson): the two sides must also share `ROS_DOMAIN_ID` and a synchronised clock, or TF
lookups fail with times that make no sense.

## Frames

Taken from `realsense2_description`, which both simulation and hardware use, so the
tree is identical on both:

```
<prefix>_bottom_screw_frame     tripod mount — what the URDF joint attaches to
  └── <prefix>_link             body frame, REP-103: +X forward, +Y left, +Z up
        ├── <prefix>_color_frame
        │     └── <prefix>_color_optical_frame     rpy = -pi/2, 0, -pi/2
        └── <prefix>_depth_frame
              └── <prefix>_depth_optical_frame     rpy = -pi/2, 0, -pi/2
```

**Optical frames are not body frames.** `_link` is REP-103; `_optical_frame` is
+X right, +Y **down**, +Z forward. Every `CameraInfo.frame_id` and every point cloud
`frame_id` is an *optical* frame. Publishing a cloud in `_link` puts it 90 degrees out
and looks exactly like a calibration error.

Two facts about the official macro that are easy to miss:

- `sensor_d455` attaches the caller's `<origin>` to `_bottom_screw_frame`, **not** to
  `_link`. `_link` sits at `(0.01115, 0.0475, 0.0145)` from it.
- `use_nominal_extrinsics` defaults to **`false`**, and with `false` the optical frames
  **are not created at all**. In simulation it must be `true` — the macro's own comment
  says so.

## Per backend

### `mock`
`trainit_hardware_layer`'s `mock_camera` node. Publishes a synthetic, static scene at a
low rate with a plausible `CameraInfo`. Enough to prove the pipeline is wired; it is not
a renderer and never will be.

### `isaac`
The **ROS2 Camera Helper** OmniGraph nodes publish the image topics directly. Note that
`camera_info` is *not* in the Camera Helper's `type` list: since Isaac 4.x it is a
separate **ROS2 Camera Info Helper** node, because it also has to serve stereo pairs.

| node | `type` | `topicName` |
|---|---|---|
| Camera Helper | `rgb` | `<ns>/color/image_raw` |
| Camera Helper | `depth` | `<ns>/depth/image_rect_raw` |
| Camera Info Helper | — | `<ns>/color/camera_info` |

Every one of them needs `execIn` from the tick, `renderProductPath` from the render
product, and **`context` from the ROS2 Context node**. Forget the context wire and the
nodes run happily, publish into a different ROS context, and `ros2 topic list` shows
nothing — with no error.

**The cloud is the exception, and it matters.** Isaac's `depth_pcl` helper emits **XYZ
only**, while the real `realsense2_camera` publishes `<ns>/depth/color/points` as
**XYZRGB**. Same topic name, different content: perception written against the simulator
would break on hardware, which is the exact failure this contract exists to prevent.

So in simulation, **do not use the `depth_pcl` helper**. Build the cloud the way the real
driver does, from the depth image plus the colour image plus the intrinsics:

```bash
ros2 component standalone depth_image_proc depth_image_proc::PointCloudXyzrgbNode \
  --node-name pcl_xyzrgb \
  -r rgb/image_rect_color:=/camera/color/image_raw \
  -r rgb/camera_info:=/camera/color/camera_info \
  -r depth_registered/image_rect:=/camera/depth/image_rect_raw \
  -r points:=/camera/depth/color/points
```

That is not a workaround: `depth_image_proc` is the standard ROS component, and the same
fusion happens inside the vendor driver. It also settles the depth-encoding question —
it accepts both `32FC1` and `16UC1`, so nothing downstream has to care.

**The rule that follows: perception reads the CLOUD, never the raw depth image.** With
that, the encoding difference between simulator and hardware stops being anybody's
problem.

`depth_registered` in the remap is literal, not aspirational: the depth must already be
registered to the colour frame. In simulation it is, because both come from one camera
prim. On hardware the driver aligns them when `align_depth.enable` is set.

If a further shim ever turns out to be necessary, it belongs in `adapters/camera/` — but
do not write one before something requires it.

### `real`
`realsense2_camera` in the Isaac ROS container on the Jetson, remapped onto the contract
names. With `pointcloud.enable:=true` it publishes `<ns>/depth/color/points` as XYZRGB
itself, so **no `depth_image_proc` is needed there** — the simulator adds it precisely to
match what the driver already does.

The URDF still comes from `realsense2_description` here, so the frame tree matches
simulation exactly and a hand-eye calibration transfers without re-derivation.

> **Unverified.** The driver's behaviour above is from its documentation; it was not
> measured, because `realsense2_camera` is not installed on the development machine.
> Confirm it against the actual Jetson container before relying on it — and if the
> container's defaults differ, remap there rather than changing the contract.
