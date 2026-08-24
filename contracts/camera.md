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
The **ROS2 Camera Helper** OmniGraph node, wired to the render product of the camera
prim. **No ROS-side node is needed** — unlike the gripper, where Isaac cannot speak a
service contract, the helper publishes the contract topics directly. Configure:

| Helper setting | Value |
|---|---|
| `type` | `rgb`, then a second helper with `depth`, and one with `depth_pcl` |
| `topicName` | the contract name above |
| `frameId` | the matching **optical** frame |
| QoS | `BEST_EFFORT` for image and points |

If a shim ever turns out to be necessary, it belongs in `adapters/camera/` — but do not
write one before something actually requires it.

### `real`
`realsense2_camera` in the Isaac ROS container on the Jetson, remapped onto the contract
names. The URDF still comes from `realsense2_description` here, so the frame tree matches
simulation exactly and a hand-eye calibration transfers without re-derivation.
