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
| `<ns>/depth/image_rect_raw` | `sensor_msgs/Image` | depth **registered to colour** (see "Registered depth"): `frame_id` = the colour optical frame. `16UC1` **millimetres** on the device (the RealSense convention); `32FC1` metres in Isaac — the encoding states the unit (REP-118) |
| `<ns>/depth/camera_info` | `sensor_msgs/CameraInfo` | geometrically **identical** to `<ns>/color/camera_info` (registered depth lives on the colour pixel grid) |
| `<ns>/depth/color/points` | `sensor_msgs/PointCloud2` | **optional**: organised, registered to colour (`frame_id` = the colour optical frame). A backend publishes this conformant cloud or none -- never another cloud under this name (`real`: none yet, D0.4 debt) |
| `<ns>/source_info` | `std_msgs/String` | **optional**, latched: one `trainit-camera-source/1.0` JSON document (see "Source information") |

`<ns>` defaults to `/camera`. A cell with two cameras gives each its own namespace and
its own frame prefix; nothing else changes.

## Registered depth (normative; clarified 2026-10-28)

The contract's depth topics carry depth **registered to the colour stream**. For those topics:

1. **Geometry.** Every depth pixel `(u, v)` is the depth of colour pixel `(u, v)`: the depth image has
   the colour image's width and height, and its geometry is expressed in the **colour optical frame**.
2. **`header.frame_id`** of `<ns>/depth/image_rect_raw` and of `<ns>/depth/camera_info` is therefore the
   **colour optical frame** (`<prefix>_color_optical_frame`), never `<prefix>_depth_optical_frame`.
3. **`<ns>/depth/camera_info` is geometrically identical to `<ns>/color/camera_info`** for the same
   moment: same `width`, `height`, `distortion_model`, `d`, `k`, `r`, `p`, `binning`, `roi` and
   `frame_id`. Only the stamp may differ.
4. **Encoding and unit are unchanged and explicit.** The image `encoding` states the unit (REP-118):
   `16UC1` = millimetres, `32FC1` = metres. A consumer reads the encoding; it never assumes one. `0`
   (`16UC1`) or NaN / inf (`32FC1`) means "no measurement".
5. **Raw depth never masquerades as registered depth.** A backend that has only unregistered depth (the
   depth sensor's own optical frame and intrinsics) must NOT publish it on these names. It registers it
   first (the driver's `align_depth`, or `depth_image_proc`'s register component), or it is not
   conforming. A depth image whose `frame_id` is not the colour optical frame is a contract violation,
   not a variant to be corrected downstream.

`<prefix>_depth_optical_frame` stays in the TF tree (it is the description's, see "Frames"): it locates
the physical depth sensor. No contract topic is published in it.

This clause states what the reference simulation already does (one render product: every image topic in the
colour optical frame, measured; it does not yet publish `<ns>/depth/camera_info`, see `isaac` below) and what
the real driver provides once its ALIGNED depth is remapped onto the contract (see `real` below; measured on a
D455 on 2026-10-29, D0.4). The `mock` backend conforms to it.

## QoS — the one that wastes an afternoon

Image and point-cloud topics are **`BEST_EFFORT`**. If one side is `RELIABLE` and the
other `BEST_EFFORT` the subscription silently never matches: no error, no data, and it
looks like a network fault. It is not. `camera_info` is `RELIABLE`.

This bites hardest when the camera is on a **different machine** than its consumer (an
edge computer, "Deployment topology" B): the two sides must also share `ROS_DOMAIN_ID` and a
synchronised clock, or TF lookups fail with times that make no sense.

## Large images on one host (DDS transport; added 2026-10-30, D0.4)

**Invariant (every backend):** a subscriber on the publisher's own host receives every frameset the backend
produces. `real` meets it and measures it (below); `isaac` is not yet verified against it (debt).

BEST_EFFORT images can also be lost **without any QoS mismatch**, on ONE host. ROS 2 Humble's default middleware,
Fast DDS 2.6, carries data between processes of one host through shared memory, in a segment of **512 KiB per
participant** by default. A colour frame of 848x480 is 1.2 MB and an aligned depth frame 0.8 MB: when the
publisher's segment is still busy, the next image is dropped, silently. Measured on the development workstation
(2026-10-30, D455, 848x480 at 30 fps): the driver produced 30 depth frames/s, a subscriber on the same host received
**5.7 to 11/s with gaps up to 0.67 s**; with the driver's segment sized for 16 framesets it received **30.0/s, no
gap** (colour alike). The subscriber needs nothing: the PUBLISHER's segment is what matters.

Rules:
- **A camera backend that publishes large images on Fast DDS sizes its own publisher's shared-memory segment** for
  the images it publishes. The real backend does it in `launch/realsense_camera.launch.py` (`dds_transport:=auto`,
  the default; `camera/dds_transport.py`): the DRIVER process alone gets a generated profile, with the segment
  computed from the hardware profile (colour + aligned depth, + the point cloud when enabled, x 16 framesets, rounded
  up to MiB: 32 MiB at 848x480) and capped at 1/4 of the host's `/dev/shm` (a container's default `/dev/shm` is
  64 MiB: give the container more, or `--ipc=host`, when the cap is reported; the cap uses the TOTAL size of
  `/dev/shm`, not what is free). UDPv4 stays on for other hosts. Verified live with and without
  `ROS_LOCALHOST_ONLY=1` (2026-10-30: 600/600 and 900/900 framesets, no gap).
- The profile variable is `FASTRTPS_DEFAULT_PROFILES_FILE` (Fast DDS 2.x: Humble, Jazzy -- Jazzy also reads
  `FASTDDS_DEFAULT_PROFILES_FILE` and warns that the old name is deprecated). Re-check the generated XML before
  supporting a distribution on Fast DDS 3. A driver killed with SIGKILL leaves its segment in `/dev/shm`; Fast DDS's
  `fastdds shm clean` removes such leftovers.
- **Nothing on the host is changed** (no sysctl, no global profile): the profile is a content-addressed file under
  `$ROS_HOME/trainit/fastdds` (`~/.ros` by default) and only the driver's environment points at it.
- **A user's choice wins.** A Fast DDS profile already set in `FASTRTPS_DEFAULT_PROFILES_FILE` /
  `FASTDDS_DEFAULT_PROFILES_FILE` is kept (its segment must then hold the framesets); another middleware
  (`RMW_IMPLEMENTATION`, e.g. Cyclone DDS) is left alone and needs its own large-message configuration.
- **The decision and its effect are recorded.** `<ns>/source_info` carries the decision (`deployment.dds_transport`:
  `APPLIED` with the segment size, `USER_PROFILE`, `OTHER_RMW`, `OFF` or `NOT_APPLIED`). It is DECLARED in the
  sense of the deployment block: what the launch CONFIGURED for the driver (with the facts it based that on: the
  middleware, the frameset size, the `/dev/shm` cap), not a verification. The verification is OBSERVED:
  `timestamping.contract_delivery`, the framesets the driver produced vs received by the source node's own
  subscriber on the acquisition host, in the start-up window (it lives in `timestamping` because the same window and
  the same reference stamps establish both; a lost metadata message makes it slightly optimistic; it is not what a
  recorder on another process or host received -- the recorder counts its own captures). Below 95 % the source
  node logs a warning.
- **Across hosts** (topology B) images travel over UDP; large best-effort messages then depend on the hosts' socket
  buffers and the network. That deployment is measured and configured when it is accepted, not assumed here.
- `isaac` publishes from Isaac Sim's own ROS bridge (its own Fast DDS participant): its segment is configured in the
  simulator's environment, not by this launch (known debt, D0.4).

## Source information (optional; added 2026-10-29, D0.4)

A backend MAY publish, on `<ns>/source_info`, ONE `trainit-camera-source/1.0` JSON document as a `std_msgs/String`,
**RELIABLE + TRANSIENT_LOCAL, depth 1** (latched: a consumer that starts later still receives it). It answers what the
image topics cannot: which physical device produced the frames, where it was attached, which host acquired them,
with which software and configuration, and how the stamps are produced. The schema and its validator live in
`trainit_hardware_layer/camera/source_info.py`.

Every block names the ONE origin of its facts:

| Block | Origin | Facts |
|---|---|---|
| `device` | DISCOVERED | name, serial number, firmware (from the driver's own device service) |
| `attachment` | DISCOVERED | `kind: USB`, port id, bus, negotiated `speed_mbps`, `usb_version`; `hub` (or `null` for a root port) with the self-powered bit AS THE HUB REPORTS IT |
| `host` | DISCOVERED | hostname, machine, kernel, `boot_id_sha256` (the acquisition host) |
| `deployment` | DECLARED | the contract namespace, an optional logical `host_id`, the launch's `dds_transport` decision for the driver ("Large images on one host") |
| `software` | DISCOVERED | the driver package and version (the acquisition host's ament index) |
| `library` | OBSERVED | the vendor library the RUNNING driver process has mapped |
| `configuration` | DISCOVERED | the driver's effective parameters (profiles, formats, alignment, global time, QoS) |
| `timestamping` | OBSERVED | the per-stream clock domain of live frames after a warm-up, and the `stamp_origin` they establish; `contract_delivery`: framesets the driver produced vs received by a subscriber on the acquisition host, in the same window |

Rules:
- **Unknown is absent.** A fact that cannot be established is left out; nothing is guessed, nothing is read from a
  topic name.
- **No addresses.** Exact IP / MAC addresses and machine ids are not part of the document. `boot_id_sha256` (a hash
  of the per-boot random id) only lets a consumer tell "same host, this boot" from "another host".
- `timestamping.stamp_origin` is present only when EVERY observed stream's clock domain maps to the same origin
  (`global_time` → `DRIVER_GLOBAL_TIME`, `system_time` → `RECEIPT`); the validator refuses any other claim.
- The document describes the ACQUISITION side. A consumer (the D0 recorder) adds its own host and transport facts
  and compares them; it never talks to the device.

`mock` and `isaac` do not publish it today; `real` does (`launch/realsense_camera.launch.py`).

## Deployment topology (D0.4)

The contract is the same wherever its publisher runs. A physical camera may be deployed as:

```
A  D455 --USB--> workstation [realsense2_camera + recorder]                         (D0-C acceptance topology)
B  D455 --USB--> edge computer [realsense2_camera] --ROS 2 / DDS, Ethernet--> workstation [recorder]
C  D455 --USB--> edge computer [realsense2_camera + recorder -> local TrainIt store]
```

- The **data plane** is the contract over ROS 2 / DDS (live) and TrainIt sessions (persisted). **SSH is the
  management plane only** (configure, launch, stop, inspect a remote acquisition runtime); camera payloads are never
  streamed over it.
- The same launch runs on every host; whether the recorder is on the same host is a deployment fact, recorded by the
  recorder from evidence (`LOCAL_SENSOR_LOCAL_RECORDER` / `REMOTE_SENSOR_SOURCE` / `UNKNOWN`), never encoded in a
  backend, a topic name or a data schema.
- Trade-off: a recorder next to the sensor (C) does not depend on network delivery of ~2 MB framesets; a recorder on
  the workstation (B) needs the two hosts on one `ROS_DOMAIN_ID`, a matching DDS configuration, and synchronised
  clocks (a header stamped on one host is compared with the recorder's clock on another).

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
a renderer and never will be. Its depth is registered by construction (one synthetic pixel grid):
every topic is in the colour optical frame and the two `camera_info`s are identical.

**Source descriptor.** A backend entry of a camera profile may carry a `source_descriptor`: the facts a
consumer that records data needs as provenance and must not guess from a device. This layer declares
them; it stores no recording. Keys (plain strings):

| Key | Meaning | `mock` |
|---|---|---|
| `backend` | the backend mode this entry describes | `mock` |
| `intrinsics_kind` | where the published CameraInfo comes from | `NOMINAL_PROFILE`: computed from this profile's nominal field of view |
| `stamp_origin` | what the image header stamp is | `HEADER`: the node clock when the frame is published |
| `registration_basis` | why depth is registered to colour | `same_pixel_grid`: depth synthesised on the colour grid |
| `sim_source` | (simulators only) the simulator whose clock stamps the frames | — |
| `intrinsics_notes` | a fact about the published intrinsics a consumer must know | — |

`isaac` declares `SYNTHETIC_RENDER`, `HEADER`, `same_render_product`, `sim_source: isaac` and, as
`intrinsics_notes`, Isaac's pixel-edge principal point (the rendered point lies 0.5 px up-left of the
CameraInfo projection).

### `isaac`
The **ROS2 Camera Helper** OmniGraph nodes publish the image topics directly. Note that
`camera_info` is *not* in the Camera Helper's `type` list: since Isaac 4.x it is a
separate **ROS2 Camera Info Helper** node, because it also has to serve stereo pairs.

| node | `type` | `topicName` |
|---|---|---|
| Camera Helper | `rgb` | `<ns>/color/image_raw` |
| Camera Helper | `depth` | `<ns>/depth/image_rect_raw` |
| Camera Info Helper | — | `<ns>/color/camera_info` |

`<ns>/depth/camera_info` has no helper row: the reference cell does not publish it today. Under
"Registered depth" it is identical to the colour info, so a consumer that needs it may read
`<ns>/color/camera_info`; publishing it from the simulator is part of the simulator acquisition work.

Every one of them needs `execIn` from the tick, `renderProductPath` from the render
product, and **`context` from the ROS2 Context node**. Forget the context wire and the
nodes run happily, publish into a different ROS context, and `ros2 topic list` shows
nothing — with no error.

**The cloud is the exception, and it matters.** Isaac's `depth_pcl` helper emits **XYZ
only**, while the contract's cloud is **XYZRGB**, organised and registered to colour.
Same topic name, different content: perception written against the simulator
would break on hardware, which is the exact failure this contract exists to prevent.

So in simulation, **do not use the `depth_pcl` helper**. Build the cloud from the contract's
depth image plus the colour image plus the intrinsics:

```bash
ros2 component standalone depth_image_proc depth_image_proc::PointCloudXyzrgbNode \
  --node-name pcl_xyzrgb \
  -r rgb/image_rect_color:=/camera/color/image_raw \
  -r rgb/camera_info:=/camera/color/camera_info \
  -r depth_registered/image_rect:=/camera/depth/image_rect_raw \
  -r points:=/camera/depth/color/points
```

That is not a workaround: `depth_image_proc` is the standard ROS component, and the same
recipe builds the contract cloud on the real backend (the vendor driver's own cloud is
unorganised, in the depth optical frame: see `real`). It also settles the depth-encoding question —
it accepts both `32FC1` and `16UC1`, so nothing downstream has to care.

**The rule that follows (corrected 2026-10-29, D0 decision D14): no consumer reads RAW (unregistered) depth.**
Collision / octomap consumers read the CLOUD. Perception and data recording may read the REGISTERED depth image
(`<ns>/depth/image_rect_raw` under "Registered depth"); they read its `encoding` for the unit (REP-118) and never
assume one, so the encoding difference between simulator and hardware is still nobody's problem.

`depth_registered` in the remap is literal, not aspirational: the depth must already be
registered to the colour frame. In simulation it is, because both come from one camera
prim. On hardware the driver aligns them when `align_depth.enable` is set.

If a further shim ever turns out to be necessary, it belongs in `adapters/camera/` — but
do not write one before something requires it.

### `real`
`realsense2_camera` on the host the camera is plugged into, started and remapped onto the contract by
`launch/realsense_camera.launch.py` (beside `realsense_source_info`, which publishes `<ns>/source_info`). The driver
runs as `<ns>/driver`; only colour and the ALIGNED depth leave that namespace:

| Driver topic (`<ns>/driver/...`) | Contract name |
|---|---|
| `color/image_raw`, `color/camera_info` | `<ns>/color/image_raw`, `<ns>/color/camera_info` |
| `aligned_depth_to_color/image_raw` | `<ns>/depth/image_rect_raw` |
| `aligned_depth_to_color/camera_info` | `<ns>/depth/camera_info` |

The driver's own `depth/image_rect_raw` is RAW depth in `<prefix>_depth_optical_frame` and stays inside
`<ns>/driver`: it never reaches a contract name ("Registered depth", clause 5).

The launch states every stream (colour + depth only; the driver's defaults also open both infrared streams and the
IMU), `align_depth.enable: true`, global time on for both modules, image QoS `SENSOR_DATA` (BEST_EFFORT) and
`publish_tf: false` (the description owns the frames: the cell's, or with `standalone_description:=true` the
camera's own nominal tree rooted at `<prefix>_mount`, `urdf/d455_standalone.urdf.xacro`).

**Source descriptor:** `DEVICE_REPORTED` (the device's factory calibration as the driver's CameraInfo reports it),
`DRIVER_GLOBAL_TIME`, `driver_align_depth`. A recorder checks the stamp origin against the OBSERVED clock domain in
`<ns>/source_info` and refuses a session they contradict.

**Measured on the development workstation (2026-10-29, D455 FW 5.16.0.1, `ros-humble-realsense2-camera` 4.58.4,
librealsense 2.58.4, kernel 6.8 stock `uvcvideo`, USB 5000 Mb/s behind a hub):**
- colour `rgb8` 848x480, aligned depth `16UC1` 848x480, both in `camera_color_optical_frame`;
  `<ns>/depth/camera_info` is identical to `<ns>/color/camera_info` (`plumb_bob`, non-zero D: the device's model);
- frame metadata reaches the driver on the stock kernel: clock domain `global_time` on both streams; one header stamp
  per frameset (colour and aligned depth share it); the header equals the metadata `frame_timestamp` (global time)
  within 0.3 us; the very first frames after start are `hardware_clock` (the driver warns once) -- the source document
  skips a warm-up before observing;
- depth `0` = no measurement (about 40 % of a desk scene); the device also emits `65535` on a few hundred pixels
  (also in the RAW depth, so not an alignment artefact): a raw value beyond the profile's `range_m`, kept as is;
  consumers clip with the profile's range;
- the device's depth units are 0.001 m (librealsense option, default and current); the driver exposes no
  parameter for it; 16UC1 = millimetres is the contract (REP-118).

**Point cloud (measured 2026-10-30): the driver's own cloud is NOT the contract's.** With `pointcloud:=true` the
driver publishes XYZRGB (16-byte points) at the frame rate, but UNORGANISED (`height: 1`, valid points only) and in
`<prefix>_depth_optical_frame` -- the contract's `<ns>/depth/color/points` is organised and registered to colour. It
therefore stays at `<ns>/driver/depth/color/points` (like the raw depth) and nothing is published on the contract's
cloud name by this launch. The contract cloud is built the same way on every backend: `depth_image_proc`
`PointCloudXyzrgbNode` from the contract's colour, aligned depth and CameraInfo (see `isaac`). Wiring it into this
launch is known debt (D0.4); no D0 consumer reads the cloud.

**Global time is a library FIT, and it can run away (measured 2026-10-30, D0-C).** librealsense maps the device
clock onto the host clock by fitting periodic clock samples (`time_diff_keeper`). Once in a session it ran AHEAD of
the host by up to 0.53 s (about 15 ms per second, for about a minute) until the library logged `clock fit re-based
from rejected-sample window after 60 consecutive innovation-gate rejections`; the host clock was verified stable. A
45-minute run did not reproduce it (healthy: header minus host arrival steady at about -27.5 ms, departures up to
10.4 ms). Two guards, neither widening a tolerance:
- `realsense_source_info` keeps watching header minus `time_of_arrival` after start-up and logs `AHEAD` (dangerous:
  departure above 20 ms, or a header after its own arrival), `BEHIND` (benign) and the recovery;
- a recorder on the SAME host (relation proven by the boot-id evidence, ROS time) refuses a frame stamped after its
  own receipt: `CLOCK_MISMATCH`, and the session warns `STAMPS_AFTER_RECEIPT`. A departure smaller than the frame's
  own latency (about 33 ms) passes it: the watch's log is the earlier signal.

**Large images:** the launch sizes the driver's Fast DDS shared-memory segment ("Large images on one host"); without
it, a same-host subscriber received 5.7 to 11 of the 30 depth frames/s (2026-10-30). The launch log states the
decision (`DDS transport: ...`).

**Do not enable every stream on a hub.** With the driver's default streams (+ both infrared + IMU) the D455 dropped off
the bus ten times in 30 s on the development machine (behind a hub); colour + depth ran without a disconnect.
