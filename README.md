# TrainIt hardware layer

The **contracts** every hardware role exposes to ROS, the **profiles** that describe a
specific piece of hardware in those terms, and the **adapters** that realise a contract
on a given backend.

```
contracts/   what a role must expose        robot · gripper · camera
profiles/    what THIS hardware is          fairino_fr3wml · softgripper_suction · realsense_d455
adapters/    one contract, one backend      mock · isaac · real · (gazebo)
```

## Why it exists

The pattern was already in the code, spread across three different homes: a vendor
package, the cell's `_isaac` package, and a file **regenerated into every bundle**. Two
projects had grown the same scatter independently. This package gives it one place and
writes the contract down instead of leaving it in docstrings.

The rule it enforces: **application code names the contract, never the backend.** A
behaviour tree calls `/suction/on`; whether that reaches Isaac, a Fairino controller or
nothing is a launch argument.

## Use it

```python
from trainit_hardware_layer import profiles

cam = profiles.load('camera', 'realsense_d455')
profiles.backend(cam, 'isaac')          # what implements the contract in this mode
profiles.available('camera')            # what this layer knows about
```

```bash
ros2 run trainit_hardware_layer mock_camera            # the camera contract, no simulator
ros2 run trainit_hardware_layer mock_camera --ros-args -p plane_distance_m:=1.5
```

## Status

| Role | mock | isaac | real |
|---|---|---|---|
| robot | external | external | external |
| gripper | *to migrate* | *to migrate* | vendor package |
| **camera** | **here** | Isaac Camera Helper — no node needed | Jetson container |

Vendor adapters stay in their own repositories: they are the vendor's, not TrainIt's.
This layer *declares* them in a profile.

Start at [`contracts/README.md`](contracts/README.md).
