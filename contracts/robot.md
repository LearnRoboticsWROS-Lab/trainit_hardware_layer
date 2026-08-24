# Contract — robot arm

Standard `ros2_control` + MoveIt, so there is little to invent: the contract is what
MoveIt already expects. What varies between backends is only *who* moves the joints.

## Interface

| | Name | Type |
|---|---|---|
| state | `/joint_states` | `sensor_msgs/JointState` — arm **and** EEF, merged |
| action | `/<controller>/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` |
| frames | `base_link` … `tool0` … `tcp` | from the URDF |

The arm controller publishes to its own topic (`/joint_states_robot` for FR3WML) and a
merger combines it with the gripper's, so `/joint_states` is complete. TSA reads that
remap out of the base config's launch file and carries it as
`arm_joint_states_remap_to`.

## Per backend

| | How the contract is met |
|---|---|
| `mock` | `ros2_control` mock components — the loop closes instantly, no physics. |
| `isaac` | `topic_based_ros2_control`, exchanging `/isaac_joint_commands` and `/isaac_joint_states` with OmniGraph nodes in the USD. |
| `real` | `fairino_bridge` against the controller. |

## Trap

In `isaac` mode Isaac must be **playing before** the bring-up launches. Start the
controllers first and the trajectory controller reads no state, commands the zero pose,
and the arm snaps. It is the single most common way to lose an afternoon in this cell.
