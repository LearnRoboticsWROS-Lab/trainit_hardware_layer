# Contract — gripper / end effector (ADR-0022)

Two contracts exist side by side. A project uses one of them.

- **Canonical** (ADR-0022, opt-in per project). Standard ROS commands per actuator, one stamped tool
  state per device, verification as a policy. Described first.
- **Legacy** (every existing bundle, unchanged). The `/suction/on` / `/suction/off` Trigger pair
  and a command Bool. Described at the end. It means "command success is the grasp; unverified".

## The model

- An **end-effector device** is the logical tool on one robot's flange. Its identity is
  `<robot namespace>/eef/<id>`, for example `/r1/eef/composite_tool`, or `/eef/main` at the root.
- An **actuator** is one independently commandable part of the device:
  `<device>/actuators/<a>`. It has exactly ONE standard command interface, the same in every mode
  that binds it. Its success is actuation only (applied, within tolerance, reached or stalled),
  never a grasp.
- A **grasp site** is where the device holds an object: `<device>/sites/<s>`. It has one grip
  actuator, with that actuator's engage and release targets, and its own grasp observation. A
  `set_bool` grip engages with `engage` (`data: true`) and releases with `release`, never swapped:
  its polarity is the adapter's. A `gripper_command` actuator's targets have pairwise distinct
  values: a goal carries only a position, so the adapter tells ENGAGE, RELEASE and TARGET apart by
  it. A block that breaks either rule is refused (exit code 2).
- A simple suction cup is one device with one actuator (`vacuum`) and one site (`main`). A
  composite tool (axes, jaws, a lock) is the same model with more entries: one code path.
- The command interface follows the actuator's SEMANTICS, not its functional label (ADR-0022, owner
  decision of vNext M2.7): a binary open/close gripping actuator (pneumatic, electromagnetic) is a
  `set_bool`; a position- or effort-controllable gripper is a `gripper_command`; a prismatic or
  revolute positioning axis is a `follow_joint_trajectory` actuator even when the application uses
  two operating poses; a binary lock, enable or valve is a `set_bool`.

Not actuators: mimic followers, fixed frames, sensors, description pseudo-joints (the suction
prismatic joint), and external conditions such as "the part is clear of the fixture".

Whether an object is held is decided by TMR's `VerifyGrasp`, a policy over the tool state (ADR-0022
D4). It is not part of the wire form.

## Interface

Every name is RELATIVE: it resolves in the cell namespace, so two robots with identical ids never
collide (`/r1/eef/composite_tool/state` and `/r2/eef/composite_tool/state`).

| What | Name | Type | QoS | Served / published by |
|---|---|---|---|---|
| binary actuator (valve, vacuum, lock, pneumatic jaw) | `eef/<id>/actuators/<a>/set_engaged` | `std_srvs/SetBool` (`data: true` engages) | service | the device's TrainIt adapter |
| gripper actuator (one driven joint, mimic followers) | the declared action, e.g. `gripper_a_controller/gripper_cmd` | `control_msgs/GripperCommand` | action | a `GripperActionController`, or the mock at the same name |
| axis, or one coordinated multi-joint mechanism | the declared action, e.g. `linear_axis_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | action | one `joint_trajectory_controller` per actuator, or the mock at the same name |
| tool state | `eef/<id>/state` | `trainit_interfaces/EndEffectorState` | RELIABLE, TRANSIENT_LOCAL, depth 1 | the device's TrainIt adapter, and nothing else |
| adapter-owned joints | `eef/<id>/joint_states` | `sensor_msgs/JointState` | RELIABLE, depth 10 | the adapter (a composite mock bundle remaps it onto the robot's `joint_states`, vNext M2.7) |
| scene signal (single-site devices) | `eef/<id>/engaged` | `std_msgs/Bool` | RELIABLE, TRANSIENT_LOCAL, depth 1 | the adapter for a `set_bool` grip; TMR for a `gripper_command` grip |

The commands carry the tolerances where the standard allows it: a FollowJointTrajectory goal
carries its goal tolerance, goal time tolerance and path tolerance. A GripperCommand goal cannot
carry one; `reached_goal` is relative to the server's own `goal_tolerance`.

The scene signal carries the APPLIED engage (true) or release (false) of the site's grip, from any
commander. It is never a grasp.

## The tool state

`EndEffectorState`, with one nested `EndEffectorActuatorState` per actuator, published on every
change (a stamp advance included) plus a 10 Hz heartbeat that republishes an unchanged sample with
the SAME stamps. A default-constructed message means "cannot tell".

| Question | Field |
|---|---|
| Which device, which robot? | `end_effector` (computed from the adapter's namespace, never configured) |
| Which adapter produced it? | `adapter`, e.g. `trainit_mock`, `trainit_isaac_surface_gripper` |
| Is the device available? | `status`: FAULT > UNAVAILABLE > UNKNOWN > BUSY > READY over the actuators; sites never feed it |
| Is an object held at a site? | `site_grasp[i]` (UNAVAILABLE, UNKNOWN, NOT_HELD, PENDING, HELD, LOST), with `site_evidence[i]` and `site_stamp[i]`; `site_grip[i]` names the site's grip actuator |
| What did each actuator do? | `actuators[j]`: `command_interface`, `joints` (keys into JointState), `status`, the applied `binary` state of a SetBool actuator, `binary_confirmed` (only a switch ON the actuator sets it), `stamp`, and the newest APPLIED `last_command` with its stamp, from any commander |
| Why is it not healthy? | `detail`, naming each actuator or site, rejected commands included |

Rules an adapter keeps:
- evidence is NONE if and only if the grasp is UNAVAILABLE or UNKNOWN; a site with no deciding
  source (no sensor, no simulator judgement) is UNAVAILABLE. A TrainIt adapter never publishes
  `COMMAND_ECHO` or `SIM_ATTACHMENT` evidence: a command echo or a weld is never a grasp;
- PENDING after an applied engage of the site's grip, until its deciding source decides;
- LOST, latched, when HELD turns into "not held" with no applied release of the site's grip since,
  from any commander (a scene reset counts as a release); the next applied engage or release
  clears it; a rejected release does not;
- stamps are never re-stamped; `header.stamp` is the newest observation;
- a rejected command is named in `detail` and never recorded as `last_command`;
- a site whose deciding source has sent nothing since the adapter started is named in `detail`
  ("<site>: no sample from its source yet") once the source's silence timeout has passed, at once
  without one; it stays UNKNOWN and the status does not move.

Referenced, never copied: joint position, velocity and effort (`sensor_msgs/JointState`, by the
names in `actuators[].joints`), trajectory tracking (`<controller>/controller_state`) and the goal
lifecycle (`<action>/_action/status`).

## Compatibility endpoints

Opt-in, set in the adapter's parameters only (never in TMR's `eef` block). Each is a mapping onto
the grip of the device's only site, never a rename.

| What | Name | Type | Note |
|---|---|---|---|
| the policy card's Trigger pair | the card's exact names, e.g. `/suction/on` and `/suction/off` (at the root) | `std_srvs/Trigger` | ON is an engage of the grip, OFF a release; the policy runtime's commands show up in `last_command`, and its run-start OFF reads as a release, never as a loss |
| the pseudo-joint echo | e.g. `/suction/joint_states`, joint `suctioncup_joint` (0 released, 1 engaged) | `sensor_msgs/JointState` | **robot-description compatibility state** for TF/MoveIt, kept because the description needs it. It follows the applied command: pseudo-joint position != object attached. Never grasp or release evidence |
| a GripperCommand suction handle | e.g. `suctioncup_controller/gripper_command` | `control_msgs/GripperCommand` | for MoveIt's SRDF suction group; a position >= 0.5 engages; SUCCEEDED only when applied, else ABORTED |

Dropped: the String `/suction/cmd` endpoint (nothing in any workspace publishes it). Never
published by an adapter: `/suction/attached` (it would be a second writer of the policy runtime's
observation slot).

## Per backend

| Backend | Adapter row | Simple binary EEF | Simple jaw | Composite |
|---|---|---|---|---|
| `mock` | `trainit_mock`: `mock_eef_adapter` (here) | yes | yes (its joint reaches `joint_states` only through a merger row) | a mock-only bundle since vNext M2.7: the generated launch remaps the adapter's `eef/<id>/joint_states` onto `joint_states`, so the tool joints reach robot_state_publisher and MoveIt |
| `isaac` | `trainit_isaac_surface_gripper`: `isaac_eef_adapter` (here), plus the stage-side script in TSA's resources | yes; grasp detection only when the stage publishes its status | `none` | refused |
| `gazebo` | `ros2_control` | `none` | GripperActionController, no state | refused |
| `real` | `ros2_control` through the vendor bring-up, or `none` | `none` | as gazebo | refused |

A `none` binding starts no EEF server: the first command ends `SERVER_MISSING`. A third-party
controller IS the adapter for its actuator; in M2.6 it is not observed, so it has no state.

### `mock` -- `mock_eef_adapter`

One process per device, in the cell namespace. It serves every actuator on its standard interface
at the canonical names and emulates the visible behaviour of the real controllers, as M9 measured
the installed ones:
- a joint_trajectory_controller: the exact joint set, effort rejected, the carried tolerances (an
  entry naming an unknown joint, or an illegal value, drops every tolerance of the goal; with a goal
  time tolerance of 0 a goal outside its goal tolerance waits instead of ending -5), -4 / -5, and
  CANCELED with an empty result (code 0, no error string) on a cancel and on a preemption. A
  preempted goal ends only when the goal that preempted it ends, so a preempted client whose own
  deadline comes first times out; a cancel of a preempted goal is accepted and ends nothing;
- a GripperActionController: one joint, a server-side goal tolerance, the stall flags, CANCELED
  with an empty result, at once on a preemption.

Every site has a scripted sensor. Failure injection is by node parameters only (the module
docstring lists every key).

### `isaac` -- `isaac_eef_adapter`

A system-ROS node, one per robot, with `use_sim_time`, for binary suction only. It publishes the
applied command on the stage command topic (required, no default: TSA fills it from the stage topic
it detects), republished because the stage's subscription is volatile, and starts released. With
no stage a command is refused and the vacuum reads UNAVAILABLE. The stage is there when its command
topic has a subscriber and, with a status topic, when its status arrived within
`status_timeout_ms` of wall time: a subscriber alone (a recorder, an echo) is not the stage. The
site's grasp comes only from the stage's own status ("Open|Closing|Closed <gripped count>", mapped
by name) as SIM_GROUND_TRUTH; without that status topic the site is UNAVAILABLE and there is no
grasp detection on this backend. "Closed 0" after an engage is a transient (Isaac 5.1 reports it
for one frame on every attach, a re-grip included), never a decision: "Open 0" decides. The
per-robot scene reset is recorded as a release.

The stage must publish its status faster than once per `status_timeout_ms`, with a margin for its
jitter: at least 5 Hz with the default 400 ms. A grasp source's floor in ADR-0022, 1/`max_age_ms`
(2 Hz with the default 500 ms), is not enough here: the adapter reads a status older than
`status_timeout_ms` as silent, so a 2 Hz stage would read UNKNOWN for part of every period. A
status topic that never publishes (a wrong name) is named in `detail`. Silence is measured on the
wall clock as well as on the ROS clock: a dead or paused stage stops `/clock`, and with it every
sim-time silence, so a stage that sends nothing for `status_timeout_ms` of real time reads
UNKNOWN, never its last decision.

## The legacy contract (unchanged)

Kept exactly as it is for every existing project; a project that does not declare a contract
generates the same content as before.

| | Name | Type |
|---|---|---|
| service | `/suction/on` | `std_srvs/Trigger` |
| service | `/suction/off` | `std_srvs/Trigger` |
| command | `/suction/cmd` | `std_msgs/String` -- `"on"` / `"off"` |
| state | `/suction/joint_states` | `sensor_msgs/JointState` -- joint `suctioncup_joint`, 0 open / 1 closed |

The state topic is merged into `/joint_states` by a joint-state merger, so the gripper appears in
the robot's state without the arm controller knowing about it. In the canonical contract the same
topic is the compatibility pseudo-joint echo above.

| | How the legacy contract is met |
|---|---|
| `mock` | A no-op action server generated by TSA into the bundle. |
| `isaac` | `isaac_suction_bridge.py` in the cell's `_isaac` package. It answers the services and publishes a latched `std_msgs/Bool` that an OmniGraph script inside the USD reads to drive the `SurfaceGripper`. |
| `real` | `fairino_gripper`, which issues `SetDO` on the FAIRINO controller. |
