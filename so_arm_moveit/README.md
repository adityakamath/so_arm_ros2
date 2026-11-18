# SO-ARM MoveIt

[MoveIt 2](https://moveit.ai) motion planning for the SO-ARM100 and SO-ARM101. This package is configuration and a launch file only - there is no code to build. It plans against the arm's `joint_states` and executes through [`so_arm_control`](../so_arm_control/README.md)'s existing `so_arm_controller`, so nothing in the control stack changes to use it.

Modeled on the [ROS Physical AI SIG's `ros2_so_arm`](https://github.com/ros-physical-ai/ros2_so_arm) MoveIt config (SO100 only there; this covers both models).

## Contents

| Path | Purpose |
|------|---------|
| `launch/move_group.launch.py` | `move_group` + RViz; optionally also starts `so_arm_control`'s control stack |
| `config/{so100,so101}_srdf_extras.xml` | The `gripper` group, named states and end effector, merged into `so_arm_control`'s generated SRDF at launch |
| `config/kinematics.yaml` | KDL IK for the `manipulator` group |
| `config/joint_limits.yaml` | Velocity/acceleration limits (the URDF's are deliberately unreachable - see below) |
| `config/ompl_planning.yaml` | OMPL (RRTConnect, RRT*) and the planning request/response adapters |
| `config/moveit_controllers.yaml` | Execution: `arm_trajectory_controller` (`FollowJointTrajectory`) and `gripper_controller` (`ParallelGripperCommand`) |
| `config/servo.yaml` | MoveIt Servo, streaming to `forward_position_controller` (`servo:=true`) |
| `rviz/moveit.rviz` | MotionPlanning display with an orbit camera |
| `test/test_launch_helpers.py` | Self-check for the launch file's link-prefixing and SRDF helpers |
| `test/test_moveit_*_launch.py`, `test/_moveit_support.py`, `test/_servo_cases.py` | `launch_testing` integration tests of the whole stack (see [Testing](#testing)) |

## Running

MoveIt runs next to the control stack, in two terminals:

```bash
ros2 launch so_arm_control control.launch.py                       # real hardware (add use_mock:=true for no hardware)
ros2 launch so_arm_moveit move_group.launch.py                     # move_group + RViz
```

`launch_control:=true` starts the control stack from the same command (`use_mock:=true` for no hardware). In RViz, pick a goal state in the MotionPlanning panel (the `rest`, `zero`, `open` and `closed` named states are defined) and press *Plan & Execute*.

### Controller modes - automatic

The control stack normally runs in `teleop` mode, where `so_arm_controller` drives the arm and gripper for teleop, record/replay and the bridge. MoveIt needs different controllers (an arm-only trajectory controller, and the gripper action controller), and two controllers can't command the same joint, so this launch file switches the running stack into [`moveit` mode](../so_arm_control/README.md#controller-modes) shortly after it starts and back to `teleop` when it exits. Nothing is switched by hand. The mapping lives in `so_arm_control/config/controller_modes.yaml`. With `launch_control:=true` the stack is started in the right mode directly.

### MoveIt Servo

```bash
ros2 launch so_arm_moveit move_group.launch.py servo:=true
```

`servo:=true` runs Servo in `servo` mode instead: `forward_position_controller` streams positions to the arm, and planned execution is off (its controller is inactive). Send joint jog commands (`control_msgs/JointJog`) to `/servo_node/delta_joint_cmds` (Cartesian twist doesn't work on this arm - see Limitations). These bypass the bridge's velocity shaping and its self-collision check; Servo does its own collision checking against the planning scene.

Needs MoveIt installed (`ros-<distro>-moveit-*`, plus `moveit-servo` for `servo:=true`). MoveIt is packaged for kilted and lyrical on robostack, but this package has only been run on kilted.

### Launch arguments

| Argument | Default | Meaning |
|----------|---------|---------|
| `model` | `so101` | `so100` or `so101` |
| `namespace` | `""` | ROS namespace for every node (e.g. `leader`); see [Namespacing](#namespacing) |
| `frame_prefix` | `""` | tf frame prefix; empty derives `<namespace>/` from `namespace`, or none |
| `use_sim_time` | `false` | Use `/clock` from a simulator |
| `rviz` | `true` | Start RViz |
| `wrist_camera_urdf` | `true` | SO101 only: must match what the control launch was given, or the planning model has links the robot doesn't |
| `servo` | `false` | Run MoveIt Servo (streams to `forward_position_controller`) instead of planned execution |
| `launch_control` | `false` | Also launch `so_arm_control`'s `control.launch.py` |
| `use_mock` | `""` | `launch_control` only: the STS driver's mock mode |

## Namespacing

`namespace:=leader` pushes `move_group` and RViz into `/leader`, so it pairs with an arm launched the same way: `control.launch.py namespace:=leader` (the tested pairing), or any other so_arm launch file's `namespace` option. `leader_follower.launch.py` already uses `/leader`, so MoveIt with `namespace:=leader` targets its leader arm. `move_action`, `joint_states` and the controller's action topics all resolve under the namespace.

The arm's TF frames carry a prefix (`leader/base_link`) because `robot_state_publisher` is given `frame_prefix`. MoveIt's model and RViz plugins have no such option, so the launch file prefixes every link name in the URDF and SRDF to match, and sets RViz's fixed frame to the prefixed root. Joint names are not prefixed, same as the rest of the stack.

**RViz needs three properties set.** Running the RViz node inside `/<namespace>` is not enough for MoveIt's MotionPlanning panel. The plugin builds `move_group`'s service and action names as `append(<Move Group Namespace>, name)`, and an empty namespace yields an absolute `/name`, so the panel looks for `/move_action` and `/query_planner_interface`. Two more properties also default to absolute topics: `Planning Scene Topic` (`/monitored_planning_scene`) and the planned path's `Trajectory Topic` (`/display_planned_path`). Left alone, the panel shows "NO PLANNING LIBRARY LOADED" and no interactive marker, Plan & Execute never reaches `move_group`, and the model never follows the arm. With a namespace, the launch file writes `Move Group Namespace`, `Planning Scene Topic` and `Trajectory Topic` (all under `/<namespace>`) into the generated RViz config, and keeps the RViz node inside the namespace, because the plugin's other names are relative (its `planning_scene` publisher, for one) and only resolve to `/<namespace>/...` from there. TF needs nothing: `robot_state_publisher` broadcasts on the global `/tf` whatever its namespace, and arms are told apart by the frame prefix. If you open your own RViz config under a namespace, set those three properties yourself.

## How it reuses `so_arm_ros2`

- **Controllers:** the MoveIt-mode controllers (`arm_trajectory_controller`, `gripper_controller`, `forward_position_controller`) are defined in `so_arm_control`, since `controller_manager` has to load them and the control stack must run without MoveIt installed. This package only maps them to MoveIt (`moveit_controllers.yaml`) and asks for a mode. The stock gripper action controller serves the `gripper` group's open/close through `gripper_cmd`, with `allow_stalling` so a grasp can end on an object.
- **URDF:** built by `so_arm_control`'s shared `urdf_xacro_cmd` helper, the same one `control.launch.py` uses, so xacro arguments can't drift apart. It uses mock hardware for the `ros2_control` tag, which MoveIt ignores.
- **SRDF:** `so_arm_control/config/{model}.srdf` is generated from the runtime `SelfCollisionChecker` and owns the collision pairs; this package only adds the gripper group, named states and end effector. At launch the URDF's adjacent link pairs are also disabled (the checker tolerates small overlaps at joints; MoveIt's FCL does not and would report start-state contacts), and entries naming links absent from the URDF (the wrist camera's, with `wrist_camera_urdf:=false`) are dropped. The SO101 extras also disable three pairs that never touch but always sit inside Servo's 10 mm `self_collision_proximity_threshold` - the moving jaw against the camera mount (1.3-2.0 mm apart across the whole gripper range) and against `wrist_link` (at least 9.07 mm), and `wrist_link` against the camera mount (at least 0.52 mm across wrist roll). Left enabled, Servo's collision monitor kept decelerating, and with the camera it brought the arm to a standstill. The distances come from FCL over each pair's full joint range; any pair that can come closer than that must stay enabled.
- **Joint limits:** the URDF sets `velocity="1e6"` on every joint (and a 100 rad/s hardware ceiling), so `joint_limits.yaml` supplies real values derived from the repo's MuJoCo motor model: 8.07 rad/s (the identified STS3215's saturation speed) and per-joint accelerations of 36-84 rad/s^2 (from the model's mass matrix and gravity load at the URDF's 2.942 N*m effort). Position limits come from the URDF. Planning runs at 0.3 of these by default, about 2.4 rad/s; the derivation and how to recompute it are in the file's header. The numbers are model-derived, not measured on hardware.

## Testing

```bash
cd so_arm_moveit/test
for f in test_*.py; do pytest "$f" -q; done      # one file at a time, as CI does
```

`test_launch_helpers.py` unit-tests the launch file's string helpers (link prefixing, SRDF merging and filtering). The `launch_testing` tests run the real stack - `control.launch.py` on the mock STS hardware (`use_mock:=true`), then `move_group.launch.py` 12 s later - and talk to it through `/move_action`, the same action RViz's panel uses:

| Test | Covers |
|------|--------|
| `test_moveit_launch.py` | the launch switches the stack into `moveit` controller mode on its own; a joint goal plans and executes; the named states `rest`, `zero`, `closed`, `open` plan and execute (with the default camera model) |
| `test_moveit_namespaced_launch.py` | the same under `namespace:=leader`, including the controller-mode switch addressing `/leader/controller_manager` |
| `test_moveit_servo_launch.py`, `test_moveit_servo_camera_launch.py` | `servo:=true` puts the stack into `servo` mode and a joint jog moves the arm, without and with the wrist camera model (the shared body is in `_servo_cases.py`; the camera one guards the collision-monitor stall described above) |

They skip themselves when MoveIt isn't installed (CI installs it for Kilted, Jazzy and Lyrical) and need `pytest<8` for the `launch_testing` plugin. The RViz panel's namespace handling needs a display and isn't covered.

## Limitations

- KDL IK on a 5-DOF arm only solves reachable 6D poses, so dragging the RViz marker often finds no solution. Joint-space goals and named states are unaffected. `pick_ik` is the usual replacement if you need pose goals.
- **Servo and the mock arm:** the installed Servo (MoveIt 2.14.3) won't start its loop until it sees a joint state *change* after it launches ("Waiting to receive robot state update"). The mock arm never moves on its own, so Servo waits until something moves a joint; send one command to `/forward_position_controller/commands` and it proceeds. Real servos jitter by a step or so, which should unblock it by itself, but that is untested on hardware.
- **Servo takes joint jog only.** Cartesian twist (`delta_twist_cmds`) and pose commands halt immediately with "Very close to a singularity": MoveIt's singularity check indexes the Jacobian SVD with the 6-element twist size, which is out of bounds for a 5-joint group. This is an upstream MoveIt bug (the same code is in `moveit2` main), not a tuning problem, and no threshold in `servo.yaml` avoids it. Use `delta_joint_cmds`, or MoveIt's planned execution for Cartesian goals.
- MuJoCo and `use_sim_time` runs are untested, for both planned execution and Servo.
- SO100 loads and runs the collision check, but the mock arm starts at all-zeros, where its `base_link` and `upper_arm_link` overlap, so planning from that start state is refused.
