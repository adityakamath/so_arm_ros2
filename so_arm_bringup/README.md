# SO-ARM Bringup

Top-level orchestration for the SO-ARM100 family. One launch file starts control, teleop, teach-and-repeat record/replay and the optional wrist camera for a single arm; another composes two arms into a leader-follower rig. This package also holds the wrist camera driver and record/replay nodes — neither controls nor calibrates the robot, so they live here instead of `so_arm_control`.

## Contents

| Path | Purpose |
|------|---------|
| `launch/so_arm.launch.py` | Single-arm bringup: composes `so_arm_control`'s control + teleop with record/replay and the wrist camera |
| `launch/leader_follower.launch.py` | Two arms under `/leader` and `/follower` namespaces; the follower tracks the leader's realized joint states |
| `launch/record_replay.launch.py` | `record_replay_node` alone |
| `launch/wrist_camera.launch.py` | The wrist camera driver alone |
| `config/record_replay.yaml` | Recordings directory, topics, replay loop count |
| `config/leader_record_replay.yaml`, `leader_teleop.yaml`, `follower_teleop.yaml` | Leader-follower launch-argument overrides |
| `config/wrist_camera.yaml`, `wrist_camera_calibration.yaml` | Camera device settings and a placeholder (FOV-derived, not measured) calibration |
| `so_arm_bringup/record_replay_node.py` | Records `/joint_states` + `/dynamic_joint_states` + `/tf` to mcap; replays the latest as joint commands |
| `so_arm_bringup/opencv_camera_node.py` | Generic OpenCV camera driver; this repo's one deployment is the wrist camera |
| `recordings/` | Where `record_replay_node` writes and reads mcap bags |

## Requirements

The other SO-ARM packages (`colcon build --packages-up-to so_arm_bringup` brings in `so_arm_control`, `so_arm_description` and `sts_hardware_interface`), plus `rosbag2` with the mcap storage plugin (`ros-kilted-rosbag2-storage-mcap`) for record/replay, and `cv_bridge` + `python3-opencv` for the wrist camera. See `so_arm_control`'s [Requirements](../so_arm_control/README.md#requirements) for the IK/collision-checking dependencies.

## Running

```bash
ros2 launch so_arm_bringup so_arm.launch.py                                    # single arm, real hardware
ros2 launch so_arm_bringup so_arm.launch.py model:=so100 wrist_camera:=false   # so100, no camera
ros2 launch so_arm_bringup so_arm.launch.py use_mock:=true                     # no hardware, simulated motors
ros2 launch so_arm_bringup so_arm.launch.py ros2_control_hardware_type:=mujoco # MuJoCo
ros2 launch so_arm_bringup so_arm.launch.py ros2_control_hardware_type:=mujoco mujoco_arena:=true  # + cubes/tray
ros2 launch so_arm_bringup leader_follower.launch.py                           # two arms, leader-follower
ros2 launch so_arm_bringup record_replay.launch.py                             # record/replay alone
ros2 launch so_arm_bringup wrist_camera.launch.py                              # wrist camera alone
```

No wrist camera fitted? Leave `wrist_camera` at its `true` default — the driver detects the missing camera, logs a warning, and exits cleanly instead of crashing; set `wrist_camera:=false` to skip it entirely.

### Launch arguments

`so_arm.launch.py` shares `model`, `serial_port`, `use_mock`, `use_sim_time`, `ros2_control_hardware_type`, `mujoco_headless`, `mujoco_arena` and `frame_prefix` with `so_arm_control control.launch.py` — see its [Launch arguments](../so_arm_control/README.md#launch-arguments). This adds:

| Argument | Default | Meaning |
|----------|---------|---------|
| `replay_loops` | `""` | Override `record_replay_node`'s `replay_loops` (`0` = loop forever, `N>0` = exactly `N` passes); empty uses the yaml default |
| `wrist_camera` | `true` | Launch `wrist_camera_node` (real hardware only). Forced off when `wrist_camera_urdf` is `false` |
| `wrist_camera_urdf` | `true` | SO101 only: `false` omits the wrist camera mount/links/joints from `robot_description`, which also disables the camera node — no mount means no camera to talk to |

`leader_follower.launch.py` takes `model`, `use_sim_time`, `replay_loops`, and `leader_`/`follower_`-prefixed versions of `serial_port`, `use_mock`, `ros2_control_hardware_type`, `mujoco_headless` and `mujoco_arena` for each arm independently. `wrist_camera.launch.py` takes `index_or_path` and `camera_info_url` (both empty by default, falling back to yaml).

## Configuration

| File | What it sets |
|------|--------------|
| `config/record_replay.yaml` | Recordings directory (`recordings/`, resolved via `ament_index`), topics, joint names, publish rate, `replay_loops` |
| `config/wrist_camera.yaml` | Device index/path, resolution, framerate, compressed-stream settings |
| `config/wrist_camera_calibration.yaml` | Placeholder camera intrinsics (FOV-derived, not measured) — pass a real one with `camera_info_url` once you have it |
| `config/leader_teleop.yaml`, `leader_record_replay.yaml`, `follower_teleop.yaml` | Leader-follower wiring: which topics each namespace's nodes read and write |

The rest of the system is configured in the packages that own it — see `so_arm_control`'s [Configuration](../so_arm_control/README.md#configuration).

## How it works

`so_arm.launch.py` includes `so_arm_control`'s `control.launch.py` and `teleop.launch.py`, then adds `record_replay.launch.py` (always) and `wrist_camera.launch.py` (real hardware, `wrist_camera:=true`, `wrist_camera_urdf:=true`).

`leader_follower.launch.py` pushes the leader's control, teleop and record/replay into a `/leader` namespace and the follower's control into `/follower`, with a 5-second stagger so both namespaces' controller spawners don't hit the ROS graph at once. The follower gets its own `bool_toggle_node`/`joint_state_switch_node` instead of the full teleop stack: `joint_state_switch_node` arbitrates between the leader's replayed recording (redirected across the namespace boundary) and live tracking of `/leader/joint_states`, so the follower mirrors the leader by default and can independently replay a recording made from it.

## Teach-and-repeat

`record_replay_node` records the arm's realized joint states (not the commanded targets) plus TF to an mcap bag under `recordings/`, and replays the most recent one as joint commands through `joint_state_switch_node`'s `replay` input — which, on a single arm, outranks live teleop while active. Toggle recording and replay from the joystick (Y / A — see the [repository README](../README.md#joystick)) or by calling the node's services directly.

## Wrist camera

SO101 only, and only with `wrist_camera_urdf:=true` (the URDF mount must exist for the driver to have somewhere physical to point at). `opencv_camera_node` publishes `image_raw` (and a compressed stream) in the `wrist_camera_optical_frame` published by `robot_state_publisher`. `wrist_camera_calibration.yaml` is a placeholder derived from the camera's FOV, not a real calibration — replace it with `camera_info_url:=<path>` once you've calibrated your unit.

## Tests

```bash
pytest test -q
```

Covers the bag recorder/player round trip, the camera node's graceful no-device handling, and that both launch files' arguments and yaml configs are consistent.
