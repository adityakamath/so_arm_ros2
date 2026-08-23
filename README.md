# SO-ARM ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Kilted_%7C_Jazzy-blue?logo=ros)](https://docs.ros.org)
[![CI](https://github.com/adityakamath/so_arm_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/adityakamath/so_arm_ros2/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

> ROS 2 software stack for SO-ARM100 (SO100 and SO101) robot arms.

## ⚠️ Safety

**This is a real, motorized robot arm with no hardwired physical emergency stop.** `/emergency_stop` is a software service call (toggled via joystick button) that tells the hardware interface to stop issuing motor commands. It is not a hardware kill switch, and it will not help if the software stack itself has hung, crashed, or lost connection to the joystick. Self-collision checking in `joint_trajectory_bridge` rejects self-colliding targets before they're sent to the controller, but it is not a substitute for supervision. While e-stopped, `teleop_ik_node` tracks the arm's live pose as its target instead of driving toward the pre-e-stop target, so releasing e-stop holds the arm where it was left by hand instead of snapping back. `/emergency_stop` is created by `sts_hardware_interface` itself, so it doesn't exist under `ros2_control_hardware_type:=mujoco` (a joystick e-stop toggle fails loudly with a "service not available" error there, not silently) — that mode is for ROS-graph/simulation testing only, never for anything near real hardware. `use_mock:=true` (with the default `ros2_control_hardware_type:=real`) still loads `sts_hardware_interface` and its `/emergency_stop`, just with servo I/O faked internally.

This repository is a work in progress and includes experimental and AI-generated content. Expect breaking changes and incomplete safety coverage. No warranty, express or implied — see [LICENSE](LICENSE).

## Overview

ROS 2 + ros2_control stack for the SO-ARM100 family of 5-DOF + gripper robot arms (SO100 and SO101). Features Pinocchio-based Cartesian IK joystick teleoperation, teach-and-repeat recording/replay, self-collision-checked trajectory execution, and gripper control — all driven through a single collision-checked bridge regardless of the source (teleop, GUI, or replay). Supports real hardware (Feetech STS servos) and MuJoCo simulation.

## Packages

- **so_arm_control** — ros2_control hardware interfaces and controller configs, the IK+gripper teleop node, self-collision-checked trajectory bridge, and the control-only and teleop-only launch files (real hardware or MuJoCo). Self-contained and launchable on its own.
- **so_arm_bringup** — top-level orchestration: `so_arm.launch.py` composes so_arm_control's control and teleop stacks for a single arm plus teach-and-repeat record/replay and an optional wrist camera (SO101 only); `leader_follower.launch.py` composes two of them into a dual-arm leader-follower rig.
- **so_arm_description** — URDF and MJCF robot models and meshes for SO100 and SO101.

### Dependencies

- **[ROS 2](https://docs.ros.org/en/kilted/)**: CI-tested on Kilted and Jazzy
- **[ros2_control](https://control.ros.org/)** framework with `joint_state_broadcaster` and `joint_trajectory_controller` (arm and gripper both commanded through the one controller)
- **[sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface)** (git submodule under `modules/`): Hardware interface for Feetech STS servos
- **[Pinocchio](https://github.com/stack-of-tasks/pinocchio)** (`sudo apt install ros-kilted-pinocchio`): Rigid-body kinematics library backing the IK teleop solver
- **[python-fcl](https://github.com/BerkeleyAutomation/python-fcl)** + **[numpy-stl](https://github.com/WoLpH/numpy-stl)** (pip): Mesh-based self-collision checking
- **[joy](https://github.com/ros-drivers/joystick_drivers)** / **[joy_teleop](https://index.ros.org/p/joy_teleop/)**: Joystick teleoperation
- **[mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)** (`sudo apt install ros-kilted-mujoco-ros2-control`): MuJoCo simulation backend, `ros2_control_hardware_type:=mujoco` only
- **OpenCV** (`python3-opencv`) + **cv_bridge**: wrist camera driver (`so_arm_bringup`'s `opencv_camera_node`), SO101 + `wrist_camera:=true` only
- **[rosbag2](https://github.com/ros2/rosbag2)** with the **mcap** storage plugin (`ros-kilted-rosbag2-storage-mcap`): teach-and-repeat record/replay

## Installation and Usage

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/so_arm_ros2.git
cd ~/ros2_ws
./src/so_arm_ros2/so_arm_control/scripts/bootstrap_external_deps.sh
colcon build --packages-up-to so_arm_bringup
source install/setup.bash
ros2 launch so_arm_bringup so_arm.launch.py
```

If `python-fcl` is missing, `joint_trajectory_bridge` now logs an explicit dependency error and
automatically disables self-collision checking instead of crashing. Install dependencies with:

```bash
./src/so_arm_ros2/so_arm_control/scripts/bootstrap_external_deps.sh
```

`sts_hardware_interface` is a submodule under `modules/`, left uninitialized by a plain clone. If you already have it elsewhere in this workspace (e.g. via `lekiwi_ros2/modules/`), leave it uninitialized here — colcon will find that copy. Otherwise, pull it in with `git submodule update --init --recursive` from `so_arm_ros2/` (`--recursive` also fetches its own `external/SCServo_Linux` submodule).

No real servos yet? `ros2 launch so_arm_bringup so_arm.launch.py use_mock:=true` brings up the full stack with `sts_hardware_interface`'s own mock mode (servo I/O faked internally) instead - still needs the submodule built, but no serial port or real hardware required.

No wrist camera fitted? Leave `wrist_camera` at its `true` default - the driver detects the missing camera, logs a warning, and exits cleanly instead of crashing; set `wrist_camera:=false` to skip it entirely.

`so_arm.launch.py` composes so_arm_control's `control.launch.py` and `teleop.launch.py` for you. To run just one - e.g. control only, or to restart teleop without restarting the control stack - launch them separately instead:

```bash
ros2 launch so_arm_control control.launch.py
# in a separate terminal, once that's up:
ros2 launch so_arm_control teleop.launch.py
```

For a dual-arm leader-follower rig, see `ros2 launch so_arm_bringup leader_follower.launch.py --show-arguments`.

## Stable Device Names (udev)

Real hardware (not `ros2_control_hardware_type:=mujoco`) expects the Feetech STS servo bus at a fixed `/dev/ttySERVO`, matching `serial_port`'s xacro default — not whatever `/dev/ttyUSB*`/`/dev/ttyACM*` name the kernel happens to assign on that boot. This repo doesn't bundle a udev rule; identify the adapter's vendor/product ID with `udevadm info -a -n /dev/ttyUSB0` (or `ttyACM0`) and add a rule like:

```
KERNEL=="ttyACM*", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", SYMLINK+="ttySERVO"
```

to `/etc/udev/rules.d/`, then `sudo udevadm control --reload-rules && sudo udevadm trigger` and reconnect the device.

## Launch Arguments

The most commonly used arguments for `so_arm_bringup so_arm.launch.py` (`model` through `use_sim_time` share names with `so_arm_control control.launch.py`, which it wraps - run with `--show-arguments` for the full list):

| Argument                    | Default | Description                                                            |
|------------------------------|---------|--------------------------------------------------------------------------|
| `model`                      | `so101` | Robot model to launch: `so100` or `so101`                                |
| `serial_port`                | `""`    | Serial port override; empty uses the xacro default (`/dev/ttySERVO`)     |
| `use_mock`                   | `""`    | `sts_hardware_interface`'s own mock mode (`true`/`false`); needs it built |
| `ros2_control_hardware_type` | `real`  | `real` for the STS hardware plugin, `mujoco` for MuJoCo simulation                        |
| `use_sim_time`                | `false` | Use `/clock` from a simulator instead of system time                     |
| `wrist_camera`                | `true`  | Launch the wrist camera driver (real hardware only). No camera detected → logs a warning and exits cleanly instead of crashing; set `false` if this arm has none fitted |
| `wrist_camera_urdf`           | `true`  | SO101 only: `false` omits the wrist camera mount/links/joints from `robot_description` entirely, independent of the `wrist_camera` arg above |
| `replay_loops`                | `""`    | Override record/replay's `replay_loops` (`0` = loop forever, `N>0` = exactly `N` passes); empty uses the yaml default |

`teleop.launch.py` takes no launch arguments — it loads `so_arm_control/config/teleop.yaml` directly. `so_arm_bringup leader_follower.launch.py` has its own `leader_*`/`follower_*`-prefixed argument set for dual-arm setups; see `--show-arguments`.

## Joystick Configuration

Teleoperation is configured for a **Steam Deck** used as a generic joystick, not through Steam Input, so button/axis numbers below are specific to that interface.

**Cartesian teleop** (requires the L1 deadman held):

| Control      | Action                                    |
|---------------|----------------------------------------------|
| L1            | Deadman                                       |
| Left stick    | Linear X/Y                                    |
| Right stick   | Linear Z / wrist roll                         |
| Axis 5        | Gripper open/close (also needs L1)            |

**Other controls**:

| Control   | Action                                              |
|------------|--------------------------------------------------------|
| B          | Toggle emergency stop                                   |
| X          | Toggle between GUI and joystick IK control               |
| Y          | Toggle recording (teach-and-repeat)                       |
| A          | Toggle replay of the latest recording                     |

Replay always takes priority over manual control while active — see `joint_state_switch_node`'s priority-ordered input switching.

## Gripper Compliance (Experimental, Untested)

`teleop_ik_node`'s `effort_gain` parameter (`so_arm_control/config/teleop.yaml`, default
`0.0`) shifts the commanded gripper position away from the raw joystick/GUI target in proportion
to `gripper_joint`'s sensed load, so closing on an object yields instead of driving through it —
a software approximation of impedance control that stays entirely in Mode 0 (Position): the
onboard servo PID remains the fast inner loop, so a stalled host still just holds position rather
than pushing indefinitely. Sign and magnitude are uncalibrated — start at `0.0` and raise
gradually.

## Structure

```text
so_arm_ros2/
├── so_arm_control/          # ros2_control config, IK+gripper teleop node, launch files
│   ├── config/               # control.yaml, teleop.yaml, joint_trajectory_bridge.yaml
│   ├── launch/                # control.launch.py, teleop.launch.py - control-only and teleop-only
│   └── so_arm_control/
│       ├── scripts/            # generate_srdf utility
│       └── so_arm_utils/       # Shared IK, collision-resolution, QoS helpers
├── so_arm_bringup/          # Top-level orchestration, record/replay, wrist camera
│   ├── config/               # leader_teleop.yaml, follower_teleop.yaml, record_replay.yaml, wrist_camera.yaml
│   ├── launch/                # so_arm.launch.py (single arm), leader_follower.launch.py (dual arm), record_replay.launch.py, wrist_camera.launch.py
│   └── so_arm_bringup/
│       ├── record_replay_node.py, opencv_camera_node.py
│       └── so_arm_utils/       # BagRecorder/BagPlayer (mcap round trip)
├── so_arm_description/      # URDF/MJCF models and meshes (SO100, SO101)
└── modules/
    └── sts_hardware_interface/  # Feetech STS servo hardware interface (git submodule, uninitialized unless needed)
```

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.
