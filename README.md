# SO-ARM ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Kilted-blue?logo=ros)](https://docs.ros.org)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

> ROS 2 software stack for SO-ARM100 (SO100 and SO101) robot arms.

## ⚠️ Safety

**This is a real, motorized robot arm with no hardwired physical emergency stop.** `/emergency_stop` is a software service call (toggled via joystick button) that tells the hardware interface to stop issuing motor commands. It is not a hardware kill switch, and it will not help if the software stack itself has hung, crashed, or lost connection to the joystick. Self-collision checking in `joint_trajectory_bridge` rejects self-colliding targets before they're sent to the controller, but it is not a substitute for supervision. Gravity-compensation mode (`gravity_comp.launch.py`) bypasses `joint_trajectory_bridge` entirely — no self-collision checking applies while it's active, only human supervision during hands-on backdriving.

This repository is a work in progress and includes experimental and AI-generated content. Expect breaking changes and incomplete safety coverage. **Gravity compensation and one-key motor calibration are both untested on physical hardware** — see their sections below before using either. No warranty, express or implied — see [LICENSE](LICENSE).

## Overview

ROS 2 + ros2_control stack for the SO-ARM100 family of 5-DOF + gripper robot arms (SO100 and SO101). Features Pinocchio-based Cartesian IK joystick teleoperation, waypoint recording and patrolling (teach-and-repeat), self-collision-checked trajectory execution, and gripper control — all driven through a single collision-checked bridge regardless of the source (teleop, GUI, or patrol). Also includes (experimental, untested) gravity compensation for manual backdriving and guided one-key motor calibration. Supports real hardware (Feetech STS servos) and MuJoCo simulation.

Control interface note: waypoint recording/following is exposed through services (`/record_waypoint`, `/waypoint_follow`, `/reset_waypoints`) in `waypoint_follow_node`; this repository does not currently expose a custom MoveToPose action.

## Packages

- **so_arm_control** — ros2_control hardware interfaces and controller configs, IK teleop / waypoint-patrol / gripper-teleop nodes, self-collision-checked trajectory bridge, and launch files (real hardware or MuJoCo).
- **so_arm_description** — URDF and MJCF robot models and meshes for SO100 and SO101.

### Dependencies

- **[ROS 2](https://docs.ros.org/en/kilted/)**: Tested with Kilted, but should work on other ROS 2 distributions
- **[ros2_control](https://control.ros.org/)** framework with `joint_state_broadcaster`, `joint_trajectory_controller`, `parallel_gripper_controller`, and `effort_controllers` (gravity compensation only)
- **[sts_hardware_interface](https://github.com/adityakamath/sts_hardware_interface)** (git submodule under `dependencies/`): Hardware interface for Feetech STS servos
- **[Pinocchio](https://github.com/stack-of-tasks/pinocchio)** (`sudo apt install ros-kilted-pinocchio`): Rigid-body kinematics library backing the IK solver shared by teleop and waypoint patrol
- **[python-fcl](https://github.com/BerkeleyAutomation/python-fcl)** + **[numpy-stl](https://github.com/WoLpH/numpy-stl)** (pip): Mesh-based self-collision checking
- **[joy](https://github.com/ros-drivers/joystick_drivers)** / **[joy_teleop](https://index.ros.org/p/joy_teleop/)**: Joystick teleoperation
- **[mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)** (`sudo apt install ros-kilted-mujoco-ros2-control`): MuJoCo simulation backend, `ros2_control_hardware_type:=mujoco` only

## Installation and Usage

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/adityakamath/so_arm_ros2.git
cd ~/ros2_ws
./src/so_arm_ros2/so_arm_control/scripts/bootstrap_external_deps.sh
colcon build --packages-up-to so_arm_control
source install/setup.bash
ros2 launch so_arm_control control.launch.py
```

If `python-fcl` is missing, `joint_trajectory_bridge` now logs an explicit dependency error and
automatically disables self-collision checking instead of crashing. Install dependencies with:

```bash
./src/so_arm_ros2/so_arm_control/scripts/bootstrap_external_deps.sh
```

If you already have `sts_hardware_interface` elsewhere in this workspace, `--recurse-submodules` is unnecessary — colcon will find either copy. Otherwise, initialize it after the fact with `git submodule update --init` from `so_arm_ros2/`.

In a separate terminal, once `control.launch.py` is up:

```bash
ros2 launch so_arm_control teleop.launch.py
```

## Stable Device Names (udev)

Real hardware (not `ros2_control_hardware_type:=mujoco`) expects the Feetech STS servo bus at a fixed `/dev/ttySERVO`, matching `serial_port`'s xacro default — not whatever `/dev/ttyUSB*`/`/dev/ttyACM*` name the kernel happens to assign on that boot. This repo doesn't bundle a udev rule; identify the adapter's vendor/product ID with `udevadm info -a -n /dev/ttyUSB0` (or `ttyACM0`) and add a rule like:

```
KERNEL=="ttyACM*", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", SYMLINK+="ttySERVO"
```

to `/etc/udev/rules.d/`, then `sudo udevadm control --reload-rules && sudo udevadm trigger` and reconnect the device.

## Launch Arguments

The most commonly used arguments for `so_arm_control control.launch.py` (run with `--show-arguments` for the full list):

| Argument                    | Default | Description                                                            |
|------------------------------|---------|--------------------------------------------------------------------------|
| `model`                      | `so101` | Robot model to launch: `so100` or `so101`                                |
| `serial_port`                | `""`    | Serial port override; empty uses the xacro default (`/dev/ttySERVO`)     |
| `use_mock`                   | `""`    | Mock hardware override (`true`/`false`); empty uses the xacro default    |
| `ros2_control_hardware_type` | `real`  | `real` for the STS hardware plugin, `mujoco` for MuJoCo simulation       |
| `self_collision_check`       | `true`  | Reject self-colliding targets; disable only for debugging                |
| `use_sim_time`                | `false` | Use `/clock` from a simulator instead of system time                     |

`teleop.launch.py` takes no launch arguments — it loads `so_arm_control/config/teleop.yaml` directly.

`gravity_comp.launch.py` takes `model`, `serial_port`, `use_mock`, and `use_sim_time` (same meaning as above) — no `ros2_control_hardware_type`/mujoco args, real or mock hardware only.

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
| Y          | Record a waypoint (end-effector pose + gripper state)    |
| A          | Toggle waypoint patrol on/off                             |
| Settings   | Reset recorded waypoints                                  |

Patrol always takes priority over manual control while active — see `joint_state_switch_node`'s priority-ordered input switching.

## Gravity Compensation (Experimental, Untested)

`gravity_comp.launch.py` puts every joint (arm + gripper) into the STS servos' PWM/effort mode and streams per-joint gravity-compensating torque — computed via Pinocchio's RNEA (`so_arm_utils/kinematics.py`'s `_PinocchioIK.gravity()`) — so the arm can be back-driven by hand for teaching poses.

```bash
ros2 launch so_arm_control gravity_comp.launch.py
```

**Separate and mutually exclusive with `control.launch.py`**: `operating_mode` (position+velocity vs. effort command interfaces) is baked into `robot_description` at hardware-interface startup, not runtime-switchable — stop one launch before starting the other. It also bypasses `joint_trajectory_bridge` entirely (see Safety above).

`max_effort` and `gravity_scale` (`so_arm_control/config/gravity_comp.yaml`) both default to a conservative `0.5`. The torque→effort conversion (`torque_nm / max_torque_nm`, the STS3215's rated stall torque) is a first-order approximation that hasn't been validated against a real servo's low-speed PWM-to-torque behavior — raise both gradually while testing, never at once.

## Calibration (Experimental, Untested)

`one_key_calibration` is a guided CLI wrapper around `sts_hardware_interface`'s `/one_key_calibration` service (position-mode joints only) — re-centers each motor's EEPROM midpoint to wherever it's currently physically positioned. Auto-detects the serial port and motor IDs from `/controller_manager`'s `robot_description`, engages `/emergency_stop` for safe manual repositioning during the process, and writes a record to `~/.config/so_arm_control/calibration/`.

```bash
ros2 run so_arm_control one_key_calibration --dry-run   # detection only, no hardware writes
ros2 run so_arm_control one_key_calibration
```

`sts_hardware_interface` documents this calibration path itself as untested on physical hardware — treat with the same caution as gravity compensation above.

## Structure

```text
so_arm_ros2/
├── so_arm_control/          # ros2_control config, teleop/waypoint-patrol/gripper nodes, launch files
│   ├── config/               # control.yaml, teleop.yaml, joint_trajectory_bridge.yaml, gravity_comp.yaml
│   ├── launch/                # control.launch.py, teleop.launch.py, gravity_comp.launch.py
│   └── so_arm_control/
│       ├── scripts/            # one_key_calibration + generate_srdf utilities
│       ├── gravity_compensation_node.py  # untested, see Gravity Compensation above
│       └── so_arm_utils/       # Shared IK/kinematic-limiting + self-collision-checking helpers
├── so_arm_description/      # URDF/MJCF models and meshes (SO100, SO101)
└── dependencies/
    └── sts_hardware_interface/  # Feetech STS servo hardware interface (git submodule)
```

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.
