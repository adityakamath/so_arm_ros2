# SO-ARM ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Kilted_%7C_Jazzy-blue?logo=ros)](https://docs.ros.org)
[![CI](https://github.com/adityakamath/so_arm_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/adityakamath/so_arm_ros2/actions/workflows/ci.yml)
[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/so_arm_ros2)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

ROS 2 software stack for the SO-ARM100 family of 5-DOF + gripper robot arms (SO100 and SO101). It provides Pinocchio-based Cartesian IK joystick/GUI teleoperation with self-collision checking, gripper control, teach-and-repeat record/replay, an optional wrist camera (SO101), a dual-arm leader-follower mode, and a MuJoCo simulation.

## ⚠️ Safety

**This is a real, motorized robot arm with no hardwired physical emergency stop.** `/emergency_stop` is a software service call (toggled via joystick button) that tells the hardware interface to stop issuing motor commands. It is not a hardware kill switch, and it will not help if the software stack itself has hung, crashed, or lost connection to the joystick. Self-collision checking in `joint_trajectory_bridge` rejects self-colliding targets before they're sent to the controller, but it is not a substitute for supervision. While e-stopped, `teleop_ik_node` tracks the arm's live pose as its target instead of driving toward the pre-e-stop target, so releasing e-stop holds the arm where it was left by hand instead of snapping back. On real hardware, `/emergency_stop` is created by `sts_hardware_interface` itself, which releases torque so the arm goes limp — matching what cutting power to the servos actually does. `ros2_control_hardware_type:=mujoco` also serves `/emergency_stop` (via [`mujoco_ros2_plugins`](https://github.com/adityakamath/mujoco_ros2_plugins)), and the simulated version **cuts actuation** the same way, so the arm falls under gravity — matching the real failure mode, though the underlying dynamics are still uncalibrated (see [`so_arm_mujoco`'s Limitations](so_arm_mujoco/README.md#limitations)). `use_mock:=true` (with the default `ros2_control_hardware_type:=real`) still loads `sts_hardware_interface` and its `/emergency_stop`, just with servo I/O faked internally.

This repository is a work in progress and includes experimental and AI-generated content. Expect breaking changes and incomplete safety coverage. No warranty, express or implied — see [LICENSE](LICENSE).

## Contents

| Package | Purpose |
|---------|---------|
| [`so_arm_bringup`](so_arm_bringup/README.md) | Top-level launch files, teach-and-repeat record/replay, wrist camera driver |
| [`so_arm_control`](so_arm_control/README.md) | `ros2_control` setup, IK teleop, self-collision-checked trajectory bridge, control/teleop-only launch files |
| [`so_arm_description`](so_arm_description/README.md) | URDF models and meshes for SO100 and SO101 |
| [`so_arm_mujoco`](so_arm_mujoco/README.md) | MuJoCo models generated from the URDF, a standalone (no ROS) viewer, and Gymnasium environments |

This is built alongside, not vendored in this tree:

| Repository | Purpose |
|------------|---------|
| [`sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface) | `ros2_control` hardware interface for the Feetech STS servos |

## Hardware

| Component | Details |
|-----------|---------|
| Joints | Six Feetech STS3215 servo motors (IDs 1–6) on one serial bus at 1 Mbaud |
| Controller | A Steam Deck used as a generic joystick (see [Joystick](#joystick)) |
| Wrist camera | Optional, SO101 only — an Innomaker U20CAM-1080P (or similar UVC webcam) on the Hex-Nut Recess mount |

### Stable device names (udev)

Real hardware (not `ros2_control_hardware_type:=mujoco`) expects the servo bus at a fixed `/dev/ttySERVO`, matching `serial_port`'s xacro default — not whatever `/dev/ttyUSB*`/`/dev/ttyACM*` name the kernel assigns on that boot. This repo doesn't bundle a udev rule; identify the adapter's vendor/product ID with `udevadm info -a -n /dev/ttyUSB0` (or `ttyACM0`) and add a rule like:

```
KERNEL=="ttyACM*", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", SYMLINK+="ttySERVO"
```

to `/etc/udev/rules.d/`, then `sudo udevadm control --reload-rules && sudo udevadm trigger` and reconnect the device.

## Installation

Requires [ROS 2](https://docs.ros.org/en/kilted/) (CI-tested on Kilted and Jazzy) with:

- [`ros2_control`](https://control.ros.org/) and `ros2_controllers` (`joint_state_broadcaster`, `joint_trajectory_controller`)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (`sudo apt install ros-kilted-pinocchio`)
- [`joy`](https://github.com/ros-drivers/joystick_drivers) and [`joy_teleop`](https://index.ros.org/p/joy_teleop/)
- [rosbag2](https://github.com/ros2/rosbag2) with the mcap storage plugin (`ros-kilted-rosbag2-storage-mcap`), for teach-and-repeat

Clone and build:

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/so_arm_ros2.git
git clone --recursive https://github.com/adityakamath/sts_hardware_interface.git   # not vendored here
cd ~/ros2_ws
./src/so_arm_ros2/so_arm_control/scripts/bootstrap_external_deps.sh   # python-fcl, numpy-stl
colcon build --packages-up-to so_arm_bringup
source install/setup.bash
```

`sts_hardware_interface` is a plain package dependency, not a submodule — if you already have it built elsewhere in this workspace's `src/`, colcon will find that copy instead.

### Simulation (optional)

`ros2_control_hardware_type:=mujoco` needs the MuJoCo packages, plus [`mujoco_ros2_plugins`](https://github.com/adityakamath/mujoco_ros2_plugins) for the simulated `/emergency_stop` (not vendored here — same "build alongside" pattern as `sts_hardware_interface` above):

```bash
sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins
git clone https://github.com/adityakamath/mujoco_ros2_plugins.git src/mujoco_ros2_plugins
./src/so_arm_ros2/so_arm_mujoco/scripts/bootstrap_external_deps.sh
colcon build --packages-select mujoco_ros2_plugins
```

`mujoco_ros2_control_plugins` also drives the simulated wrist camera (SO101 only). See the [`so_arm_mujoco` README](so_arm_mujoco/README.md) for the full list, including the standalone (no ROS) viewer.

## Running

```bash
ros2 launch so_arm_bringup so_arm.launch.py                                    # single arm, real hardware
ros2 launch so_arm_bringup so_arm.launch.py model:=so100                       # so100 instead of so101
ros2 launch so_arm_bringup so_arm.launch.py use_mock:=true                     # no hardware, simulated motors
ros2 launch so_arm_bringup so_arm.launch.py ros2_control_hardware_type:=mujoco # MuJoCo simulation
ros2 launch so_arm_bringup so_arm.launch.py ros2_control_hardware_type:=mujoco mujoco_arena:=true  # + graspable cubes/tray
ros2 launch so_arm_bringup leader_follower.launch.py                           # dual-arm leader-follower
```

`so_arm.launch.py` composes `so_arm_control`'s `control.launch.py` and `teleop.launch.py` for you. To restart teleop without restarting control, launch them separately instead:

```bash
ros2 launch so_arm_control control.launch.py
# in a separate terminal, once that's up:
ros2 launch so_arm_control teleop.launch.py
```

### Launch arguments

The most common arguments for `so_arm_bringup so_arm.launch.py` (`--show-arguments` lists them all):

| Argument | Default | Description |
|----------|---------|-------------|
| `model` | `so101` | Robot model: `so100` or `so101` |
| `serial_port` | `""` | Serial port override; empty uses the xacro default (`/dev/ttySERVO`) |
| `use_mock` | `""` | `sts_hardware_interface`'s own mock mode (`true`/`false`); empty uses the xacro default |
| `ros2_control_hardware_type` | `real` | `real` for the STS hardware plugin, `mujoco` for MuJoCo simulation |
| `use_sim_time` | `false` | Use `/clock` from a simulator instead of system time |
| `mujoco_headless` | `false` | `mujoco` only: suppress the viewer window |
| `mujoco_arena` | `false` | `mujoco` only: `true` adds a small tabletop workspace within reach - three graspable cubes and a tray - on top of the default flat skybox+floor scene |
| `wrist_camera` | `true` | Launch the wrist camera driver (real hardware only); `false` if this arm has none fitted |
| `wrist_camera_urdf` | `true` | SO101 only: `false` omits the wrist camera mount/links/joints from `robot_description` |
| `replay_loops` | `""` | Override teach-and-repeat's `replay_loops` (`0` = loop forever, `N>0` = exactly `N` passes); empty uses the yaml default |

`so_arm_bringup leader_follower.launch.py` has its own `leader_*`/`follower_*`-prefixed argument set; see `--show-arguments`. The full set for each package is in its own README ([`so_arm_control`](so_arm_control/README.md#launch-arguments), [`so_arm_bringup`](so_arm_bringup/README.md#launch-arguments)).

### Joystick

Teleoperation is configured for a **Steam Deck** used as a generic joystick, not through Steam Input, so the button and axis numbers are specific to that interface.

| Control | Action |
|---------|--------|
| L1 (hold) | Deadman: teleop commands are only sent while it is held |
| Left stick | Linear X / Y |
| Right stick | Linear Z / wrist roll |
| Axis 5 | Gripper open/close (also needs L1) |
| B | Toggle emergency stop |
| X | Toggle between GUI and joystick IK control |
| Y | Toggle recording (teach-and-repeat) |
| A | Toggle replay of the latest recording |

Replay always takes priority over manual control while active — see `joint_state_switch_node`'s priority-ordered input switching in [`so_arm_control`](so_arm_control/README.md#how-it-works).

## Configuration

| File | What it sets |
|------|--------------|
| [`so_arm_control/config/urdf_config.yaml`](so_arm_control/config/urdf_config.yaml) | Servo serial port, motor IDs, PID coefficients, mock mode |
| [`so_arm_control/config/control.yaml`](so_arm_control/config/control.yaml) | Controller manager and `so_arm_controller` |
| [`so_arm_control/config/teleop.yaml`](so_arm_control/config/teleop.yaml) | Joystick buttons/axes, `teleop_ik_node` parameters (including gripper compliance) |
| [`so_arm_control/config/joint_trajectory_bridge.yaml`](so_arm_control/config/joint_trajectory_bridge.yaml) | Self-collision-check tolerances every control path routes through |
| [`so_arm_bringup/config/record_replay.yaml`](so_arm_bringup/config/record_replay.yaml) | Teach-and-repeat recordings directory, topics, replay loop count |
| [`so_arm_bringup/config/wrist_camera.yaml`](so_arm_bringup/config/wrist_camera.yaml) | Wrist camera device settings |

## Simulation

`ros2_control_hardware_type:=mujoco` runs the same launch files against a MuJoCo model instead of the hardware. The model is generated at launch from the URDF by [`so_arm_mujoco`](so_arm_mujoco/README.md), which is also usable entirely standalone with no ROS. `mujoco_ros2_plugins` and `mujoco_ros2_control_plugins` add a simulated `/emergency_stop` (cuts actuation so the arm falls, matching real hardware — see the Safety section above) and, for SO101, the wrist camera on `/image_raw`/`/camera_info`, matching the real driver's topics. This closes the ROS-graph gap with real hardware, but the underlying dynamics are still uncalibrated (see [`so_arm_mujoco`'s Limitations](so_arm_mujoco/README.md#limitations)) — this mode is for integration/graph testing, not for validating precise hardware dynamics. To watch a headless run (`mujoco_headless:=true`) from another machine, run `foxglove_bridge` and connect [Foxglove](https://foxglove.dev/) to `ws://<host>:8765`.

## Gripper compliance (experimental, untested)

`teleop_ik_node`'s `effort_gain` parameter (`so_arm_control/config/teleop.yaml`, default `0.0`) shifts the commanded gripper position away from the raw joystick/GUI target in proportion to `gripper_joint`'s sensed load, so closing on an object yields instead of driving through it — a software approximation of impedance control that stays entirely in Mode 0 (Position): the onboard servo PID remains the fast inner loop, so a stalled host still just holds position rather than pushing indefinitely. Sign and magnitude are uncalibrated — start at `0.0` and raise gradually.

## Leader-follower (dual arm)

`so_arm_bringup leader_follower.launch.py` runs two arms: the leader under `/leader` with full control and teleop, the follower under `/follower` tracking the leader's realized joint states live. Teach-and-repeat records from the leader; replaying it drives the follower (not the leader) instead, so a recorded demonstration can be played back on a second arm. See [`so_arm_bringup`'s README](so_arm_bringup/README.md#how-it-works) for how the namespacing and topic redirection work.

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.
