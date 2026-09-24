# SO-ARM Description

URDF/xacro models of the SO-ARM100 family: SO100 and SO101, 5-DOF + gripper arms. The MuJoCo model (`so_arm_mujoco`) and the controllers and launch files (`so_arm_control`) are all generated from these files.

## Contents

| Path | Purpose |
|------|---------|
| `urdf/so100/so100.urdf.xacro`, `urdf/so101/so101.urdf.xacro` | Links, joints and the `<ros2_control>` block, plus the xacro arguments below |
| `urdf/*/*.control.xacro` | Shared hardware parameters: serial port, motor IDs, PID coefficients, joint limits |
| `urdf/*/*.common.xacro` | Geometry constants: colours and joint origins (from onshape-to-robot calibration) |
| `urdf/*/*.urdf` | Pre-generated URDFs of both models |
| `meshes/so100/`, `meshes/so101/` | STL files for each model, including the SO101 wrist camera mount |

## Requirements

ROS 2 Kilted or Jazzy with `xacro`, and [`sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface) (the `<ros2_control>` block references its plugin name; only needed for the actual hardware interface at runtime, not to expand the xacro).

## Running

There is no launch file here. Expand the xacro yourself, for example to check the model in RViz, or let `so_arm_control` do it at launch:

```bash
cd urdf
xacro so101/so101.urdf.xacro > /tmp/so101.urdf
xacro so100/so100.urdf.xacro > /tmp/so100.urdf
```

## Configuration

Pass these to `xacro` as `name:=value`. `so_arm_control` fills most of them in from its `urdf_config.yaml`.

| Argument | Default | Meaning |
|----------|---------|---------|
| `serial_port` | `/dev/ttySERVO` | Servo bus serial port |
| `baud_rate` | `1000000` | Servo bus baud rate |
| `use_mock` | `false` | Simulated motor responses instead of hardware (`sts_hardware_interface`'s own mock mode) |
| `use_sync_write` | `true` | Send all servo commands in one bus write |
| `shoulder_pan_motor_id` … `gripper_motor_id` | `1`–`6` | Per-joint servo IDs, in kinematic order |
| `arm_operating_mode`, `gripper_operating_mode` | `0`, `0` | Servo mode: `0` Position, `1` Velocity, `2` PWM/Effort |
| `<joint>_p_coefficient`, `_d_coefficient`, `_i_coefficient` | `14`, `24`, `0` | Per-joint Mode 0 PID, written to EEPROM on activate |
| `deadband` | `10` | Position insensitive-area (steps) |
| `ros2_control_hardware_type` | `real` | Hardware plugin: `real`, `mujoco` or `mock_components` |
| `mujoco_model`, `mujoco_headless` | `""`, `false` | `mujoco` only: the generated MJCF and whether to skip the viewer |
| `wrist_camera_urdf` | `true` | SO101 only: `false` omits the wrist camera mount/links/joints entirely |

Joint limits, motor IDs and PID values live in `*.control.xacro`'s property defaults; joint origins and colours in `*.common.xacro`. Edit those files directly to change them — they aren't exposed as xacro arguments.

## Frames

```text
base_footprint
└── base_link
    └── shoulder_link ─ shoulder_pan_joint
        └── upper_arm_link ─ shoulder_lift_joint
            └── lower_arm_link ─ elbow_flex_joint
                └── wrist_link ─ wrist_flex_joint
                    └── gripper_link ─ wrist_roll_joint
                        ├── end_effector_link          ← fixed, teleop/IK target frame
                        ├── moving_jaw_link ─ gripper_joint
                        ├── wrist_camera_mount_link     ← SO101 + wrist_camera_urdf:=true only
                        │   └── wrist_camera_body_link
                        └── wrist_camera_frame          ← same condition, separate fixed joint off gripper_link
                            └── wrist_camera_optical_frame  ← REP-103 optical frame, used as the camera's frame_id
```

## Regenerating the pre-built URDFs

The checked-in `.urdf` files are plain output from the commands in [Running](#running) above — there's no separate generator or staleness check, so regenerate them by hand after any xacro change and commit the result.

## Tests

```bash
pytest test -q
```

For both models: expands the xacro with its defaults and checks the output is valid XML, that every movable joint has valid limits, that the `<ros2_control>` block covers exactly those joints, and that every referenced mesh exists on disk. Also checks that `ros2_control_hardware_type` (`real`, `mujoco`, `mock_components`) selects the right hardware plugin, and that `arm_operating_mode`/`gripper_operating_mode:=2` (PWM) switches to an `effort` command interface where the default is `position`+`velocity`.
