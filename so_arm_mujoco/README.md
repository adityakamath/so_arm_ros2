# SO-ARM MuJoCo

MuJoCo models of the SO-ARM100 family (SO100, SO101), generated from `so_arm_description`'s URDF — the only source of truth for geometry and actuator tuning, so this never drifts from the ROS-in-the-loop simulation. The package gives you three ways to use them:

- **Standalone:** a native viewer (plain preview and Cartesian IK teleop), a model builder and a benchmark, with no ROS needed.
- **ROS simulation:** `ros2_control_hardware_type:=mujoco` runs the same launch files as the real robot against the model in `mujoco_ros2_control`.
- **Gymnasium:** single-arm and vectorized training environments.

## Contents

| Path | Purpose |
|------|---------|
| `so_arm_mujoco/simulation.py` | The reusable physics core (`reset()`/`step()`/`joint_state()`/`ee_pose()`) and the xacro→MJCF compiler |
| `so_arm_mujoco/cli.py` | `build` and `preview` commands |
| `so_arm_mujoco/teleop_ik.py` | Cartesian IK teleop via a MuJoCo equality constraint, plus its `joint` mode |
| `so_arm_mujoco/keyboard.py` | Shared held-key tracking and e-stop, used by both viewers |
| `so_arm_mujoco/benchmark_mujoco.py` | Per-joint step-response measurement |
| `so_arm_mujoco/mujoco_env.py` | `SoArmEnv` / `SoArmVecEnv` Gymnasium environments |
| `mjcf/so_arm.mjcf.xacro`, `sts3215.mjcf.xacro` | MJCF sources: model geometry and the identified servo actuator/joint defaults |
| `mjcf/scenes/flat.mjcf.xacro`, `arena.mjcf.xacro` | Named scenes (`--scene`, see [Configuration](#configuration)) - bare skybox+floor, and a tabletop workspace with graspable cubes/tray |
| `mjcf/so100.xml`, `so101.xml` | Pre-built models (scene=flat), read directly for default arguments — see [Regenerating](#regenerating-the-committed-models) |
| `config/mujoco_ros2_control_plugins.yaml` | `mujoco_ros2_control` plugin config: the simulated `/emergency_stop`, loaded by `so_arm_control` in ROS mode |
| `config/mujoco_ros2_control_plugins_so101_camera.yaml` | Same, for the simulated wrist camera — SO101 + `wrist_camera_urdf:=true` only |

## Requirements

- Python packages, installed into the interpreter that ROS and colcon use: `./scripts/bootstrap_external_deps.sh` (`mujoco`, `glfw`; add `--gym` for the Gymnasium extra).
- For the ROS simulation only: [`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control) and `mujoco_ros2_control_plugins` (`sudo apt install ros-kilted-mujoco-ros2-control ros-kilted-mujoco-ros2-control-plugins`), plus [`mujoco_ros2_plugins`](https://github.com/adityakamath/mujoco_ros2_plugins) for the simulated `/emergency_stop` (not vendored — see the [repository README](../README.md#simulation-optional)). This package's own `config/` files are consumed by `so_arm_control`'s launch file, not by anything here — the standalone tools above never touch ROS or these plugins.

## Running

### Standalone

Run these from this directory. `preview` and `teleop_ik` open a GUI window, so on macOS run them with `mjpython`; everything else runs with plain `python3`.

```bash
# macOS
mjpython -m so_arm_mujoco.cli preview --model so101      # plain viewer
mjpython -m so_arm_mujoco.teleop_ik --model so101         # Cartesian IK teleop
mjpython -m so_arm_mujoco.teleop_ik --model so101 --scene arena   # + graspable cubes/tray, try picking one up

# Linux
python3 -m so_arm_mujoco.cli preview --model so101
python3 -m so_arm_mujoco.teleop_ik --model so101

# Any platform
python3 -m so_arm_mujoco.cli preview --model so101 --headless          # scripted/CI use
python3 -m so_arm_mujoco.cli build --model so101 --output /tmp/so101.xml
python3 -m so_arm_mujoco.benchmark_mujoco --output /tmp/motion.json
```

The same tools are also installed as commands (`ros2 run so_arm_mujoco <tool>`, or `pip install -e .`) — `build_mujoco_models` and `benchmark_mujoco` work fine that way on any platform; the installed `mujoco_preview` only works on Linux, so on macOS always launch it as `mjpython -m so_arm_mujoco.cli preview`.

**Plain preview** (`cli.py preview`): `Space` trips an e-stop (releases torque/damping so the arm falls, like real power cut). Individual joints are driven by the viewer's own **Control** sidebar (per-actuator ctrl sliders) — this code deliberately never writes to the joints itself outside of e-stop, so those sliders always take effect.

**Cartesian IK teleop** (`teleop_ik.py`): arrow keys move a mocap target the end effector tracks via MuJoCo's own equality-constraint solver (Shift+Up/Down for height, Shift+Left/Right to rotate the wrist), `Z` (hold) closes the gripper, `Space` toggles the same e-stop. `M` switches to a `joint` mode that hands every actuator to the Control sidebar instead — for poses the IK tracker can't reach cleanly, or to move one joint precisely. Switching back, and releasing e-stop, is always jerk-free: the tracker stays glued to the end effector's actual position the whole time it isn't driving, and residual velocity is zeroed the instant IK tracking resumes.

From Python:

```python
import mujoco
from so_arm_mujoco.simulation import Simulation, build_model_xml

sim = Simulation(mujoco.MjModel.from_xml_string(build_model_xml('so101')))
sim.reset()
sim.step(count=200, targets={'shoulder_pan_joint': 0.3})
```

### ROS simulation

```bash
colcon build --packages-up-to so_arm_bringup && . install/setup.bash
ros2 launch so_arm_bringup so_arm.launch.py ros2_control_hardware_type:=mujoco
ros2 launch so_arm_control control.launch.py ros2_control_hardware_type:=mujoco mujoco_headless:=true
```

`mujoco_headless:=false` (the default) opens the MuJoCo viewer; `true` needs no display. To watch a headless run from another machine, start `foxglove_bridge` and connect [Foxglove](https://foxglove.dev/) to `ws://<host>:8765`.

`so_arm_control`'s `control.launch.py` loads this package's `config/mujoco_ros2_control_plugins.yaml` into `mujoco_ros2_control`'s node, adding a simulated `/emergency_stop` (`mujoco_ros2_plugins/EmergencyStopPlugin`) and, for SO101 with `wrist_camera_urdf:=true`, the wrist camera (`mujoco_ros2_control_plugins/CameraPlugin`, `config/mujoco_ros2_control_plugins_so101_camera.yaml`) on the same `/image_raw`/`/camera_info` topics the real driver uses. The simulated e-stop **cuts actuation for every actuator**, so the arm falls under gravity (plus its own passive damping/frictionloss) — matching the standalone tools' `keyboard.py` `EStop` and the real gravity-drop on power cut. The underlying dynamics are still uncalibrated (see [Limitations](#limitations)), so treat this as integration testing of the ROS graph and failure-mode behavior, not a precise physics match.

### Gymnasium (`--gym` extra)

```python
from so_arm_mujoco.mujoco_env import SoArmEnv, SoArmVecEnv

env = SoArmEnv(model='so101')           # single arm, standard gymnasium.Env
obs, info = env.reset()

fleet = SoArmVecEnv(num_envs=100)       # N independent arms, for training at scale
obs, infos = fleet.reset()
```

No reward, task or termination logic is defined here on purpose — this provides the environment, not the RL problem. A policy trained against `SoArmVecEnv` runs unmodified against `SoArmEnv` (same obs/action space) for sim validation before deploying to real hardware via `so_arm_control`.

## Configuration

| Argument | Default | Meaning |
|----------|---------|---------|
| `--model` | `so101` | `so100` or `so101` |
| `--no-wrist-camera` | off | Drops the SO101 wrist camera bodies from the compiled model |
| `--scene` | `flat` | `flat` (skybox+floor), `arena` (flat + three graspable cubes and a tray, all within SO101's reach - `teleop_ik --scene arena` is the natural way to try it), or `none` (compose into an external scene) |
| `--headless` | off | `cli.py preview` only: step and exit, no GUI |

Timestep, integrator, and the STS3215 servo's actuator gain/damping/friction (identified via [Rhoban's BAM project](https://github.com/Rhoban/bam)) are set directly in `mjcf/so_arm.mjcf.xacro` and `sts3215.mjcf.xacro`; there is no separate `config/mujoco.yaml` here — one servo class and no wheel/payload kinematics leaves little to externalize.

## Limitations

Contact, friction, servo dynamics and inertias are uncalibrated approximations beyond the identified servo gains, with no torque-speed curves, backlash or sensor noise.

## Regenerating the committed models

`mjcf/so100.xml` and `mjcf/so101.xml` are pre-compiled from `mjcf/so_arm.mjcf.xacro` and read back directly by `build_model_xml()` for default arguments, so **an edit to the xacro (or to `so_arm_description`'s URDF) does nothing until they're regenerated**:

```bash
python3 -m so_arm_mujoco.cli build --commit     # rewrites every committed mjcf/<model>.xml
```

`test_generated_assets.py` fails if either file drifts from the xacro, so CI catches a missed regeneration rather than letting the stale copy silently win. It also checks the committed files keep mesh paths relative, resolve all `$(find ...)`/`xacro:include`, and still load.

`test_urdf_agreement.py` is the companion check in the other direction: it reimplements the URDF's kinematics independently and verifies the MJCF agrees on link poses, joint limits, joint rotation axes, full forward kinematics, the wrist camera's optical frame, and the `end_effector_link` site.

### Joint sign convention

Sim joint values match `so_arm_description`'s URDF — and so the real servos through `sts_hardware_interface` — for both models. This is **not** a copy of the URDF's `axis` attributes: some MJCF bodies don't share their URDF link's frame (SO101's `shoulder_link` is turned 180°), so an axis written in this file's frame can need the opposite sign to produce the same physical rotation. `test_joint_axes_reproduce_urdf_world_axes` enforces the frame-agnostic rule directly (`axis_mjcf = R_body_mjcf^T · (R_link_urdf · axis_urdf)`) — don't "tidy" the axes to match the URDF text.

Teleop directions are unaffected: the compensation lives in `teleop_ik.py` (`_wrist_rotation_delta`, and closing the gripper toward its upper limit, matching `so_arm_control`'s `_compute_gripper_target` on the real arm), not in the model.

## Tests

```bash
pytest test -q
```
