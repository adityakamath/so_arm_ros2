# so_arm_mujoco

Standalone MuJoCo simulation for SO-ARM100 (SO100/SO101) - **no ROS required**. Reuses
`so_arm_description`'s `mjcf/so_arm.mjcf.xacro` as its only source of truth for geometry and
actuator tuning, so it never drifts from the ROS-in-the-loop simulation below.

## Standalone (no ROS)

```bash
./scripts/bootstrap_external_deps.sh          # add --gym for the Gymnasium extra
python3 -m so_arm_mujoco.cli build --model so101 --output /tmp/so101.xml
python3 -m so_arm_mujoco.cli preview --model so101 --headless   # scripted/CI use
mjpython -m so_arm_mujoco.cli preview --model so101             # interactive viewer (macOS)
```

Run from this package's root directory - `-m` puts it on `sys.path` automatically, so no ROS
sourcing or `pip install` is required first. After `pip install -e .` (or a colcon build), the
same tools are also on PATH as `build_mujoco_models` / `mujoco_preview` (or via
`ros2 run so_arm_mujoco <name>`).

`Simulation` (`so_arm_mujoco/simulation.py`) is the reusable physics core - `reset()` /
`step()` / `set_joint_targets()` / `joint_state()` / `ee_pose()`, pure MuJoCo + numpy, no
rendering or wall-clock pacing. It backs the viewer above and both Gym envs below.

### Interactive preview controls

`mujoco_preview` (the plain, non-`--headless` viewer) lets you trip an e-stop with `Space`
(releases torque/damping so the arm falls, like real power cut). Individual joints are driven
by the viewer's own **Control** sidebar (Rendering panel on the right - per-actuator ctrl
sliders), not a custom keyboard scheme - this code deliberately never writes `data.ctrl` itself
outside of e-stop, so those sliders always take effect immediately instead of being overwritten
next frame.

## Cartesian teleop (`teleop_ik`)

```bash
mjpython -m so_arm_mujoco.teleop_ik --model so101   # interactive viewer only (macOS: mjpython)
```

Arrow keys move a mocap target that the end effector tracks via MuJoCo's own equality-constraint
solver (Shift+Up/Down for height, Shift+Left/Right to rotate the wrist); `Z` (hold) closes the
gripper; `Space` toggles the same e-stop as the plain preview above. `HeldKeys`/`EStop`
(`so_arm_mujoco/keyboard.py`) are shared between this tool and the plain preview.

`M` switches to a second mode, `joint`: disables the IK tracker and hands every actuator to the
viewer's own **Control** sidebar (same as the plain preview above) - useful for poses the
tracker's CONNECT constraint can't reach cleanly (outside the workspace, or near a singularity),
or to move one joint precisely, without a redundant keyboard scheme. `M` again switches back to
IK tracking. Either way - and across e-stop trip/release too - the tracker is kept glued to the
end effector's actual position the whole time it isn't driving, and residual velocity is zeroed
at the moment IK tracking resumes, so switching back never snaps or jerks the arm toward a stale
target (`sync_connect_constraint` in `teleop_ik.py`).

## Benchmarking (`benchmark_mujoco`)

```bash
python3 -m so_arm_mujoco.benchmark_mujoco --output /tmp/so_arm_benchmark.json
```

Headless: steps each joint through two step-response targets (25%/75% along its range) and
reports settle time, overshoot, and final error as JSON - a quick way to spot a joint whose PD
gains (in `mjcf/sts3215.mjcf.xacro`) don't converge within a reasonable window after a model or
tuning change. `--model so101` (repeatable) limits the run to specific models; defaults to all
of `so100`/`so101`.

## Gymnasium (`[gym]` extra)

```python
from so_arm_mujoco.mujoco_env import SoArmEnv, SoArmVecEnv

env = SoArmEnv(model='so101')           # single arm, standard gymnasium.Env
obs, info = env.reset()

fleet = SoArmVecEnv(num_envs=100)       # N independent arms, for training at scale
obs, infos = fleet.reset()
```

No reward, task, or termination logic is defined here on purpose - this package provides the
environment, not the RL problem. Wrap `SoArmEnv`/`SoArmVecEnv` with your own reward function;
a policy trained against `SoArmVecEnv` runs unmodified against `SoArmEnv` (same obs/action
space) for sim validation before deploying to real hardware via `so_arm_control`.

## With ROS 2 in the loop

This package isn't involved - `so_arm_control`'s `control.launch.py` already runs the same
MJCF through [`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control) when
`ros2_control_hardware_type:=mujoco`. See the top-level [README](../README.md).

## Model variants

`--model so100` or `--model so101` (default); `--no-wrist-camera` drops the SO101 wrist camera
bodies from the compiled model.

## Regenerating the committed models

`mjcf/so100.xml` and `mjcf/so101.xml` are pre-compiled from `mjcf/so_arm.mjcf.xacro` and read
back directly by `build_model_xml()` for default arguments, so **an edit to the xacro (or to
`so_arm_description`'s URDF) does nothing until they're regenerated**:

```bash
python3 -m so_arm_mujoco.cli build --commit     # rewrites every committed mjcf/<model>.xml
```

`test_generated_assets.py` fails if either file drifts from the xacro, so CI catches a missed
regeneration rather than letting the stale copy silently win. It also checks the committed
files keep mesh paths relative, resolve all `$(find ...)`/`xacro:include`, and still load.

`test_urdf_agreement.py` is the companion check in the other direction: it reimplements the
URDF's kinematics independently and verifies the MJCF agrees on link poses, joint limits, joint
rotation axes, full forward kinematics, the wrist camera's optical frame, and the
`end_effector_link` site.

### Joint sign convention

Sim joint values match `so_arm_description`'s URDF - and so the real servos through
`sts_hardware_interface` - for both models. SO101 briefly did not: its MJCF had every joint on
`axis="0 0 1"`, which sign-flipped sim joint values against the URDF (SO100 was never affected,
and nothing in `so_arm_control` compensated). All six SO101 joints are now `axis="0 0 -1"`.

That is deliberately **not** a copy of the URDF's `axis` attributes, which read `0 0 1` for
`shoulder_pan` and `0 0 -1` for the rest. What has to match is the *world* rotation axis, and
some MJCF bodies don't share their URDF link's frame (SO101's `shoulder_link` is turned 180°),
so an axis written in this file's frame can need the opposite sign for the same physical
rotation. `test_joint_axes_reproduce_urdf_world_axes` enforces the frame-agnostic rule
directly, so don't "tidy" the axes to match the URDF text.

Teleop directions are unchanged by this: the compensation lives in `teleop_ik.py`
(`_wrist_rotation_delta`, and closing the gripper toward its *upper* limit, which is what
`so_arm_control`'s `_compute_gripper_target` does on the real arm) rather than in the model.
