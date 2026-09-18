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
