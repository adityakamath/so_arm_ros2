"""Standalone MuJoCo core for so_arm: resolve so_arm_description's assets, compile
so_arm.mjcf.xacro, and step physics. No ROS required.
"""
import os
from pathlib import Path

import mujoco
import numpy as np
import xacro

VALID_MODELS = ('so100', 'so101')


def _package_dir(package: str, env_var: str, marker: str) -> str:
    """env_var override -> ament_index (if ROS is sourced) -> sibling source checkout (this
    package and `package` live side by side under so_arm_ros2/). `marker` is a path, relative
    to the package root, used to sanity-check the sibling-checkout fallback.
    """
    override = os.environ.get(env_var)
    if override:
        return override

    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(package)
    except Exception:
        pass

    sibling = Path(__file__).resolve().parents[2] / package
    if (sibling / marker).exists():
        return str(sibling)

    raise FileNotFoundError(
        f'Could not locate {package} share dir. Set {env_var}, source a workspace with it '
        f'built, or check it out as a sibling of so_arm_mujoco.'
    )


def description_share_dir() -> str:
    """Where so_arm_description's meshes (and URDF) live - the mjcf itself moved here."""
    return _package_dir('so_arm_description', 'SO_ARM_DESCRIPTION_SHARE', 'meshes')


def mujoco_share_dir() -> str:
    """Where this package's own mjcf/ lives - same resolution as description_share_dir, but
    for so_arm_mujoco itself, since mjcf/so_arm.mjcf.xacro's xacro:include of sts3215.mjcf.xacro
    needs `$(find so_arm_mujoco)` resolved too.
    """
    return _package_dir('so_arm_mujoco', 'SO_ARM_MUJOCO_SHARE', 'mjcf/so_arm.mjcf.xacro')


def build_model_xml(
    model: str,
    wrist_camera: bool = True,
    scene: bool = True,
    description_share: str = None,
    mujoco_share: str = None,
) -> str:
    """xacro so_arm.mjcf.xacro -> compiled MJCF XML string.

    Resolves `$(find so_arm_description)` and `$(find so_arm_mujoco)` ourselves (via
    description_share_dir/mujoco_share_dir) before handing the xacro to the `xacro` library, so
    this works without a sourced ROS environment - xacro's own `$(find ...)` substitution needs
    ament_index + a built/sourced workspace, which standalone use explicitly should not require.

    `scene=False` omits the free-standing scene (skybox/floor/lighting) - e.g. to compose the
    arm into an external scene instead.

    For the defaults (wrist_camera=True, scene=True, no description_share override), this skips
    the xacro recompile and reads mjcf/<model>/<model>.xml directly instead - so100/so101's
    geometry is fixed (sourced from URDF, which doesn't change), so that committed file doesn't
    go stale. Any other combination still compiles live via xacro, as before. Regenerate the
    committed file via `python3 -m so_arm_mujoco.cli build --model <model>` if so_arm.mjcf.xacro
    (or the URDF it's derived from) ever actually changes.
    """
    if model not in VALID_MODELS:
        raise ValueError(f'model must be one of {VALID_MODELS}, got {model!r}')

    mujoco_dir = mujoco_share or mujoco_share_dir()

    if wrist_camera and scene and description_share is None:
        static_path = Path(mujoco_dir) / 'mjcf' / model / f'{model}.xml'
        if static_path.is_file():
            desc_dir = description_share_dir()
            return static_path.read_text().replace('../../../so_arm_description', str(desc_dir))

    desc_dir = description_share or description_share_dir()
    xacro_path = Path(mujoco_dir) / 'mjcf' / 'so_arm.mjcf.xacro'
    raw = xacro_path.read_text()
    raw = raw.replace('$(find so_arm_description)', str(desc_dir))
    raw = raw.replace('$(find so_arm_mujoco)', str(mujoco_dir))

    mappings = {
        'so_arm_config': model,
        'scene': 'true' if scene else 'false',
        'wrist_camera_mjcf': 'true' if wrist_camera else 'false',
    }
    doc = xacro.parse(raw)
    xacro.process_doc(doc, mappings=mappings)
    return doc.toprettyxml(indent='  ')


class Simulation:
    """Owns model/data, steps physics. No rendering, no wall-clock pacing - caller-paced, so
    the same class backs the interactive viewer, headless tests, and Gymnasium envs unmodified.
    """

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.data = mujoco.MjData(model)
        self.joint_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in range(model.nu)
        ]
        mujoco.mj_forward(self.model, self.data)

    def reset(self, settle_seconds: float = 0.5):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.data.ctrl[:] = self.data.qpos[: self.model.nu]
        settle_steps = int(settle_seconds / self.model.opt.timestep)
        for _ in range(settle_steps):
            mujoco.mj_step(self.model, self.data)
        self.data.time = 0.0

    def set_joint_targets(self, targets: dict):
        for name, value in targets.items():
            self.data.ctrl[self.joint_names.index(name)] = value

    def step(self, count: int = 1, targets: dict = None):
        if targets:
            self.set_joint_targets(targets)
        for _ in range(count):
            mujoco.mj_step(self.model, self.data)

    def joint_state(self) -> dict:
        return {
            'position': {n: float(self.data.qpos[i]) for i, n in enumerate(self.joint_names)},
            'velocity': {n: float(self.data.qvel[i]) for i, n in enumerate(self.joint_names)},
        }

    def ee_pose(self, site_name: str = 'end_effector_link') -> dict:
        site = self.data.site(site_name)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, site.xmat)
        return {'position': np.array(site.xpos), 'orientation': quat}

    def info(self) -> dict:
        return {
            'time': float(self.data.time),
            'ncon': int(self.data.ncon),
            'warnings': [int(w) for w in self.data.warning.number if w],
        }
