"""Gymnasium envs for so_arm_mujoco - part of the [gym] extra (requirements-gym.txt).

No reward/task/termination logic lives here by design: this package provides the
environment, not the RL problem. Downstream training code wraps SoArmEnv/SoArmVecEnv with
its own reward and episode-end logic.
"""
import mujoco
import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as exc:
    raise ImportError(
        'mujoco_env requires the [gym] extra: pip install -r requirements-gym.txt'
    ) from exc

from so_arm_mujoco.simulation import Simulation, build_model_xml


def _spaces_for(model: mujoco.MjModel):
    ctrlrange = model.actuator_ctrlrange
    action_space = spaces.Box(
        low=ctrlrange[:, 0].astype(np.float32), high=ctrlrange[:, 1].astype(np.float32),
    )
    obs_dim = 2 * model.nu + 7  # joint pos + joint vel + ee position(3) + ee quaternion(4)
    observation_space = spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)
    return action_space, observation_space


def _observe(sim: Simulation) -> np.ndarray:
    state = sim.joint_state()
    pos = np.array([state['position'][n] for n in sim.joint_names], dtype=np.float32)
    vel = np.array([state['velocity'][n] for n in sim.joint_names], dtype=np.float32)
    ee = sim.ee_pose()
    return np.concatenate([pos, vel, ee['position'], ee['orientation']]).astype(np.float32)


class SoArmEnv(gym.Env):
    """Single-instance Gymnasium env, backed by Simulation."""

    metadata = {'render_modes': []}

    def __init__(self, model: str = 'so101', wrist_camera: bool = True, scene: str = 'flat'):
        xml = build_model_xml(model, wrist_camera=wrist_camera, scene=scene)
        self.sim = Simulation(mujoco.MjModel.from_xml_string(xml))
        self.action_space, self.observation_space = _spaces_for(self.sim.model)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.sim.reset()
        return _observe(self.sim), {}

    def step(self, action):
        targets = dict(zip(self.sim.joint_names, np.asarray(action, dtype=float)))
        self.sim.step(count=1, targets=targets)
        return _observe(self.sim), 0.0, False, False, self.sim.info()


class SoArmVecEnv:
    """N independent SoArmEnv instances (one shared mjModel, N mjData), for training a policy
    against a fleet of simulated arms before deploying it to one.

    ponytail: stepped in a plain Python loop, not mujoco.rollout's batched/threaded API - the
    simplest correct thing, and a shared mjModel already avoids N redundant xacro/MJCF
    compiles. Swap step()'s loop body for mujoco.rollout if profiling shows this loop, not
    mj_step itself, is the bottleneck at your N.
    """

    def __init__(
        self, num_envs: int, model: str = 'so101', wrist_camera: bool = True,
        scene: str = 'flat',
    ):
        xml = build_model_xml(model, wrist_camera=wrist_camera, scene=scene)
        mjmodel = mujoco.MjModel.from_xml_string(xml)
        self.num_envs = num_envs
        self.sims = [Simulation(mjmodel) for _ in range(num_envs)]
        self.action_space, self.observation_space = _spaces_for(mjmodel)

    def reset(self):
        for sim in self.sims:
            sim.reset()
        return np.stack([_observe(s) for s in self.sims]), {}

    def step(self, actions):
        actions = np.asarray(actions, dtype=float)
        for sim, action in zip(self.sims, actions):
            sim.step(count=1, targets=dict(zip(sim.joint_names, action)))
        obs = np.stack([_observe(s) for s in self.sims])
        rewards = np.zeros(self.num_envs, dtype=np.float32)
        terminations = np.zeros(self.num_envs, dtype=bool)
        truncations = np.zeros(self.num_envs, dtype=bool)
        return obs, rewards, terminations, truncations, {}
