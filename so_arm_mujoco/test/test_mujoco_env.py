"""Gymnasium envs: single-arm SoArmEnv and vectorized SoArmVecEnv (N independent instances)."""
import time

import numpy as np

from so_arm_mujoco.mujoco_env import SoArmEnv, SoArmVecEnv


def test_soarm_env_reset_returns_obs_in_space():
    env = SoArmEnv()
    obs, info = env.reset()

    assert env.observation_space.contains(obs)


def test_soarm_env_step_returns_obs_in_space():
    env = SoArmEnv()
    env.reset()
    action = env.action_space.sample()

    obs, reward, terminated, truncated, info = env.step(action)

    assert env.observation_space.contains(obs)
    assert reward == 0.0
    assert terminated is False
    assert truncated is False


def test_vecenv_reset_shape_matches_num_envs():
    vec = SoArmVecEnv(num_envs=4)
    obs, infos = vec.reset()

    assert obs.shape == (4,) + vec.observation_space.shape


def test_vecenv_instances_are_independent():
    vec = SoArmVecEnv(num_envs=3)
    vec.reset()

    actions = np.zeros((3,) + vec.action_space.shape, dtype=np.float32)
    actions[0] = vec.action_space.high  # only env 0 gets pushed hard

    obs, rewards, terms, truncs, infos = vec.step(actions)

    assert not np.allclose(obs[0], obs[1])
    assert np.allclose(obs[1], obs[2])


def test_vecenv_100_instances_step_within_budget():
    vec = SoArmVecEnv(num_envs=100)
    vec.reset()
    actions = np.stack([vec.action_space.sample() for _ in range(100)])

    start = time.time()
    vec.step(actions)
    elapsed = time.time() - start

    assert elapsed < 5.0
