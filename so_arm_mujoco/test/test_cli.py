"""so_arm_mujoco.cli: only the headless path is testable without a GUI/mjpython."""
from so_arm_mujoco.cli import run_headless


def test_run_headless_steps_without_crashing():
    info = run_headless(model='so101', steps=50)

    assert info['time'] > 0.0
