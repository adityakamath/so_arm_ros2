#!/usr/bin/env python3
"""
config/{model}.srdf is generated from SelfCollisionChecker (generate_srdf.py). Regenerating it
here must give back the committed file, so a change to the URDF's collision geometry, the joint
limits or the checker that alters which pairs are checked can't land without the SRDF - which
MoveIt loads - being regenerated too:

    ros2 run so_arm_control generate_srdf -- <model> <urdf_path> <output_srdf_path>

The URDF is the default xacro (wrist camera included, the superset: move_group.launch.py drops
the entries for the camera links when the camera is left out).
"""

import pathlib
import subprocess

from ament_index_python.packages import get_package_share_directory
import pytest

from so_arm_control.scripts.generate_srdf import generate
from so_arm_control.so_arm_utils.robot_paths import urdf_xacro_path, VALID_MODELS


@pytest.mark.parametrize('model', VALID_MODELS)
def test_committed_srdf_matches_the_generator(model, tmp_path):
    urdf = tmp_path / f'{model}.urdf'
    urdf.write_text(subprocess.run(
        ['xacro', urdf_xacro_path(get_package_share_directory('so_arm_description'), model)],
        capture_output=True, text=True, check=True).stdout)
    generated = tmp_path / f'{model}.srdf'
    generate(model, str(urdf), str(generated))
    committed = pathlib.Path(get_package_share_directory('so_arm_control')) / 'config' / f'{model}.srdf'
    assert generated.read_text() == committed.read_text(), (
        f'{model}.srdf is out of date - regenerate it with generate_srdf (see this file)')
