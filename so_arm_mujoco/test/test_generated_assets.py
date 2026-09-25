"""The committed mjcf/<model>.xml must stay in sync with so_arm.mjcf.xacro.

build_model_xml() reads the committed file directly for default arguments instead of
recompiling the xacro, so a missed regeneration doesn't fail loudly - the stale copy just
silently wins, and edits to the xacro appear to do nothing. That has actually happened here
(so100.xml sat drifted for several xacro revisions). These tests are the backstop: CI fails
with the exact command to run instead of the drift reaching a viewer or a trained policy.
"""
import re
from pathlib import Path

import mujoco
import pytest

from so_arm_mujoco.simulation import (
    COMMITTED_DESCRIPTION_PATH,
    VALID_MODELS,
    committed_mjcf_path,
    committed_model_xml,
    description_share_dir,
)

REGENERATE = 'python3 -m so_arm_mujoco.cli build --commit'


@pytest.fixture(params=VALID_MODELS)
def model_name(request):
    return request.param


def test_committed_mjcf_matches_the_xacro(model_name):
    pytest.importorskip('xacro')
    path = committed_mjcf_path(model_name)

    assert path.read_text() == committed_model_xml(model_name), (
        f'{path.name} is stale - it no longer matches so_arm.mjcf.xacro. Run: {REGENERATE}'
    )


def test_committed_mjcf_keeps_mesh_paths_relative(model_name):
    """Absolute paths would compile on the machine that generated them and nowhere else."""
    text = committed_mjcf_path(model_name).read_text()
    meshes = re.findall(r'<mesh file="([^"]+)"', text)

    assert meshes, 'no <mesh> entries found - the model lost its geometry'
    for mesh in meshes:
        assert mesh.startswith(COMMITTED_DESCRIPTION_PATH), f'{mesh} is not portable'
    assert str(description_share_dir()) not in text


def test_committed_mjcf_is_self_contained(model_name):
    """xacro:include/$(find ...) must be fully resolved at generation time - MuJoCo can't."""
    text = committed_mjcf_path(model_name).read_text()

    assert '$(find' not in text
    assert '<xacro:' not in text
    assert 'xacro:include' not in text


def test_committed_mjcf_loads_in_mujoco(model_name):
    """Loads the source-checkout asset, where its relative mesh paths are valid."""
    source_path = Path(__file__).resolve().parents[1] / 'mjcf' / f'{model_name}.xml'
    model = mujoco.MjModel.from_xml_path(str(source_path))

    assert model.nu == 6
    assert model.ngeom > 0
