#!/usr/bin/env python3
"""Shared so100/so101 path conventions, used by control.launch.py and test/conftest.py."""

VALID_MODELS = ('so100', 'so101')

def urdf_xacro_path(description_share_dir: str, model: str) -> str:
    """Path to a model's URDF xacro within so_arm_description's share directory."""
    return f'{description_share_dir}/urdf/{model}/{model}.urdf.xacro'
