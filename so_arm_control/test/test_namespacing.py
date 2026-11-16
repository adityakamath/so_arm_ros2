"""Tests for the shared `namespace` launch option (so_arm_utils/namespacing.py)."""
from launch.actions import GroupAction

from so_arm_control.so_arm_utils.namespacing import (
    in_namespace, namespace_arg, resolve_frame_prefix,
)


class TestResolveFramePrefix:
    def test_no_namespace_no_prefix(self):
        assert resolve_frame_prefix('', '') == ''

    def test_namespace_derives_prefix(self):
        assert resolve_frame_prefix('leader', '') == 'leader/'
        assert resolve_frame_prefix('/leader/', '') == 'leader/'

    def test_explicit_prefix_wins(self):
        assert resolve_frame_prefix('leader', 'custom/') == 'custom/'

    def test_prefix_without_namespace_is_kept(self):
        assert resolve_frame_prefix('', 'leader/') == 'leader/'


class TestInNamespace:
    def test_empty_namespace_is_passthrough(self):
        actions = ['a', 'b']
        assert in_namespace('', actions) == actions

    def test_namespace_wraps_in_one_group(self):
        wrapped = in_namespace('leader', ['a'])
        assert len(wrapped) == 1 and isinstance(wrapped[0], GroupAction)


def test_namespace_arg_defaults_to_empty():
    arg = namespace_arg()
    assert arg.name == 'namespace'
    assert ''.join(s.text for s in arg.default_value) == ''
