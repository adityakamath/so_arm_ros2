"""Self-check for move_group.launch.py's link prefixing and SRDF merging (run: pytest test)."""
import importlib.util
import pathlib

spec = importlib.util.spec_from_file_location(
    'move_group_launch', pathlib.Path(__file__).parents[1] / 'launch' / 'move_group.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)

URDF = ('<robot><link name="base_link"/><link name="arm"/><joint name="base_link_joint">'
        '<parent link="base_link"/><child link="arm"/></joint></robot>')
SRDF = ('<robot><group name="manipulator"><chain base_link="base_link" tip_link="arm"/></group>'
        '<disable_collisions link1="base_link" link2="arm" reason="Adjacent"/></robot>')


def test_prefix_links_touches_links_not_joints():
    out = launch.prefix_links(URDF, 'leader/')
    assert '<link name="leader/base_link"' in out and '<link name="leader/arm"' in out
    assert '<parent link="leader/base_link"' in out and '<child link="leader/arm"' in out
    assert '<joint name="base_link_joint"' in out  # joint name untouched


def test_prefix_links_srdf():
    out = launch.prefix_links(SRDF, 'leader/')
    assert 'base_link="leader/base_link"' in out and 'tip_link="leader/arm"' in out
    assert 'link1="leader/base_link"' in out and 'link2="leader/arm"' in out
    assert 'name="manipulator"' in out


def test_empty_prefix_is_identity():
    assert launch.prefix_links(URDF, '') == URDF


def test_merge_srdf_inserts_before_close():
    out = launch.merge_srdf(SRDF, '<group name="gripper"/>')
    assert out.endswith('<group name="gripper"/></robot>')


def test_adjacent_pairs_follows_joints():
    assert launch.adjacent_pairs(URDF).strip() == (
        '<disable_collisions link1="base_link" link2="arm" reason="Adjacent" />')


def test_drop_unknown_links_keeps_only_known():
    srdf = ('<robot>\n  <disable_collisions link1="base_link" link2="arm" reason="x" />\n'
            '  <disable_collisions link1="base_link" link2="camera" reason="x" />\n</robot>')
    out = launch.drop_unknown_links(srdf, URDF)
    assert 'link2="arm"' in out and 'camera' not in out
