"""Static checks of the SRDF MoveIt loads - so_arm_control's generated SRDF, this package's extras
and the launch file's adjacent-pair entries, merged exactly as move_group.launch.py does - for
every model, and for the SO101 with and without the wrist camera. No MoveIt or ROS graph needed.

They catch what would otherwise only show up as MoveIt refusing to plan: a name the URDF doesn't
have, a named state outside the joint limits, or a link pair still enabled that touches in a pose
the arm starts from or is sent to (FCL contact in the start state is a planning failure).
Run on its own (python-fcl and pinocchio can't share a process)."""

import importlib.util
import pathlib
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

fcl = pytest.importorskip('fcl')

from so_arm_control.so_arm_utils.robot_description import urdf_xacro_cmd  # noqa: E402
from so_arm_control.so_arm_utils.self_collision_checker import SelfCollisionChecker  # noqa: E402

from _moveit_support import JOINT_GOAL  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    'move_group_launch',
    pathlib.Path(__file__).parents[1] / 'launch' / 'move_group.launch.py')
launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(launch)

CASES = [('so101', 'true'), ('so101', 'false'), ('so100', 'true')]


class Case:
    """One model's URDF and the SRDF MoveIt would be given for it."""

    def __init__(self, model, camera):
        desc = get_package_share_directory('so_arm_description')
        ctrl = get_package_share_directory('so_arm_control')
        moveit = get_package_share_directory('so_arm_moveit')
        cmd = urdf_xacro_cmd(shutil.which('xacro'), desc, ctrl, model,
                             hardware_type='mock_components', wrist_camera_urdf=camera)
        self.urdf = subprocess.run(cmd.split(), capture_output=True, text=True, check=True).stdout
        with open(f'{ctrl}/config/{model}.srdf') as f, \
                open(f'{moveit}/config/{model}_srdf_extras.xml') as g:
            self.srdf = launch.drop_unknown_links(
                launch.merge_srdf(f.read(), g.read() + launch.adjacent_pairs(self.urdf)),
                self.urdf)
        self.urdf_root, self.srdf_root = ET.fromstring(self.urdf), ET.fromstring(self.srdf)
        self.links = {x.get('name') for x in self.urdf_root.findall('link')}
        self.joints = {x.get('name'): x for x in self.urdf_root.findall('joint')}
        self.disabled = {frozenset(m) for m in re.findall(
            r'<disable_collisions link1="([^"]+)" link2="([^"]+)"', self.srdf)}
        self.checker = SelfCollisionChecker(self.urdf)
        self.states = {
            s.get('name'): {j.get('name'): float(j.get('value')) for j in s.findall('joint')}
            for s in self.srdf_root.findall('group_state')
        }

    def limits(self, joint):
        limit = self.joints[joint].find('limit')
        return float(limit.get('lower')), float(limit.get('upper'))

    def closest_enabled_pair(self, pose):
        """(distance in m, link_a, link_b) of the closest link pair MoveIt still checks."""
        objects = self.checker._link_objects
        transforms = self.checker._forward_kinematics(pose)
        for link, obj in objects.items():
            obj.setTransform(fcl.Transform(*transforms[link]))
        links = list(objects)
        return min(
            (fcl.distance(objects[a], objects[b], fcl.DistanceRequest(), fcl.DistanceResult()),
             a, b)
            for i, a in enumerate(links) for b in links[i + 1:]
            if frozenset((a, b)) not in self.disabled)


@pytest.fixture(scope='module', params=CASES, ids=[f'{m}-camera_{c}' for m, c in CASES])
def case(request):
    return Case(*request.param)


def test_srdf_names_only_links_and_joints_the_urdf_has(case):
    for entry in case.srdf_root.findall('disable_collisions'):
        assert {entry.get('link1'), entry.get('link2')} <= case.links, entry.attrib
    for group in case.srdf_root.findall('group'):
        for chain in group.findall('chain'):
            assert {chain.get('base_link'), chain.get('tip_link')} <= case.links
        for joint in group.findall('joint'):
            assert joint.get('name') in case.joints, joint.attrib
    assert case.srdf_root.find('end_effector').get('parent_link') in case.links


def test_named_states_are_within_the_joint_limits(case):
    assert case.states, 'no named states'
    for name, joints in case.states.items():
        for joint, value in joints.items():
            lower, upper = case.limits(joint)
            assert lower <= value <= upper, f'{name}: {joint}={value} outside [{lower}, {upper}]'


def test_joint_goal_used_by_the_launch_tests_is_within_the_joint_limits(case):
    for joint, value in JOINT_GOAL.items():
        lower, upper = case.limits(joint)
        assert lower <= value <= upper, f'{joint}={value} outside [{lower}, {upper}]'


def test_no_enabled_link_pair_touches_in_any_pose_the_arm_starts_from_or_goes_to(case):
    poses = {'all-zeros (the mock start pose)': {}, 'joint goal': JOINT_GOAL}
    poses.update({f'named state {name!r}': joints for name, joints in case.states.items()})
    for name, pose in poses.items():
        distance, a, b = case.closest_enabled_pair(pose)
        assert distance > 0, f'{name}: {a} touches {b} (MoveIt would refuse planning)'


def test_so100_start_pose_overlap_is_disabled():
    """The SO100's base_link and upper_arm_link overlap at all zeros; if the SRDF ever enables
    that pair again, MoveIt refuses every plan from the start pose."""
    so100 = Case('so100', 'true')
    assert frozenset(('base_link', 'upper_arm_link')) in so100.disabled
