#!/usr/bin/env python3
"""
Node-level unit tests for RecordReplayNode.

Covers _on_estop_change's reaction once e-stop is resolved to a bool (event correlation itself
is shared logic, tested in test_service_event.py), /record's start/stop gating, /replay's
fresh-start/resume/pause/reject gating, gripper-goal dispatch, and mutual exclusion between
record and replay - now plain in-process attribute checks against node._recorder /
node._playback rather than the cross-node service-event observation two separate nodes would
need.

BagRecorder's own start/stop/write behavior is tested in isolation in test_bag_recorder.py;
BagPlayer's own bag-loading and playback-timing/looping state machine is tested in isolation in
test_bag_player.py. This file only tests what's left once those are wired in: gating, gripper
dispatch, and the transitions RecordReplayNode itself is responsible for.

RecordReplayNode requires 'joint_names' with no default, so - like the other required-parameter
nodes in this suite - it's built via __new__()/Node.__init__() directly rather than its own
parameter-declaring constructor.
"""

import itertools
import os
import time
import types

import pytest
from rclpy.node import Node as RclpyNode
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import JointState
from so_arm_control.record_replay_node import RecordReplayNode, _resolve_package_uri
from so_arm_control.so_arm_utils.bag_player import BagPlayer
from so_arm_control.so_arm_utils.bag_recorder import BagRecorder
from std_srvs.srv import SetBool

_name_counter = itertools.count()
_JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint']


class TestRealConstruction:
    """Builds RecordReplayNode through its actual __init__, not the __new__() double below.

    Every other test in this file uses the __new__()/Node.__init__() double (see _make_node),
    which never runs RecordReplayNode's own __init__ and so can't catch a bug in it - e.g. a
    functools.partial binding a method that doesn't exist (see joint_state_switch_node's own
    history). This is the one test in the file that would catch that class of bug here too.
    """

    def test_constructs_without_error(self):
        node = RecordReplayNode(parameter_overrides=[
            Parameter('joint_names', Parameter.Type.STRING_ARRAY, _JOINT_NAMES),
            Parameter('recordings_dir', Parameter.Type.STRING, '/tmp/test_record_replay_ctor'),
        ])
        node.destroy_node()


class TestResolvePackageUri:

    def test_resolves_package_scheme_via_ament_index(self):
        from ament_index_python.packages import get_package_share_directory
        resolved = _resolve_package_uri('package://so_arm_control/recordings')
        expected = os.path.join(get_package_share_directory('so_arm_control'), 'recordings')
        assert resolved == expected

    def test_resolves_src_scheme_via_ament_index(self):
        from ament_index_python.packages import get_package_prefix
        resolved = _resolve_package_uri('src://so_arm_ros2/so_arm_control/recordings')
        ws_root = os.path.dirname(os.path.dirname(get_package_prefix('so_arm_control')))
        expected = os.path.join(ws_root, 'src', 'so_arm_ros2', 'so_arm_control', 'recordings')
        assert resolved == expected

    def test_non_package_value_is_returned_unchanged(self):
        assert _resolve_package_uri('/tmp/some/absolute/path') == '/tmp/some/absolute/path'
        assert _resolve_package_uri('relative/path') == 'relative/path'


class _FakeRecorder:
    """Mirrors BagRecorder's public surface without touching the filesystem.

    BagRecorder's own start/stop/write behavior (including the real mcap round trip) is tested
    in test_bag_recorder.py - this double is only used here to verify the node WIRES a recorder
    in and respects its state, not to re-test the recorder itself.
    """

    def __init__(self, is_recording=False, bag_dir=None):
        self.is_recording = is_recording
        self.bag_dir = bag_dir
        self.written = []
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True
        self.is_recording = True
        self.bag_dir = self.bag_dir or '/tmp/fake_bag'
        return True, f"Recording to '{self.bag_dir}'"

    def write(self, topic, data, timestamp):
        self.written.append((topic, data, timestamp))

    def stop(self):
        self.stopped = True
        bag_dir = self.bag_dir
        self.is_recording = False
        self.bag_dir = None
        return bag_dir


def _write_bag(bag_dir, samples, topic='/joint_states'):
    """Write samples ([(offset_seconds, {joint_name: position}), ...]) as a real mcap bag."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name=topic, type='sensor_msgs/msg/JointState', serialization_format='cdr',
    ))
    for offset, positions in samples:
        msg = JointState(name=list(positions), position=list(positions.values()))
        writer.write(topic, serialize_message(msg), int(offset * 1e9))
    writer.close()


def _make_node():
    node = RecordReplayNode.__new__(RecordReplayNode)
    RclpyNode.__init__(node, f'test_record_replay_{next(_name_counter)}')
    node._recordings_dir = '/tmp/unused-in-these-tests'
    node._joint_states_topic = '/joint_states'
    node._dynamic_joint_states_topic = '/dynamic_joint_states'
    node._tf_topic = '/tf'
    node._joint_names = list(_JOINT_NAMES)
    node._gripper_joint = 'gripper_joint'

    node._recorder = BagRecorder(node._recordings_dir, [
        (0, node._joint_states_topic, 'sensor_msgs/msg/JointState'),
        (1, node._dynamic_joint_states_topic, 'control_msgs/msg/DynamicJointState'),
        (2, node._tf_topic, 'tf2_msgs/msg/TFMessage'),
    ])
    node._playback = BagPlayer(replay_loops=1)
    node._last_gripper_sent = None

    node._estop_active = False

    published = []
    node._pub = types.SimpleNamespace(publish=lambda msg: published.append(msg))
    node.published = published
    node.record_active_published = []
    node._record_active_pub = types.SimpleNamespace(
        publish=lambda msg: node.record_active_published.append(msg.data),
    )
    node.replay_active_published = []
    node._replay_active_pub = types.SimpleNamespace(
        publish=lambda msg: node.replay_active_published.append(msg.data),
    )
    node._gripper_client = type(
        'C', (), {'server_is_ready': lambda self: False, 'send_goal_async': lambda self, g: None},
    )()
    node._self_client = type('C', (), {'service_is_ready': lambda self: False})()
    return node


@pytest.fixture
def node():
    n = _make_node()
    yield n
    n.destroy_node()


class TestEstopChange:

    def test_sets_flag(self, node):
        node._on_estop_change(True)
        assert node._estop_active is True

    def test_engaging_mid_recording_does_not_stop_it(self, node):
        """Hand-guided (backdriven) recording needs torque off, i.e. e-stop engaged."""
        fake = _FakeRecorder(is_recording=True, bag_dir='/tmp/fake_bag')
        node._recorder = fake
        node._on_estop_change(True)
        assert fake.stopped is False
        assert fake.is_recording is True

    def test_engaging_mid_replay_aborts_it(self, node):
        node._playback.messages = [(0.0, JointState())]
        node._playback.completed = False
        node._playback.segment_start_wall = time.monotonic()
        node._on_estop_change(True)
        assert node._playback.completed is True
        assert node._playback.segment_start_wall is None

    def test_engaging_with_both_active_stops_only_replay(self, node):
        """The scenario merging the two nodes made possible to hit in one estop callback."""
        fake = _FakeRecorder(is_recording=True, bag_dir='/tmp/fake_bag')
        node._recorder = fake
        node._playback.messages = [(0.0, JointState())]
        node._playback.completed = False
        node._playback.segment_start_wall = time.monotonic()

        node._on_estop_change(True)

        assert fake.stopped is False
        assert fake.is_recording is True
        assert node._playback.completed is True
        assert node._playback.segment_start_wall is None

    def test_with_nothing_active_is_a_noop(self, node):
        node._on_estop_change(True)  # must not raise
        assert node._recorder.is_recording is False
        assert node._playback.messages is None


class TestAbortReplay:

    def test_noop_when_nothing_loaded(self, node):
        node._abort_replay('test')
        assert node._playback.completed is True

    def test_noop_when_already_completed(self, node):
        node._playback.messages = [(0.0, JointState())]
        node._playback.completed = True
        node._playback.segment_start_wall = None
        node._abort_replay('test')  # must not raise/re-trigger anything observable

    def test_aborts_active_playback(self, node):
        node._playback.messages = [(0.0, JointState())]
        node._playback.completed = False
        node._playback.segment_start_wall = time.monotonic()
        node._abort_replay('test')
        assert node._playback.completed is True
        assert node._playback.segment_start_wall is None


class TestRawWriteCallbacks:

    def test_forwards_to_recorder_when_recording(self, node):
        fake = _FakeRecorder(is_recording=True)
        node._recorder = fake
        node._on_joint_states_raw(b'joint-bytes')
        node._on_dynamic_joint_states_raw(b'dynamic-bytes')
        node._on_tf_raw(b'tf-bytes')
        assert [w[0] for w in fake.written] == ['/joint_states', '/dynamic_joint_states', '/tf']
        assert [w[1] for w in fake.written] == [b'joint-bytes', b'dynamic-bytes', b'tf-bytes']

    def test_does_not_write_when_not_recording(self, node):
        node._on_joint_states_raw(b'joint-bytes')  # real, never-started BagRecorder - no crash
        node._on_dynamic_joint_states_raw(b'dynamic-bytes')
        node._on_tf_raw(b'tf-bytes')


class TestRecordCallbackGating:

    def test_start_allowed_while_estop_active(self, node, tmp_path):
        """Hand-guiding (backdriving) for kinesthetic teaching needs torque off during record."""
        node._recorder = BagRecorder(str(tmp_path), [
            (0, '/joint_states', 'sensor_msgs/msg/JointState'),
        ])
        node._estop_active = True
        response = node._record_cb(SetBool.Request(data=True), SetBool.Response())
        assert response.success is True
        assert node._recorder.is_recording is True
        node._recorder.stop()

    def test_start_rejected_while_replay_actively_playing(self, node):
        node._playback.messages = [(0.0, JointState())]
        node._playback.completed = False
        node._playback.segment_start_wall = time.monotonic()
        response = node._record_cb(SetBool.Request(data=True), SetBool.Response())
        assert response.success is False
        assert node._recorder.is_recording is False

    def test_start_allowed_while_replay_merely_paused(self, node, tmp_path):
        """Pause hands control back to teleop - starting a recording then is not disallowed."""
        node._recorder = BagRecorder(str(tmp_path), [
            (0, '/joint_states', 'sensor_msgs/msg/JointState'),
        ])
        node._playback.messages = [(0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0]))]
        node._playback.completed = False
        node._playback.segment_start_wall = None  # paused, not actively running
        response = node._record_cb(SetBool.Request(data=True), SetBool.Response())
        assert response.success is True
        assert node._recorder.is_recording is True
        node._recorder.stop()

    def test_start_is_idempotent_when_already_recording(self, node):
        fake = _FakeRecorder(is_recording=True, bag_dir='/tmp/already_recording')
        node._recorder = fake
        response = node._record_cb(SetBool.Request(data=True), SetBool.Response())
        assert response.success is True
        assert fake.started is False  # untouched, no new bag opened

    def test_stop_is_idempotent_when_not_recording(self, node):
        response = node._record_cb(SetBool.Request(data=False), SetBool.Response())
        assert response.success is True
        assert response.message == 'Not recording'

    def test_stop_closes_writer_and_reports_bag_dir(self, node):
        fake = _FakeRecorder(is_recording=True, bag_dir='/tmp/some_bag')
        node._recorder = fake
        response = node._record_cb(SetBool.Request(data=False), SetBool.Response())
        assert response.success is True
        assert fake.stopped is True
        assert '/tmp/some_bag' in response.message


class TestOnTimer:
    """Tests the node's own responsibilities around BagPlayer.advance()'s result: publishing,
    gripper dispatch, and finish/loop side effects. The timing/looping math itself is
    BagPlayer's job, tested directly in test_bag_player.py."""

    def _load(self, node, samples, replay_loops=1):
        node._playback = BagPlayer(replay_loops=replay_loops)
        node._playback.messages = samples
        node._playback.start(time.monotonic())

    def test_paused_or_unloaded_does_not_publish(self, node):
        node._on_timer()  # nothing loaded
        assert node.published == []
        self._load(node, [(0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0]))])
        node._playback.segment_start_wall = None  # loaded but paused
        node._on_timer()
        assert node.published == []

    def test_publishes_current_sample_positions(self, node):
        self._load(node, [
            (0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0])),
            (0.5, JointState(name=_JOINT_NAMES, position=[0.1, 0.2])),
            (10.0, JointState(name=_JOINT_NAMES, position=[0.9, 0.9])),
        ])
        node._playback.segment_start_wall = time.monotonic() - 0.6  # 0.6s elapsed -> index 1
        node._on_timer()
        assert len(node.published) == 1
        assert list(node.published[0].position) == pytest.approx([0.1, 0.2])

    def test_missing_joint_in_sample_skips_publish_without_crashing(self, node):
        self._load(node, [(0.0, JointState(name=['only_one_joint'], position=[0.0]))])
        node._on_timer()
        assert node.published == []

    def test_gripper_goal_sent_on_change(self, node):
        sent = []
        node._gripper_client = type('C', (), {
            'server_is_ready': lambda self: True,
            'send_goal_async': lambda self, g: sent.append(g),
        })()
        self._load(node, [
            (0.0, JointState(name=[*_JOINT_NAMES, 'gripper_joint'], position=[0.0, 0.0, 0.3])),
        ])
        node._on_timer()
        assert len(sent) == 1
        assert sent[0].command.position[0] == pytest.approx(0.3)

    def test_looping_resets_last_gripper_sent(self, node):
        node._last_gripper_sent = 0.5
        self._load(node, [
            (0.0, JointState(name=[*_JOINT_NAMES, 'gripper_joint'], position=[0.0, 0.0, 0.0])),
            (0.1, JointState(name=[*_JOINT_NAMES, 'gripper_joint'], position=[0.1, 0.1, 0.1])),
        ], replay_loops=0)
        node._playback.segment_start_wall = time.monotonic() - 1.0  # past the end -> loops
        node._on_timer()
        assert node._last_gripper_sent is None

    def test_reaching_the_end_finishes_and_deactivates_replay(self, node):
        calls = []
        node._self_client = type('C', (), {
            'service_is_ready': lambda self: True,
            'call_async': lambda self, req: calls.append(req.data),
        })()
        self._load(node, [
            (0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0])),
            (0.1, JointState(name=_JOINT_NAMES, position=[0.1, 0.1])),
        ], replay_loops=1)
        node._playback.segment_start_wall = time.monotonic() - 1.0  # well past the last sample
        node._on_timer()
        assert node._playback.completed is True
        assert calls == [False]


class TestActiveTopics:
    """record_active/replay_active are published from the same code paths that own the state
    transition - see so_arm_utils/qos.py's LATCHED_BOOL_QOS docstring for why."""

    def test_record_start_and_stop_publish(self, node, tmp_path):
        node._recorder = BagRecorder(str(tmp_path), [
            (0, '/joint_states', 'sensor_msgs/msg/JointState'),
        ])
        node._record_cb(SetBool.Request(data=True), SetBool.Response())
        node._record_cb(SetBool.Request(data=False), SetBool.Response())
        assert node.record_active_published == [True, False]

    def test_estop_during_recording_publishes_nothing(self, node):
        fake = _FakeRecorder(is_recording=True, bag_dir='/tmp/fake_bag')
        node._recorder = fake
        node._on_estop_change(True)
        assert node.record_active_published == []

    def test_replay_start_pause_publish(self, node, tmp_path):
        _write_bag(tmp_path / '20260101_000000', [
            (0.0, {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': 0.0}),
        ])
        node._recordings_dir = str(tmp_path)
        node._replay_cb(SetBool.Request(data=True), SetBool.Response())
        node._replay_cb(SetBool.Request(data=False), SetBool.Response())
        assert node.replay_active_published == [True, False]

    def test_replay_abort_publishes_false(self, node):
        node._playback.messages = [(0.0, JointState())]
        node._playback.completed = False
        node._playback.segment_start_wall = time.monotonic()
        node._abort_replay('test')
        assert node.replay_active_published == [False]

    def test_replay_natural_finish_publishes_false(self, node):
        node._playback = BagPlayer(replay_loops=1)
        node._playback.messages = [
            (0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0])),
            (0.1, JointState(name=_JOINT_NAMES, position=[0.1, 0.1])),
        ]
        node._playback.start(time.monotonic() - 1.0)
        node._on_timer()
        assert node.replay_active_published == [False]


class TestReplayCallbackGating:

    def _req(self, data):
        return SetBool.Request(data=data)

    def test_start_rejected_while_estop_active(self, node):
        node._estop_active = True
        response = node._replay_cb(self._req(True), SetBool.Response())
        assert response.success is False
        assert node._playback.segment_start_wall is None

    def test_start_rejected_while_recording_active(self, node):
        node._recorder = _FakeRecorder(is_recording=True)
        response = node._replay_cb(self._req(True), SetBool.Response())
        assert response.success is False

    def test_fresh_start_with_no_recordings_fails(self, node, tmp_path):
        node._recordings_dir = str(tmp_path)  # empty directory - nothing to load
        response = node._replay_cb(self._req(True), SetBool.Response())
        assert response.success is False
        assert 'No recordings' in response.message
        assert node._playback.segment_start_wall is None

    def test_fresh_start_loads_latest_and_arms_playback(self, node, tmp_path):
        _write_bag(tmp_path / '20260101_000000', [
            (0.0, {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': 0.0}),
        ])
        node._recordings_dir = str(tmp_path)
        response = node._replay_cb(self._req(True), SetBool.Response())
        assert response.success is True
        assert node._playback.messages is not None
        assert node._playback.completed is False
        assert node._playback.segment_start_wall is not None

    def test_pause_then_resume_does_not_reload(self, node):
        node._playback.messages = [(0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0]))]
        node._playback.completed = False
        node._playback.segment_start_wall = time.monotonic()

        def _boom(*args, **kwargs):
            raise AssertionError('load_latest must not be called on resume')
        node._playback.load_latest = _boom

        pause_response = node._replay_cb(self._req(False), SetBool.Response())
        assert pause_response.success is True
        assert node._playback.segment_start_wall is None
        assert node._playback.elapsed_at_pause > 0.0

        resume_response = node._replay_cb(self._req(True), SetBool.Response())
        assert resume_response.success is True
        assert node._playback.segment_start_wall is not None

    def test_pause_when_not_replaying_is_idempotent(self, node):
        response = node._replay_cb(self._req(False), SetBool.Response())
        assert response.success is True
        assert response.message == 'Not replaying'
