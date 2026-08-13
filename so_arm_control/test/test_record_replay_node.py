#!/usr/bin/env python3
"""
Node-level unit tests for RecordReplayNode.

Covers the shared e-stop event correlation, /record's start/stop gating and raw-passthrough
write callbacks, /replay's fresh-start/resume/pause/reject gating and timer-driven playback math,
and mutual exclusion between the two - now plain in-process attribute checks
(self._writer is not None / self._segment_start_wall is not None) rather than the cross-node
service-event observation two separate nodes would need.

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
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import JointState
from service_msgs.msg import ServiceEventInfo
from so_arm_control.record_replay_node import RecordReplayNode
from std_srvs.srv import SetBool, SetBool_Event

_name_counter = itertools.count()
_JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint']


class _FakeWriter:
    """Records write()/close() calls instead of touching the filesystem."""

    def __init__(self):
        self.written = []
        self.closed = False

    def write(self, topic, data, timestamp):
        self.written.append((topic, data, timestamp))

    def close(self):
        self.closed = True


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
    node._joint_names = list(_JOINT_NAMES)
    node._replay_gripper = True
    node._gripper_joint = 'gripper_joint'

    node._writer = None
    node._bag_dir = None

    node._messages = None
    node._index = 0
    node._elapsed_at_pause = 0.0
    node._segment_start_wall = None
    node._completed = True
    node._last_gripper_sent = None

    node._estop_active = False
    node._estop_pending = {}

    published = []
    node._pub = types.SimpleNamespace(publish=lambda msg: published.append(msg))
    node.published = published
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


def _event(event_type, *, client_gid=b'\x01' * 16, seq=1, data=None, success=None):
    msg = SetBool_Event()
    msg.info.event_type = event_type
    msg.info.client_gid = list(client_gid)
    msg.info.sequence_number = seq
    if data is not None:
        req = SetBool.Request()
        req.data = data
        msg.request = [req]
    if success is not None:
        resp = SetBool.Response()
        resp.success = success
        msg.response = [resp]
    return msg


class TestTrackEvent:

    def test_full_cycle_invokes_on_change(self, node):
        seen = []
        pending = {}
        node._track_event(
            _event(ServiceEventInfo.REQUEST_RECEIVED, data=True), pending=pending,
            on_change=seen.append,
        )
        node._track_event(
            _event(ServiceEventInfo.RESPONSE_SENT, success=True), pending=pending,
            on_change=seen.append,
        )
        assert seen == [True]

    def test_failed_response_does_not_invoke_on_change(self, node):
        seen = []
        pending = {}
        node._track_event(
            _event(ServiceEventInfo.REQUEST_RECEIVED, data=True), pending=pending,
            on_change=seen.append,
        )
        node._track_event(
            _event(ServiceEventInfo.RESPONSE_SENT, success=False), pending=pending,
            on_change=seen.append,
        )
        assert seen == []

    def test_response_without_matching_request_is_ignored(self, node):
        seen = []
        node._track_event(
            _event(ServiceEventInfo.RESPONSE_SENT, success=True), pending={},
            on_change=seen.append,
        )
        assert seen == []


class TestEstopChange:

    def test_sets_flag(self, node):
        node._on_estop_change(True)
        assert node._estop_active is True

    def test_engaging_mid_recording_stops_and_saves(self, node):
        fake = _FakeWriter()
        node._writer = fake
        node._bag_dir = '/tmp/fake_bag'
        node._on_estop_change(True)
        assert fake.closed is True
        assert node._writer is None

    def test_engaging_mid_replay_aborts_it(self, node):
        node._messages = [(0.0, JointState())]
        node._completed = False
        node._segment_start_wall = time.monotonic()
        node._on_estop_change(True)
        assert node._completed is True
        assert node._segment_start_wall is None

    def test_engaging_with_both_active_stops_both(self, node):
        """The scenario merging the two nodes made possible to hit in one estop callback."""
        fake = _FakeWriter()
        node._writer = fake
        node._bag_dir = '/tmp/fake_bag'
        node._messages = [(0.0, JointState())]
        node._completed = False
        node._segment_start_wall = time.monotonic()

        node._on_estop_change(True)

        assert fake.closed is True
        assert node._writer is None
        assert node._completed is True
        assert node._segment_start_wall is None

    def test_with_nothing_active_is_a_noop(self, node):
        node._on_estop_change(True)  # must not raise
        assert node._writer is None
        assert node._messages is None


class TestAbortAndFinish:

    def test_abort_noop_when_nothing_loaded(self, node):
        node._abort_replay('test')
        assert node._completed is True

    def test_abort_noop_when_already_completed(self, node):
        node._messages = [(0.0, JointState())]
        node._completed = True
        node._segment_start_wall = None
        node._abort_replay('test')  # must not raise/re-trigger anything observable

    def test_finish_marks_completed_and_clears_segment(self, node):
        node._messages = [(0.0, JointState())]
        node._completed = False
        node._segment_start_wall = time.monotonic()
        node._finish_replay()
        assert node._completed is True
        assert node._segment_start_wall is None


class TestRawWriteCallbacks:

    def test_writes_when_recording(self, node):
        fake = _FakeWriter()
        node._writer = fake
        node._on_joint_states_raw(b'joint-bytes')
        node._on_dynamic_joint_states_raw(b'dynamic-bytes')
        assert [w[0] for w in fake.written] == ['/joint_states', '/dynamic_joint_states']
        assert [w[1] for w in fake.written] == [b'joint-bytes', b'dynamic-bytes']

    def test_does_not_write_when_not_recording(self, node):
        node._on_joint_states_raw(b'joint-bytes')
        node._on_dynamic_joint_states_raw(b'dynamic-bytes')  # no writer set - must not crash


class TestRecordCallbackGating:

    def test_start_rejected_while_estop_active(self, node):
        node._estop_active = True
        response = node._record_cb(SetBool.Request(data=True), SetBool.Response())
        assert response.success is False
        assert node._writer is None

    def test_start_rejected_while_replay_actively_playing(self, node):
        node._messages = [(0.0, JointState())]
        node._completed = False
        node._segment_start_wall = time.monotonic()
        response = node._record_cb(SetBool.Request(data=True), SetBool.Response())
        assert response.success is False
        assert node._writer is None

    def test_start_allowed_while_replay_merely_paused(self, node):
        """Pause hands control back to teleop - starting a recording then is not disallowed."""
        node._messages = [(0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0]))]
        node._completed = False
        node._segment_start_wall = None  # paused, not actively running
        response = node._record_cb(SetBool.Request(data=True), SetBool.Response())
        assert response.success is True
        assert node._writer is not None
        node._stop_recording()

    def test_start_is_idempotent_when_already_recording(self, node):
        fake = _FakeWriter()
        node._writer = fake
        node._bag_dir = '/tmp/already_recording'
        response = node._record_cb(SetBool.Request(data=True), SetBool.Response())
        assert response.success is True
        assert node._writer is fake  # untouched, no new bag opened

    def test_stop_is_idempotent_when_not_recording(self, node):
        response = node._record_cb(SetBool.Request(data=False), SetBool.Response())
        assert response.success is True
        assert response.message == 'Not recording'

    def test_stop_closes_writer_and_reports_bag_dir(self, node):
        fake = _FakeWriter()
        node._writer = fake
        node._bag_dir = '/tmp/some_bag'
        response = node._record_cb(SetBool.Request(data=False), SetBool.Response())
        assert response.success is True
        assert fake.closed is True
        assert node._writer is None
        assert '/tmp/some_bag' in response.message


class TestStartStopRecordingRealFilesystem:
    """Exercises the real rosbag2_py mcap round trip, not a double - tmp_path keeps it isolated."""

    def test_start_creates_a_bag_directory(self, node, tmp_path):
        node._recordings_dir = str(tmp_path)
        ok, message = node._start_recording()
        assert ok is True
        assert node._writer is not None
        bag_dir = node._bag_dir
        assert os.path.isdir(bag_dir)
        node._stop_recording()
        assert os.path.isfile(os.path.join(bag_dir, 'metadata.yaml'))

    def test_stop_when_never_started_is_a_noop(self, node):
        node._stop_recording()  # must not raise
        assert node._writer is None


class TestOnTimer:

    def _load(self, node, samples):
        node._messages = samples
        node._index = 0
        node._elapsed_at_pause = 0.0
        node._completed = False

    def test_paused_or_unloaded_does_not_publish(self, node):
        node._on_timer()  # nothing loaded
        assert node.published == []
        self._load(node, [(0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0]))])
        node._segment_start_wall = None  # loaded but paused
        node._on_timer()
        assert node.published == []

    def test_publishes_current_sample_positions(self, node):
        self._load(node, [
            (0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0])),
            (0.5, JointState(name=_JOINT_NAMES, position=[0.1, 0.2])),
            (10.0, JointState(name=_JOINT_NAMES, position=[0.9, 0.9])),
        ])
        node._segment_start_wall = time.monotonic() - 0.6  # 0.6s elapsed -> index advances to 1
        node._on_timer()
        assert len(node.published) == 1
        assert list(node.published[0].position) == pytest.approx([0.1, 0.2])
        assert node._index == 1

    def test_missing_joint_in_sample_skips_publish_without_crashing(self, node):
        self._load(node, [(0.0, JointState(name=['only_one_joint'], position=[0.0]))])
        node._segment_start_wall = time.monotonic()
        node._on_timer()
        assert node.published == []

    def test_gripper_goal_sent_on_change_when_enabled(self, node):
        sent = []
        node._gripper_client = type('C', (), {
            'server_is_ready': lambda self: True,
            'send_goal_async': lambda self, g: sent.append(g),
        })()
        self._load(node, [
            (0.0, JointState(name=[*_JOINT_NAMES, 'gripper_joint'], position=[0.0, 0.0, 0.3])),
        ])
        node._segment_start_wall = time.monotonic()
        node._on_timer()
        assert len(sent) == 1
        assert sent[0].command.position[0] == pytest.approx(0.3)

    def test_gripper_goal_not_sent_when_replay_gripper_disabled(self, node):
        sent = []
        node._replay_gripper = False
        node._gripper_client = type('C', (), {
            'server_is_ready': lambda self: True,
            'send_goal_async': lambda self, g: sent.append(g),
        })()
        self._load(node, [
            (0.0, JointState(name=[*_JOINT_NAMES, 'gripper_joint'], position=[0.0, 0.0, 0.3])),
        ])
        node._segment_start_wall = time.monotonic()
        node._on_timer()
        assert sent == []

    def test_reaching_the_end_finishes_replay(self, node):
        self._load(node, [
            (0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0])),
            (0.1, JointState(name=_JOINT_NAMES, position=[0.1, 0.1])),
        ])
        node._segment_start_wall = time.monotonic() - 1.0  # well past the last sample's offset
        node._on_timer()
        assert node._completed is True
        assert node._segment_start_wall is None


class TestReplayCallbackGating:

    def _req(self, data):
        return SetBool.Request(data=data)

    def test_start_rejected_while_estop_active(self, node):
        node._estop_active = True
        response = node._replay_cb(self._req(True), SetBool.Response())
        assert response.success is False
        assert node._segment_start_wall is None

    def test_start_rejected_while_recording_active(self, node):
        node._writer = _FakeWriter()
        response = node._replay_cb(self._req(True), SetBool.Response())
        assert response.success is False

    def test_fresh_start_with_no_recordings_fails(self, node, tmp_path):
        node._recordings_dir = str(tmp_path)  # empty directory - nothing to load
        response = node._replay_cb(self._req(True), SetBool.Response())
        assert response.success is False
        assert 'No recordings' in response.message
        assert node._segment_start_wall is None

    def test_fresh_start_loads_latest_and_arms_playback(self, node, tmp_path):
        _write_bag(tmp_path / '20260101_000000', [
            (0.0, {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': 0.0}),
        ])
        node._recordings_dir = str(tmp_path)
        response = node._replay_cb(self._req(True), SetBool.Response())
        assert response.success is True
        assert node._messages is not None
        assert node._completed is False
        assert node._segment_start_wall is not None

    def test_pause_then_resume_does_not_reload(self, node):
        node._messages = [(0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0]))]
        node._completed = False
        node._segment_start_wall = time.monotonic()

        def _boom():
            raise AssertionError('_load_latest_recording must not be called on resume')
        node._load_latest_recording = _boom

        pause_response = node._replay_cb(self._req(False), SetBool.Response())
        assert pause_response.success is True
        assert node._segment_start_wall is None
        assert node._elapsed_at_pause > 0.0

        resume_response = node._replay_cb(self._req(True), SetBool.Response())
        assert resume_response.success is True
        assert node._segment_start_wall is not None

    def test_pause_when_not_replaying_is_idempotent(self, node):
        response = node._replay_cb(self._req(False), SetBool.Response())
        assert response.success is True
        assert response.message == 'Not replaying'


class TestLoadLatestRecording:

    def test_picks_the_lexicographically_latest_directory(self, node, tmp_path):
        _write_bag(tmp_path / '20260101_000000', [
            (0.0, {'shoulder_pan_joint': 0.1, 'shoulder_lift_joint': 0.0}),
        ])
        _write_bag(tmp_path / '20260102_000000', [
            (0.0, {'shoulder_pan_joint': 0.2, 'shoulder_lift_joint': 0.0}),
        ])
        node._recordings_dir = str(tmp_path)
        ok, message = node._load_latest_recording()
        assert ok is True
        assert '20260102_000000' in message
        assert node._messages[0][1].position[0] == pytest.approx(0.2)

    def test_offsets_are_relative_to_the_first_sample(self, node, tmp_path):
        bag_dir = tmp_path / '20260101_000000'
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'),
        )
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name='/joint_states', type='sensor_msgs/msg/JointState',
            serialization_format='cdr',
        ))
        msg = JointState(name=_JOINT_NAMES, position=[0.0, 0.0])
        writer.write('/joint_states', serialize_message(msg), 5_000_000_000)
        writer.write('/joint_states', serialize_message(msg), 5_500_000_000)
        writer.close()

        node._recordings_dir = str(tmp_path)
        ok, _ = node._load_latest_recording()
        assert ok is True
        assert node._messages[0][0] == pytest.approx(0.0)
        assert node._messages[1][0] == pytest.approx(0.5)

    def test_missing_recordings_dir_fails_gracefully(self, node, tmp_path):
        node._recordings_dir = str(tmp_path / 'does_not_exist')
        ok, message = node._load_latest_recording()
        assert ok is False
        assert 'No recordings' in message
