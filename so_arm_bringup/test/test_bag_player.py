#!/usr/bin/env python3
"""Unit tests for BagPlayer - no ROS node needed. TestLoadLatest exercises the real rosbag2_py
mcap round trip; the rest exercise the timing/looping state machine directly with synthetic
(offset, JointState) samples and hand-picked 'now' values."""

import pytest
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import JointState
from so_arm_bringup.so_arm_utils.bag_player import BagPlayer

_JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint']


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


class TestConstruction:

    def test_negative_replay_loops_raises(self):
        with pytest.raises(ValueError):
            BagPlayer(-1)


class TestLoadLatest:

    def test_picks_the_lexicographically_latest_directory(self, tmp_path):
        _write_bag(tmp_path / '20260101_000000', [
            (0.0, {'shoulder_pan_joint': 0.1, 'shoulder_lift_joint': 0.0}),
        ])
        _write_bag(tmp_path / '20260102_000000', [
            (0.0, {'shoulder_pan_joint': 0.2, 'shoulder_lift_joint': 0.0}),
        ])
        player = BagPlayer(replay_loops=1)
        ok, message = player.load_latest(str(tmp_path), '/joint_states')
        assert ok is True
        assert '20260102_000000' in message
        assert player.messages[0][1].position[0] == pytest.approx(0.2)

    def test_offsets_are_relative_to_the_first_sample(self, tmp_path):
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

        player = BagPlayer(replay_loops=1)
        ok, _ = player.load_latest(str(tmp_path), '/joint_states')
        assert ok is True
        assert player.messages[0][0] == pytest.approx(0.0)
        assert player.messages[1][0] == pytest.approx(0.5)

    def test_missing_recordings_dir_fails_gracefully(self, tmp_path):
        player = BagPlayer(replay_loops=1)
        ok, message = player.load_latest(str(tmp_path / 'does_not_exist'), '/joint_states')
        assert ok is False
        assert 'No recordings' in message

    def test_empty_recordings_dir_fails_gracefully(self, tmp_path):
        player = BagPlayer(replay_loops=1)
        ok, message = player.load_latest(str(tmp_path), '/joint_states')
        assert ok is False
        assert 'No recordings' in message


class TestArmAbortFinish:

    def test_arm_resets_position_and_loop_count(self):
        player = BagPlayer(replay_loops=1)
        player.index = 5
        player.elapsed_at_pause = 3.0
        player.completed = True
        player.loops_completed = 2
        player.arm()
        assert player.index == 0
        assert player.elapsed_at_pause == 0.0
        assert player.completed is False
        assert player.loops_completed == 0

    def test_abort_stops_and_marks_completed(self):
        player = BagPlayer(replay_loops=1)
        player.segment_start_wall = 10.0
        player.abort()
        assert player.segment_start_wall is None
        assert player.completed is True

    def test_finish_stops_and_marks_completed(self):
        player = BagPlayer(replay_loops=1)
        player.segment_start_wall = 10.0
        player.finish()
        assert player.segment_start_wall is None
        assert player.completed is True


class TestStartPauseResume:

    def test_start_sets_segment_start(self):
        player = BagPlayer(replay_loops=1)
        player.start(100.0)
        assert player.segment_start_wall == 100.0

    def test_pause_accumulates_elapsed_and_clears_segment(self):
        player = BagPlayer(replay_loops=1)
        player.start(100.0)
        player.pause(102.5)
        assert player.elapsed_at_pause == pytest.approx(2.5)
        assert player.segment_start_wall is None

    def test_is_running_reflects_loaded_and_started_state(self):
        player = BagPlayer(replay_loops=1)
        assert player.is_running is False
        player.messages = [(0.0, JointState())]
        assert player.is_running is False  # loaded but not started
        player.start(0.0)
        assert player.is_running is True


class TestAdvance:

    def _samples(self):
        return [
            (0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0])),
            (0.5, JointState(name=_JOINT_NAMES, position=[0.1, 0.2])),
            (10.0, JointState(name=_JOINT_NAMES, position=[0.9, 0.9])),
        ]

    def test_returns_none_when_nothing_loaded(self):
        player = BagPlayer(replay_loops=1)
        assert player.advance(0.0) is None

    def test_returns_none_when_paused(self):
        player = BagPlayer(replay_loops=1)
        player.messages = self._samples()
        assert player.advance(0.0) is None  # segment_start_wall is None

    def test_advances_index_to_current_sample(self):
        player = BagPlayer(replay_loops=1)
        player.messages = self._samples()
        player.start(0.0)
        sample, looped, finished = player.advance(0.6)  # 0.6s elapsed -> index advances to 1
        assert list(sample.position) == pytest.approx([0.1, 0.2])
        assert player.index == 1
        assert looped is False
        assert finished is False

    def test_default_replay_loops_finishes_after_one_pass(self):
        player = BagPlayer(replay_loops=1)
        player.messages = [
            (0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0])),
            (0.1, JointState(name=_JOINT_NAMES, position=[0.1, 0.1])),
        ]
        player.start(0.0)
        _, looped, finished = player.advance(1.0)  # well past the last sample's offset
        assert looped is False
        assert finished is True
        assert player.completed is True
        assert player.segment_start_wall is None

    def test_zero_loops_forever_does_not_finish_and_wraps(self):
        player = BagPlayer(replay_loops=0)
        player.messages = [
            (0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0])),
            (0.1, JointState(name=_JOINT_NAMES, position=[0.1, 0.1])),
        ]
        player.arm()
        player.start(0.0)
        _, looped, finished = player.advance(1.0)
        assert looped is True
        assert finished is False
        assert player.completed is False
        assert player.index == 0  # looped back to the start
        assert player.loops_completed == 1
        assert player.segment_start_wall == 1.0  # loop rebases the segment start to 'now'

    def test_n_loops_finishes_after_exactly_n_passes(self):
        player = BagPlayer(replay_loops=3)
        player.messages = [
            (0.0, JointState(name=_JOINT_NAMES, position=[0.0, 0.0])),
            (0.1, JointState(name=_JOINT_NAMES, position=[0.1, 0.1])),
        ]
        player.arm()
        player.start(0.0)
        # Each pass rebases segment_start_wall to 'now' on loop, so 'now' must keep advancing
        # by a full pass's worth of time - reusing the same 'now' would read as 0s elapsed.
        now = 0.0
        for _ in range(2):
            now += 1.0
            _, looped, finished = player.advance(now)
            assert looped is True
            assert finished is False
        now += 1.0
        _, looped, finished = player.advance(now)
        assert looped is False
        assert finished is True
        assert player.loops_completed == 3

    def test_missing_joint_in_sample_still_returns_the_sample(self):
        """Joint-name reconciliation against required output joints is the node's job, not
        BagPlayer's - it just hands back whatever sample is due."""
        player = BagPlayer(replay_loops=1)
        player.messages = [(0.0, JointState(name=['only_one_joint'], position=[0.0]))]
        player.start(0.0)
        sample, _, _ = player.advance(0.0)
        assert list(sample.name) == ['only_one_joint']
