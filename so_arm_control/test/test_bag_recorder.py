#!/usr/bin/env python3
"""Unit tests for BagRecorder - no ROS node needed, but exercises the real rosbag2_py mcap
round trip (tmp_path keeps it isolated) since that's the whole point of this class."""

import os

from so_arm_control.so_arm_utils.bag_recorder import BagRecorder

_TOPICS = [
    (0, '/joint_states', 'sensor_msgs/msg/JointState'),
    (1, '/dynamic_joint_states', 'control_msgs/msg/DynamicJointState'),
    (2, '/tf', 'tf2_msgs/msg/TFMessage'),
]


class TestStartStop:

    def test_start_creates_a_bag_directory(self, tmp_path):
        recorder = BagRecorder(str(tmp_path), _TOPICS)
        ok, message = recorder.start()
        assert ok is True
        assert recorder.is_recording is True
        assert os.path.isdir(recorder.bag_dir)
        assert 'Recording to' in message

    def test_stop_returns_bag_dir_and_writes_metadata(self, tmp_path):
        recorder = BagRecorder(str(tmp_path), _TOPICS)
        recorder.start()
        bag_dir = recorder.bag_dir
        returned = recorder.stop()
        assert returned == bag_dir
        assert os.path.isfile(os.path.join(bag_dir, 'metadata.yaml'))
        assert recorder.is_recording is False
        assert recorder.bag_dir is None

    def test_stop_when_never_started_is_a_noop(self, tmp_path):
        recorder = BagRecorder(str(tmp_path), _TOPICS)
        assert recorder.stop() is None
        assert recorder.is_recording is False

    def test_start_fails_gracefully_when_recordings_dir_cannot_be_created(self):
        # A path under /dev/null can never be created - os.makedirs must fail with OSError.
        recorder = BagRecorder('/dev/null/not-a-real-dir', _TOPICS)
        ok, message = recorder.start()
        assert ok is False
        assert recorder.is_recording is False


class TestWrite:

    def test_writes_forward_to_the_open_writer(self, tmp_path):
        recorder = BagRecorder(str(tmp_path), _TOPICS)
        recorder.start()
        # No exception is the assertion here - actual write-verification happens via a real
        # SequentialReader in test_bag_player.py's TestLoadLatest tests.
        recorder.write('/joint_states', b'not-real-cdr-bytes', 0)
        recorder.stop()

    def test_write_before_start_is_a_noop(self, tmp_path):
        recorder = BagRecorder(str(tmp_path), _TOPICS)
        recorder.write('/joint_states', b'ignored', 0)  # must not raise
