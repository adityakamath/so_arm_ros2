#!/usr/bin/env python3
"""Unit tests for opencv_camera_node.py's pure helpers and node-level publish logic. See
conftest.py for the numpy/cv2 ABI import-order constraint this file must run under.

_CameraCapture (a real cv2.VideoCapture) isn't tested here - no camera hardware in CI; a fake
stands in for it (mirroring test_record_replay_node.py's _FakeRecorder) to test _on_timer."""

import itertools

# Must precede rclpy/sensor_msgs below - those pull in numpy 2.x, which breaks cv2's ABI
# expectations the same way conftest.py's own comment documents.
from so_arm_bringup.opencv_camera_node import (
    OpenCVCameraNode, _BACKENDS, _load_camera_info, _parse_index_or_path,
)
import cv2
from cv_bridge import CvBridge
import numpy as np
import pytest
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import CameraInfo

_name_counter = itertools.count()


class TestParseIndexOrPath:

    def test_digit_string_becomes_int_index(self):
        assert _parse_index_or_path('0') == 0
        assert _parse_index_or_path('2') == 2

    def test_negative_digit_string_becomes_int(self):
        assert _parse_index_or_path('-1') == -1

    def test_device_path_stays_a_string(self):
        assert _parse_index_or_path('/dev/video0') == '/dev/video0'


class TestBackends:

    def test_empty_and_any_both_map_to_cap_any(self):
        assert _BACKENDS[''] == cv2.CAP_ANY
        assert _BACKENDS['any'] == cv2.CAP_ANY

    def test_v4l2_and_avfoundation_map_to_their_own_constants(self):
        assert _BACKENDS['v4l2'] == cv2.CAP_V4L2
        assert _BACKENDS['avfoundation'] == cv2.CAP_AVFOUNDATION


class TestLoadCameraInfo:

    def test_empty_path_returns_uncalibrated_info(self):
        info = _load_camera_info('', 640, 480, 'camera_optical_frame')
        assert info.width == 640
        assert info.height == 480
        assert list(info.d) == []
        assert list(info.k) == [0.0] * 9

    def test_missing_file_returns_uncalibrated_info(self, tmp_path):
        info = _load_camera_info(str(tmp_path / 'does_not_exist.yaml'), 640, 480, 'x')
        assert info.width == 640
        assert list(info.d) == []

    def test_valid_calibration_file_is_parsed(self, tmp_path):
        path = tmp_path / 'calib.yaml'
        path.write_text("""
image_width: 1920
image_height: 1080
distortion_model: plumb_bob
camera_matrix:
  rows: 3
  cols: 3
  data: [1000.0, 0.0, 960.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1, 0.2, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [1000.0, 0.0, 960.0, 0.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 0.0, 1.0, 0.0]
""")
        info = _load_camera_info(str(path), 640, 480, 'x')
        assert info.width == 1920
        assert info.height == 1080
        assert info.distortion_model == 'plumb_bob'
        assert list(info.d) == pytest.approx([0.1, 0.2, 0.0, 0.0, 0.0])
        assert list(info.k) == pytest.approx(
            [1000.0, 0.0, 960.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 1.0]
        )

    def test_missing_keys_in_a_present_file_fall_back_to_defaults(self, tmp_path):
        """A partial/malformed calibration file shouldn't crash - just leave those fields
        at their uncalibrated defaults."""
        path = tmp_path / 'partial.yaml'
        path.write_text('image_width: 1920\nimage_height: 1080\n')
        info = _load_camera_info(str(path), 640, 480, 'x')
        assert info.width == 1920
        assert list(info.d) == []
        assert list(info.k) == [0.0] * 9


class _FakeCapture:
    """Mirrors _CameraCapture's public surface without touching real hardware."""

    def __init__(self, frame=None, stamp=0.0):
        self.frame = frame
        self.stamp = stamp
        self.closed = False

    def read_latest(self):
        return self.frame, self.stamp

    def close(self):
        self.closed = True


def _make_node():
    node = OpenCVCameraNode.__new__(OpenCVCameraNode)
    RclpyNode.__init__(node, f'test_opencv_camera_{next(_name_counter)}')
    node._frame_id = 'wrist_camera_optical_frame'
    node._bridge = CvBridge()
    node._camera_info = CameraInfo(width=640, height=480)
    node._camera_info.header.frame_id = node._frame_id
    node._capture = _FakeCapture()

    published_images = []
    published_infos = []
    node._image_pub = _Publisher(published_images)
    node._camera_info_pub = _Publisher(published_infos)
    node.published_images = published_images
    node.published_infos = published_infos
    return node


class _Publisher:
    def __init__(self, sink):
        self._sink = sink

    def publish(self, msg):
        self._sink.append(msg)


@pytest.fixture
def node():
    n = _make_node()
    yield n
    n.destroy_node()


class TestOnTimer:

    def test_no_frame_yet_publishes_nothing(self, node):
        node._on_timer()
        assert node.published_images == []
        assert node.published_infos == []

    def test_publishes_image_and_camera_info_when_frame_available(self, node):
        node._capture.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        node._on_timer()
        assert len(node.published_images) == 1
        assert len(node.published_infos) == 1

    def test_published_image_has_the_configured_frame_id(self, node):
        node._capture.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        node._on_timer()
        assert node.published_images[0].header.frame_id == 'wrist_camera_optical_frame'

    def test_camera_info_stamp_matches_image_stamp(self, node):
        """image_proc-style consumers correlate Image/CameraInfo by matching stamps."""
        node._capture.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        node._on_timer()
        img_stamp = node.published_images[0].header.stamp
        info_stamp = node.published_infos[0].header.stamp
        assert (img_stamp.sec, img_stamp.nanosec) == (info_stamp.sec, info_stamp.nanosec)

    def test_repeated_ticks_with_no_new_frame_republish_the_same_frame(self, node):
        """_capture.read_latest() always returns whatever's current - on_timer doesn't track
        whether it's actually new, it just republishes the latest known frame each tick."""
        node._capture.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        node._on_timer()
        node._on_timer()
        assert len(node.published_images) == 2


class TestDestroyNode:

    def test_closes_the_capture(self, node):
        node.destroy_node()
        assert node._capture.closed is True
