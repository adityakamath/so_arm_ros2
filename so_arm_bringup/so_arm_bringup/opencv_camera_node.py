#!/usr/bin/env python3
"""Generic OpenCV-`VideoCapture`-based camera driver, not wrist-specific in its own code - see
wrist_camera.launch.py/wrist_camera.yaml for this repo's one deployment of it.

Cross-platform via cv2.VideoCapture (V4L2 on Linux, AVFoundation on macOS) instead of
usb_cam/v4l2_camera, which are Linux-only and unavailable on Pixi for osx-arm64.

Shape follows LeRobot's own OpenCVCamera: a daemon thread keeps `latest_frame` current; the
ROS publish timer just reads it non-blocking, instead of calling cv2.VideoCapture.read() itself.

No image_transport/camera_info_manager (neither has Python bindings in this ROS 2 install) -
Image publishes raw on `image_raw` (the topic name image_transport itself expects); CameraInfo
is parsed directly from the standard camera_calibration YAML format.
"""

import sys
import threading
import time

# cv2 needs numpy 1.x ABI - drop user-site numpy 2.x before importing (same fix kinematics.py
# uses for pinocchio). Must be the first non-stdlib import in this file.
sys.path = [p for p in sys.path if '/.local/lib/' not in p]

import cv2  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from so_arm_control.so_arm_utils.params import require_parameter  # noqa: E402
from so_arm_control.so_arm_utils.spin import spin_and_shutdown  # noqa: E402
import yaml  # noqa: E402

_BACKENDS = {
    '': cv2.CAP_ANY,
    'any': cv2.CAP_ANY,
    'v4l2': cv2.CAP_V4L2,
    'avfoundation': cv2.CAP_AVFOUNDATION,
    'dshow': cv2.CAP_DSHOW,
    'msmf': cv2.CAP_MSMF,
}


def _parse_index_or_path(value: str) -> int | str:
    """A bare integer string ('0') opens by device index; anything else is a path ('/dev/video0')."""
    return int(value) if value.strip().lstrip('-').isdigit() else value


def _load_camera_info(path: str, width: int, height: int, frame_id: str) -> CameraInfo:
    """Parse camera_info_manager's calibration YAML format into a CameraInfo (not the C++
    library itself, just its file format). Falls back to uncalibrated (D=[], K/R/P zero) if
    `path` is empty or unreadable."""
    info = CameraInfo(width=width, height=height)
    if not path:
        return info
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
    except OSError:
        return info

    info.width = data.get('image_width', width)
    info.height = data.get('image_height', height)
    info.distortion_model = data.get('distortion_model', '')
    info.d = list(data.get('distortion_coefficients', {}).get('data', []))
    info.k = list(data.get('camera_matrix', {}).get('data', info.k))
    info.r = list(data.get('rectification_matrix', {}).get('data', info.r))
    info.p = list(data.get('projection_matrix', {}).get('data', info.p))
    return info


class _CameraCapture:
    """Owns a cv2.VideoCapture and a daemon thread that keeps `latest_frame` current. No ROS
    Node dependency (mirrors bag_player.py/bag_recorder.py), so it's unit-testable standalone."""

    def __init__(
        self, index_or_path: int | str, backend: int, width: int, height: int, fps: float,
        fourcc: str,
    ):
        self._cap = cv2.VideoCapture(index_or_path, backend)
        if width:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps:
            self._cap.set(cv2.CAP_PROP_FPS, fps)
        if fourcc:
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))

        if not self._cap.isOpened():
            raise RuntimeError(f"Failed to open camera '{index_or_path}' (backend={backend})")

        self._lock = threading.Lock()
        self.latest_frame = None  # BGR numpy array, or None until the first frame lands
        self.latest_stamp = 0.0  # time.time() of the last successful read
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok:
                continue
            with self._lock:
                self.latest_frame = frame
                self.latest_stamp = time.time()

    def read_latest(self):
        """Non-blocking: whatever's current, or (None, 0.0) if nothing has landed yet."""
        with self._lock:
            return self.latest_frame, self.latest_stamp

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._cap.release()


class OpenCVCameraNode(Node):

    def __init__(self, parameter_overrides: list | None = None):
        super().__init__('opencv_camera_node', parameter_overrides=parameter_overrides)

        self.declare_parameter('frame_id', Parameter.Type.STRING)
        self.declare_parameter('index_or_path', '0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('fourcc', '')
        self.declare_parameter('backend', '')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('warmup_s', 1.0)

        self._capture = None
        self._hardware_missing = False

        frame_id = require_parameter(self, 'frame_id')
        index_or_path = _parse_index_or_path(self.get_parameter('index_or_path').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        fourcc = self.get_parameter('fourcc').value
        backend_name = self.get_parameter('backend').value.lower()
        if backend_name not in _BACKENDS:
            raise RuntimeError(
                f"Unknown backend '{backend_name}' - one of {sorted(_BACKENDS)}"
            )
        camera_info_url = self.get_parameter('camera_info_url').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        warmup_s = float(self.get_parameter('warmup_s').value)

        self._frame_id = frame_id
        self._bridge = CvBridge()
        try:
            capture = _CameraCapture(
                index_or_path, _BACKENDS[backend_name], width, height, fps, fourcc,
            )
        except RuntimeError as exc:
            self.get_logger().warning(
                f"No camera at '{index_or_path}' ({exc}). Shutting down without publishing "
                f'image_raw - set wrist_camera:=false if this robot has no camera fitted.'
            )
            self._hardware_missing = True
            return
        self._capture = capture
        self._camera_info = _load_camera_info(camera_info_url, width, height, frame_id)
        self._camera_info.header.frame_id = frame_id

        self._image_pub = self.create_publisher(Image, image_topic, 10)
        self._camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)

        self._capture.start()
        if warmup_s > 0:
            time.sleep(warmup_s)  # let auto-exposure/auto-white-balance settle before publishing

        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)
        self.get_logger().info(
            f"opencv_camera_node ready: '{index_or_path}' -> '{image_topic}' "
            f'({width}x{height}@{fps}, frame_id={frame_id})'
        )

    @property
    def hardware_missing(self) -> bool:
        return self._hardware_missing

    def _on_timer(self) -> None:
        frame, stamp = self._capture.read_latest()
        if frame is None:
            self.get_logger().warning('No frame available yet', throttle_duration_sec=5.0)
            return

        header_stamp = self.get_clock().now().to_msg()
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = header_stamp
        msg.header.frame_id = self._frame_id
        self._image_pub.publish(msg)

        self._camera_info.header.stamp = header_stamp
        self._camera_info_pub.publish(self._camera_info)

    def destroy_node(self) -> bool:
        if self._capture is not None:
            self._capture.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenCVCameraNode()
    if node.hardware_missing:
        # No error, no traceback - the warning already logged in __init__ says why.
        node.destroy_node()
        rclpy.try_shutdown()
        return
    spin_and_shutdown(node)


if __name__ == '__main__':
    main()
