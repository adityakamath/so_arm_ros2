#!/usr/bin/env python3
"""Loads a recorded joint_states bag and drives timed playback through it.

No ROS Node dependency - timestamps ('now') are passed in by the caller (typically
time.monotonic()) rather than read from a clock, so this is directly unit-testable without a
live node/timer. Used by record_replay_node, the sole owner of '/replay'.
"""

import os

from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import JointState


class BagPlayer:
    """Owns loaded playback samples plus the looping/pause/resume timing state machine."""

    def __init__(self, replay_loops: int):
        # 0 = loop forever (until paused or e-stopped), N>0 = exactly N total passes.
        if replay_loops < 0:
            raise ValueError(f"replay_loops must be >= 0 (0 = loop forever), got {replay_loops}")
        self.replay_loops = replay_loops

        self.messages: list[tuple[float, JointState]] | None = None
        self.bag_dir: str | None = None
        self.index = 0
        self.elapsed_at_pause = 0.0  # seconds of playback consumed before the current segment
        # 'now' ref for the running segment; None = paused/stopped.
        self.segment_start_wall: float | None = None
        self.completed = True
        self.loops_completed = 0

    @property
    def is_running(self) -> bool:
        return self.messages is not None and self.segment_start_wall is not None

    def load_latest(self, recordings_dir: str, joint_states_topic: str) -> tuple[bool, str]:
        """Load the lexicographically-latest recording's joint_states samples."""
        if not os.path.isdir(recordings_dir):
            return False, 'No recordings found'
        candidates = sorted(
            name for name in os.listdir(recordings_dir)
            if os.path.isfile(os.path.join(recordings_dir, name, 'metadata.yaml'))
        )
        if not candidates:
            return False, 'No recordings found'
        bag_dir = os.path.join(recordings_dir, candidates[-1])

        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(
                rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'),
            )
        except RuntimeError as exc:
            return False, f"Failed to open '{bag_dir}': {exc}"
        reader.set_filter(rosbag2_py.StorageFilter(topics=[joint_states_topic]))

        raw_messages = []
        while reader.has_next():
            _, data, t = reader.read_next()
            raw_messages.append((t, deserialize_message(data, JointState)))
        reader.close()

        if not raw_messages:
            return False, f"Recording '{bag_dir}' has no {joint_states_topic} samples"

        t0 = raw_messages[0][0]
        self.messages = [((t - t0) / 1e9, msg) for t, msg in raw_messages]
        self.bag_dir = bag_dir
        return True, f"Replaying '{bag_dir}' ({len(self.messages)} samples)"

    def arm(self) -> None:
        """Reset playback position for a freshly (re)loaded recording."""
        self.index = 0
        self.elapsed_at_pause = 0.0
        self.completed = False
        self.loops_completed = 0

    def start(self, now: float) -> None:
        self.segment_start_wall = now

    def pause(self, now: float) -> None:
        self.elapsed_at_pause += now - self.segment_start_wall
        self.segment_start_wall = None

    def abort(self) -> None:
        self.segment_start_wall = None
        self.completed = True

    def finish(self) -> None:
        self.segment_start_wall = None
        self.completed = True

    def advance(self, now: float) -> tuple[JointState, bool, bool] | None:
        """
        Advance playback to `now`.

        Returns (sample, looped, finished), or None if nothing is loaded/running. `looped` is
        True on the tick where playback wraps back to the start; `finished` is True on the tick
        where the configured replay_loops count is reached (finish() has already been applied).
        """
        if not self.is_running:
            return None

        offset = self.elapsed_at_pause + (now - self.segment_start_wall)
        last_offset = self.messages[-1][0]
        while self.index + 1 < len(self.messages) and self.messages[self.index + 1][0] <= offset:
            self.index += 1

        sample = self.messages[self.index][1]
        looped = False
        finished = False
        if offset >= last_offset and self.index == len(self.messages) - 1:
            self.loops_completed += 1
            if self.replay_loops == 0 or self.loops_completed < self.replay_loops:
                # Loop back to the start - joint_state_switch_node's velocity-limited ramp
                # smooths the jump from this last sample to the recording's first sample.
                self.index = 0
                self.elapsed_at_pause = 0.0
                self.segment_start_wall = now
                looped = True
            else:
                self.finish()
                finished = True
        return sample, looped, finished
