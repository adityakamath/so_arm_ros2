#!/usr/bin/env python3
"""Owns the rosbag2_py SequentialWriter lifecycle for a fixed set of raw-passthrough topics.

No ROS Node dependency - the caller supplies raw CDR bytes and a timestamp per write() call,
so this is directly unit-testable without a live node.
"""

from datetime import datetime
import os

import rosbag2_py


class BagRecorder:
    """Opens/writes/closes an mcap bag under a timestamped subdirectory of recordings_dir."""

    def __init__(self, recordings_dir: str, topics: list[tuple[int, str, str]]):
        """`topics` is [(id, name, type), ...], passed straight to rosbag2_py.TopicMetadata."""
        self._recordings_dir = recordings_dir
        self._topics = topics
        self._writer = None
        self.bag_dir: str | None = None

    @property
    def is_recording(self) -> bool:
        return self._writer is not None

    def start(self) -> tuple[bool, str]:
        try:
            os.makedirs(self._recordings_dir, exist_ok=True)
        except OSError as exc:
            return False, f"Failed to create recordings_dir '{self._recordings_dir}': {exc}"
        stamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        bag_dir = os.path.join(self._recordings_dir, stamp)

        writer = rosbag2_py.SequentialWriter()
        try:
            writer.open(
                rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'),
            )
        except RuntimeError as exc:
            return False, f"Failed to open bag at '{bag_dir}': {exc}"

        for topic_id, name, msg_type in self._topics:
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=topic_id, name=name, type=msg_type, serialization_format='cdr',
            ))

        self._writer = writer
        self.bag_dir = bag_dir
        return True, f"Recording to '{bag_dir}'"

    def write(self, topic: str, raw: bytes, timestamp: int) -> None:
        if self._writer is None:
            return
        self._writer.write(topic, raw, timestamp)

    def stop(self) -> str | None:
        """Close the writer if open; return the bag_dir that was being recorded, or None."""
        if self._writer is None:
            return None
        self._writer.close()
        bag_dir = self.bag_dir
        self._writer = None
        self.bag_dir = None
        return bag_dir
