#!/usr/bin/env python3
"""The `namespace` launch option shared by every so_arm launch file.

namespace:=leader pushes every node into /leader and, unless frame_prefix is given explicitly,
sets the tf frame prefix to "leader/" - the same convention leader_follower.launch.py uses when it
pushes namespaces itself (those launches leave `namespace` empty, so nothing is pushed twice).
"""

from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace


def namespace_arg() -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        'namespace', default_value='',
        description=(
            'ROS namespace for every node (e.g. "leader"); empty = none. Also derives '
            'frame_prefix ("<namespace>/") when that is left empty.'
        ),
    )


def resolve_frame_prefix(namespace: str, frame_prefix: str) -> str:
    """An explicit frame_prefix wins; otherwise "<namespace>/" if a namespace is set."""
    ns = namespace.strip().strip('/')
    return frame_prefix.strip() or (f'{ns}/' if ns else '')


def in_namespace(namespace: str, actions: list) -> list:
    """Wrap `actions` in the namespace; a no-op when `namespace` is empty."""
    ns = namespace.strip().strip('/')
    return [GroupAction([PushRosNamespace(ns), *actions])] if ns else list(actions)
