#!/usr/bin/env python3
"""Parse per-joint velocity limits and position limits from a rendered robot_description URDF."""

import xml.etree.ElementTree as ElementTree


def parse_joint_velocity_and_limits(
    urdf_xml: str, joint_names: list[str] | None,
) -> tuple[dict[str, float], dict[str, tuple[float, float]]]:
    """Return (max_velocity, limits) parsed from urdf_xml.

    max_velocity prefers ros2_control's own <param name="max_velocity"> (the real, servo-derived
    ceiling) over the kinematic <joint>'s own <limit velocity="..."> (a dummy placeholder in this
    project's URDFs - see so_arm_description's *.urdf.xacro comments). limits is each joint's
    (lower, upper) position range, from its kinematic <joint>'s own <limit>.
    joint_names=None parses every joint found; otherwise only the given names are included.
    """
    root = ElementTree.fromstring(urdf_xml)

    limits: dict[str, tuple[float, float]] = {}
    max_velocity: dict[str, float] = {}
    for joint_elem in root.findall('joint'):
        name = joint_elem.get('name')
        if joint_names is not None and name not in joint_names:
            continue
        limit_elem = joint_elem.find('limit')
        if limit_elem is None:
            continue
        lower, upper = limit_elem.get('lower'), limit_elem.get('upper')
        if lower is not None and upper is not None:
            limits[name] = (float(lower), float(upper))
        velocity = limit_elem.get('velocity')
        if velocity is not None:
            max_velocity[name] = float(velocity)

    for ros2_control_elem in root.findall('ros2_control'):
        for joint_elem in ros2_control_elem.findall('joint'):
            name = joint_elem.get('name')
            if joint_names is not None and name not in joint_names:
                continue
            for param_elem in joint_elem.findall('param'):
                if param_elem.get('name') == 'max_velocity' and param_elem.text:
                    max_velocity[name] = float(param_elem.text)

    return max_velocity, limits
