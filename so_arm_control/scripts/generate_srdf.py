#!/usr/bin/env python3
"""
Generate config/{model}.srdf's <disable_collisions> entries.

Derived from SelfCollisionChecker's own adjacency/rest-pose analysis - keeps the SRDF consistent
with the FCL-based runtime checker instead of an independent guess. Re-run whenever the URDF's
collision geometry changes.

Usage: python3 generate_srdf.py <model> <urdf_path> <output_srdf_path>
"""
import sys

from so_arm_control.so_arm_utils.collision import SelfCollisionChecker

SRDF_TEMPLATE = """<?xml version="1.0" encoding="UTF-8" ?>
<!--This does not replace URDF, and is not an extension of URDF.
    This is a format for representing semantic information about the robot structure.
    A URDF file must exist for this robot as well, where the joints and the links that are
    referenced are defined
-->
<robot name="{model}">
  <group name="manipulator">
    <chain base_link="base_link" tip_link="end_effector_link" />
  </group>
{disable_collisions}</robot>
"""


def generate(model: str, urdf_path: str, output_path: str) -> None:
    with open(urdf_path) as f:
        urdf_xml = f.read()
    checker = SelfCollisionChecker(urdf_xml)

    # Local recompute (checker doesn't expose this mapping itself) - frozenset(parent, child) ->
    # joint name, matching SelfCollisionChecker's own adjacency test.
    adjacent_joint = {
        frozenset((j['parent'], j['child'])): jname for jname, j in checker._joints.items()
    }

    links = sorted(checker._link_models)
    candidate_pairs = [(a, b) for i, a in enumerate(links) for b in links[i + 1:]]
    # Both orderings, since checked_pairs' own tuple order isn't guaranteed to match this
    # script's independently-sorted candidate_pairs.
    checked = {pair for a, b in checker.checked_pairs for pair in ((a, b), (b, a))}

    lines = []
    for a, b in candidate_pairs:
        if (a, b) in checked:
            continue  # still needs runtime checking - leave it enabled
        jname = adjacent_joint.get(frozenset((a, b)))
        reason = 'Adjacent' if jname else 'Default'
        lines.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="{reason}" />')

    srdf = SRDF_TEMPLATE.format(
        model=model, disable_collisions='\n'.join(lines) + '\n' if lines else '',
    )
    with open(output_path, 'w') as f:
        f.write(srdf)
    print(
        f'{model}: {len(checker.checked_pairs)} pairs left enabled, '
        f'{len(lines)} disabled -> {output_path}'
    )


if __name__ == '__main__':
    generate(*sys.argv[1:4])
