#!/usr/bin/env python3
"""Shared self-collision checking, used by joint_trajectory_bridge_node."""

from urllib.parse import urlparse
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
import numpy as np

try:
    import fcl
except ImportError as exc:
    fcl = None
    _FCL_IMPORT_ERROR = exc
else:
    _FCL_IMPORT_ERROR = None

try:
    from stl import mesh as stlmesh
except ImportError as exc:
    stlmesh = None
    _STL_IMPORT_ERROR = exc
else:
    _STL_IMPORT_ERROR = None


def _rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _resolve_package_uri(uri: str) -> str:
    parsed = urlparse(uri)
    return get_package_share_directory(parsed.netloc) + parsed.path


def _origin_xyz_rpy(origin) -> tuple:
    """Return (xyz, rpy) tuples from a URDF <origin> element, or identity if it's absent."""
    if origin is None:
        return (0, 0, 0), (0, 0, 0)
    xyz = tuple(float(x) for x in origin.get('xyz', '0 0 0').split())
    rpy = tuple(float(x) for x in origin.get('rpy', '0 0 0').split())
    return xyz, rpy


class SelfCollisionChecker:
    """Checks a candidate joint configuration for self-intersecting links."""

    # 0.001m sits between measured mesh-precision noise (<0.0008m) and the real upper/lower-arm
    # collision at full elbow extension (0.0043m).
    def __init__(self, urdf_xml: str, collision_margin: float = 0.001):
        if fcl is None:
            raise RuntimeError(
                'python-fcl is required for self-collision checking but is not installed. '
                'Install it with: pip install -r so_arm_control/requirements.txt '
                '(or disable self-collision checking via enable_self_collision_check:=false).'
            ) from _FCL_IMPORT_ERROR
        if stlmesh is None:
            raise RuntimeError(
                'numpy-stl is required for self-collision checking mesh loading but is not installed. '
                'Install it with: pip install -r so_arm_control/requirements.txt '
                '(or disable self-collision checking via enable_self_collision_check:=false).'
            ) from _STL_IMPORT_ERROR
        self._collision_margin = collision_margin
        # enable_contact + a small margin: a bare is_collision would reject on the slightest
        # mesh-precision graze even with real clearance - identical every call, so built once.
        self._request = fcl.CollisionRequest(enable_contact=True)
        root = ElementTree.fromstring(urdf_xml)

        self._link_meshes: dict[str, list] = {}  # link -> [(mesh_path, xyz, rpy), ...]
        for link in root.findall('link'):
            meshes = []
            for coll in link.findall('collision'):
                xyz, rpy = _origin_xyz_rpy(coll.find('origin'))
                mesh_el = coll.find('geometry/mesh')
                if mesh_el is not None:
                    meshes.append((_resolve_package_uri(mesh_el.get('filename')), xyz, rpy))
            if meshes:
                self._link_meshes[link.get('name')] = meshes

        self._joints: dict[str, dict] = {}  # name -> {parent, child, xyz, rpy, axis, type, limit}
        self._children_of: dict[str, list] = {}
        for joint in root.findall('joint'):
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            xyz, rpy = _origin_xyz_rpy(joint.find('origin'))
            axis_el = joint.find('axis')
            if axis_el is not None:
                axis = tuple(float(x) for x in axis_el.get('xyz').split())
            else:
                axis = (1, 0, 0)
            limit_el = joint.find('limit')
            if limit_el is not None:
                limit = (float(limit_el.get('lower')), float(limit_el.get('upper')))
            else:
                limit = None
            self._joints[joint.get('name')] = {
                'parent': parent, 'child': child, 'xyz': xyz, 'rpy': rpy,
                'axis': axis, 'type': joint.get('type'), 'limit': limit,
            }
            self._children_of.setdefault(parent, []).append(joint.get('name'))

        all_children = {j['child'] for j in self._joints.values()}
        all_parents = {j['parent'] for j in self._joints.values()}
        self._root_link = next(iter(all_parents - all_children))

        # frozenset(parent, child) -> joint name - an adjacent pair's collision status depends
        # only on that one joint, so it can be swept exactly across its range.
        adjacent_joint = {
            frozenset((j['parent'], j['child'])): jname for jname, j in self._joints.items()
        }

        self._link_models: dict[str, fcl.BVHModel] = {}
        # (local-frame center, radius) per link - a bounding-sphere reject before the far more
        # expensive mesh-vs-mesh FCL test (see _collide).
        self._link_bounds: dict[str, tuple] = {}
        for link, meshes in self._link_meshes.items():
            model = fcl.BVHModel()
            model.beginModel()
            all_verts = []
            for mesh_path, xyz, rpy in meshes:
                R, t = _rpy_to_R(*rpy), np.array(xyz)
                verts = stlmesh.Mesh.from_file(mesh_path).vectors.reshape(-1, 3) @ R.T + t
                for i in range(0, len(verts), 3):
                    model.addTriangle(verts[i], verts[i + 1], verts[i + 2])
                all_verts.append(verts)
            model.endModel()
            self._link_models[link] = model
            all_verts = np.concatenate(all_verts, axis=0)
            center = (all_verts.min(axis=0) + all_verts.max(axis=0)) / 2.0
            radius = float(np.linalg.norm(all_verts - center, axis=1).max())
            self._link_bounds[link] = (center, radius)

        # Persistent per-link CollisionObjects (transform updated in place) - rebuilding one
        # fresh per check() call costs about as much as the collision test itself.
        self._link_objects: dict[str, fcl.CollisionObject] = {
            link: fcl.CollisionObject(model, fcl.Transform())
            for link, model in self._link_models.items()
        }

        links = list(self._link_models)
        candidate_pairs = [(a, b) for i, a in enumerate(links) for b in links[i + 1:]]

        # Pairs already touching at rest are a valid touch, not a collision - checked once since
        # non-adjacent links only get close by coincidence, not as a function of one joint.
        rest_pose = {
            name: min(max(0.0, j['limit'][0]), j['limit'][1]) if j['limit'] else 0.0
            for name, j in self._joints.items()
        }
        non_adjacent = [
            (a, b) for a, b in candidate_pairs if frozenset((a, b)) not in adjacent_joint
        ]
        always_touching_at_rest = set(self._collide(rest_pose, non_adjacent))

        self.checked_pairs = []
        for a, b in candidate_pairs:
            jname = adjacent_joint.get(frozenset((a, b)))
            if jname is None:
                if (a, b) not in always_touching_at_rest:
                    self.checked_pairs.append((a, b))
                continue
            # Adjacent pair: a rest-pose check alone would wrongly exclude a pair that only
            # collides once its joint bends far enough - sweep its full limit range instead.
            limit = self._joints[jname]['limit']
            if limit is None:
                self.checked_pairs.append((a, b))  # no range info - always check, safest default
                continue
            lo, hi = limit
            if any(self._collide({jname: angle}, [(a, b)]) for angle in np.linspace(lo, hi, 40)):
                self.checked_pairs.append((a, b))

        # Joint path from root to each link, for joints_between() - freezing only joints
        # strictly between two links (not every commanded joint) avoids over-blocking.
        self._joint_path_to: dict[str, list] = {self._root_link: []}
        frontier = [self._root_link]
        while frontier:
            parent = frontier.pop()
            for jname in self._children_of.get(parent, []):
                child = self._joints[jname]['child']
                self._joint_path_to[child] = self._joint_path_to[parent] + [jname]
                frontier.append(child)

        # Static per checked_pairs entry - precomputed once so callers don't recompute it on
        # every round of every message (see joint_trajectory_bridge_node's retry loop).
        self.joints_between_cache = {
            (a, b): self.joints_between(a, b) for a, b in self.checked_pairs
        }

    def joints_between(self, link_a: str, link_b: str) -> set:
        """Joint names on the kinematic path between two links (their relative-pose DOF)."""
        path_a = self._joint_path_to.get(link_a, [])
        path_b = self._joint_path_to.get(link_b, [])
        i = 0
        while i < len(path_a) and i < len(path_b) and path_a[i] == path_b[i]:
            i += 1
        return set(path_a[i:]) | set(path_b[i:])

    def _forward_kinematics(self, joint_values: dict) -> dict:
        """joint_values: joint name -> angle (rad). Returns link name -> (R, t) world transform."""
        transforms = {self._root_link: (np.eye(3), np.zeros(3))}
        frontier = [self._root_link]
        while frontier:
            parent = frontier.pop()
            R_p, t_p = transforms[parent]
            for jname in self._children_of.get(parent, []):
                j = self._joints[jname]
                R_j = _rpy_to_R(*j['rpy'])
                if j['type'] in ('revolute', 'continuous'):
                    axis = np.array(j['axis'])
                    angle = joint_values.get(jname, 0.0)
                    K = np.array([
                        [0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0],
                    ])
                    R_j = R_j @ (np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K))
                R_c, t_c = R_p @ R_j, t_p + R_p @ np.array(j['xyz'])
                transforms[j['child']] = (R_c, t_c)
                frontier.append(j['child'])
        return transforms

    def _collide(self, joint_values: dict, pairs: list[tuple[str, str]]) -> list[tuple[str, str]]:
        transforms = self._forward_kinematics(joint_values)
        links = {link for pair in pairs for link in pair}
        for link in links:
            self._link_objects[link].setTransform(fcl.Transform(*transforms[link]))
        world_centers = {
            link: transforms[link][1] + transforms[link][0] @ self._link_bounds[link][0]
            for link in links
        }
        colliding = []
        for a, b in pairs:
            # Cheap bounding-sphere reject before the expensive mesh-vs-mesh FCL test.
            ra, rb = self._link_bounds[a][1], self._link_bounds[b][1]
            center_dist = np.linalg.norm(world_centers[a] - world_centers[b])
            if center_dist > ra + rb + self._collision_margin:
                continue
            result = fcl.CollisionResult()
            fcl.collide(self._link_objects[a], self._link_objects[b], self._request, result)
            if result.is_collision:
                depth = max((c.penetration_depth for c in result.contacts), default=0.0)
                if depth > self._collision_margin:
                    colliding.append((a, b))
        return colliding

    def check(
        self, joint_values: dict, pairs: list[tuple[str, str]] | None = None,
    ) -> list[tuple[str, str]]:
        """Return the (link_a, link_b) pairs colliding; checks `pairs` or every tracked pair."""
        return self._collide(joint_values, pairs if pairs is not None else self.checked_pairs)
