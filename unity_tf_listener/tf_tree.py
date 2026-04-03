"""
tf_tree.py — Builds and maintains the TF frame graph.
Computes world-space transforms by chaining through the tree.
"""

from typing import Dict, List, Optional, Tuple

from unity_tf_listener.ros_message import TransformStamped, Vector3, Quaternion, Transform


def quat_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """Hamilton product q1 * q2."""
    x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y
    y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x
    z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
    return Quaternion(x, y, z, w)


def rotate_vector(q: Quaternion, v: Vector3) -> Vector3:
    """Rotate vector v by quaternion q."""
    # v' = q * (0,v) * q^-1
    qv = Quaternion(v.x, v.y, v.z, 0.0)
    q_inv = Quaternion(-q.x, -q.y, -q.z, q.w)
    r = quat_multiply(quat_multiply(q, qv), q_inv)
    return Vector3(r.x, r.y, r.z)


def compose_transforms(parent: Transform, child: Transform) -> Transform:
    """Compose two transforms: result = parent * child."""
    # Rotated child translation
    rotated_t = rotate_vector(parent.rotation, child.translation)
    new_t = Vector3(
        parent.translation.x + rotated_t.x,
        parent.translation.y + rotated_t.y,
        parent.translation.z + rotated_t.z,
    )
    new_r = quat_multiply(parent.rotation, child.rotation)
    return Transform(translation=new_t, rotation=new_r)


IDENTITY = Transform(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1))


class TFTree:
    """
    Maintains the TF frame graph and computes world-space transforms.
    """

    def __init__(self):
        # child -> (parent, transform_from_parent_to_child)
        self._edges: Dict[str, Tuple[str, Transform]] = {}
        self._all_frames: set = set()

    def update(self, transforms: Dict[str, Dict[str, TransformStamped]]):
        """Update graph from {parent: {child: TransformStamped}}."""
        for parent, children in transforms.items():
            self._all_frames.add(parent)
            for child, ts in children.items():
                self._all_frames.add(child)
                self._edges[child] = (parent, ts.transform)

    def roots(self) -> List[str]:
        """Frames with no parent (root frames)."""
        return [f for f in self._all_frames if f not in self._edges]

    def frames(self) -> List[str]:
        return sorted(self._all_frames)

    def world_transform(self, frame: str) -> Transform:
        """
        Compute transform from world (root) to the given frame by
        chaining up the tree.
        """
        chain: List[Transform] = []
        current = frame
        visited = set()
        while current in self._edges:
            if current in visited:
                break  # cycle guard
            visited.add(current)
            parent, t = self._edges[current]
            chain.append(t)
            current = parent

        # Compose from root down to frame
        result = IDENTITY
        for t in reversed(chain):
            result = compose_transforms(result, t)
        return result

    def parent_of(self, frame: str) -> Optional[str]:
        if frame in self._edges:
            return self._edges[frame][0]
        return None

    def children_of(self, frame: str) -> List[str]:
        return [child for child, (parent, _) in self._edges.items() if parent == frame]

    def edge_transform(self, child: str) -> Optional[Transform]:
        """Local transform from parent to child."""
        if child in self._edges:
            return self._edges[child][1]
        return None
