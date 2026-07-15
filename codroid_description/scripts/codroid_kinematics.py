"""Small URDF-based damped least-squares IK solver for the CoDroid arms."""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass(frozen=True)
class Joint:
    name: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


def _rotation_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _transform(xyz: List[float], rpy: List[float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = _rotation_rpy(*rpy)
    result[:3, 3] = xyz
    return result


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array([
        [x*x*v+c, x*y*v-z*s, x*z*v+y*s],
        [y*x*v+z*s, y*y*v+c, y*z*v-x*s],
        [z*x*v-y*s, z*y*v+x*s, z*z*v+c],
    ])


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError('invalid quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


class ArmKinematics:
    def __init__(self, robot_description: str, joint_names: List[str]) -> None:
        root = ET.fromstring(robot_description)
        elements = {joint.get('name'): joint for joint in root.findall('joint')}
        joints = []
        for name in joint_names:
            element = elements.get(name)
            if element is None:
                raise ValueError(f'URDF missing joint {name}')
            origin = element.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0').split())]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0').split())]
            axis = np.array([float(v) for v in element.find('axis').get('xyz').split()])
            limit = element.find('limit')
            joints.append(Joint(
                name, _transform(xyz, rpy), axis,
                float(limit.get('lower')), float(limit.get('upper'))))
        self.joints = joints
        self.lower = np.array([joint.lower for joint in joints])
        self.upper = np.array([joint.upper for joint in joints])
        self.center = (self.lower + self.upper) * 0.5

    def forward_jacobian(self, positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        transform = np.eye(4)
        joint_points, joint_axes = [], []
        for joint, angle in zip(self.joints, positions):
            transform = transform @ joint.origin
            joint_points.append(transform[:3, 3].copy())
            joint_axes.append(transform[:3, :3] @ joint.axis)
            rotation = np.eye(4)
            rotation[:3, :3] = _axis_rotation(joint.axis, float(angle))
            transform = transform @ rotation
        end_position = transform[:3, 3]
        jacobian = np.zeros((6, len(self.joints)))
        for index, (point, axis) in enumerate(zip(joint_points, joint_axes)):
            jacobian[:3, index] = np.cross(axis, end_position - point)
            jacobian[3:, index] = axis
        return transform, jacobian

    @staticmethod
    def _orientation_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
        # World-frame SO(3) error, stable for iterative small steps.
        return 0.5 * (
            np.cross(current[:, 0], target[:, 0])
            + np.cross(current[:, 1], target[:, 1])
            + np.cross(current[:, 2], target[:, 2]))

    def inverse(
        self, position: List[float], rotation: np.ndarray, seed: List[float],
        *, max_iterations: int = 250, damping: float = 0.03,
        position_tolerance: float = 2e-4, orientation_tolerance: float = 2e-3,
    ) -> List[float]:
        target_position = np.asarray(position, dtype=float)
        q = np.clip(np.asarray(seed, dtype=float), self.lower, self.upper)
        if target_position.shape != (3,) or q.shape != (7,):
            raise ValueError('IK requires a 3D position and 7-joint seed')
        for _ in range(max_iterations):
            current, jacobian = self.forward_jacobian(q)
            position_error = target_position - current[:3, 3]
            orientation_error = self._orientation_error(rotation, current[:3, :3])
            if (np.linalg.norm(position_error) <= position_tolerance
                    and np.linalg.norm(orientation_error) <= orientation_tolerance):
                return q.tolist()
            error = np.concatenate((position_error, orientation_error))
            regularized = jacobian @ jacobian.T + damping * damping * np.eye(6)
            primary = jacobian.T @ np.linalg.solve(regularized, error)
            pseudo_inverse = jacobian.T @ np.linalg.solve(regularized, np.eye(6))
            nullspace = np.eye(7) - pseudo_inverse @ jacobian
            secondary = nullspace @ (0.04 * (self.center - q))
            step = primary + secondary
            largest = float(np.max(np.abs(step)))
            if largest > 0.12:
                step *= 0.12 / largest
            q = np.clip(q + step, self.lower, self.upper)
        final, _ = self.forward_jacobian(q)
        pos_error = np.linalg.norm(target_position - final[:3, 3])
        rot_error = np.linalg.norm(self._orientation_error(rotation, final[:3, :3]))
        raise ValueError(
            f'local IK did not converge (position error={pos_error:.6f}m, '
            f'orientation error={rot_error:.6f}rad)')
