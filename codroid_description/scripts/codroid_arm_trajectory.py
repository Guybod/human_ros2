#!/usr/bin/env python3
"""Fixed-rate, cubic joint interpolation for the CoDroid dual arms."""

import json
import math
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
import websocket
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from codroid_kinematics import ArmKinematics, quaternion_matrix


LEFT_JOINTS = [f'J_arm_l_{index:02d}' for index in range(1, 8)]
RIGHT_JOINTS = [f'J_arm_r_{index:02d}' for index in range(1, 8)]


@dataclass(frozen=True)
class Waypoint:
    time_from_start: float
    positions: Tuple[float, ...]
    velocities: Tuple[float, ...]


class CoDroidArmTrajectory(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__(
            'codroid_arm_trajectory', parameter_overrides=parameter_overrides)
        self.declare_parameter('frequency_hz', 100.0)
        self.declare_parameter('default_max_velocity', 0.25)
        self.declare_parameter('minimum_duration', 1.0)
        self.declare_parameter('state_timeout', 0.5)
        self.declare_parameter('hold_final_position', True)
        self.declare_parameter('robot_description', '')
        self.declare_parameter('robot_ip', '192.168.2.16')
        self.declare_parameter('ws_port', 9000)
        self.declare_parameter('ik_timeout', 3.0)
        self.declare_parameter('ik_solver', 'controller')
        self.declare_parameter('path_minimum_segment_duration', 0.5)

        frequency = float(self.get_parameter('frequency_hz').value)
        if frequency <= 0.0:
            raise ValueError('frequency_hz must be positive')
        self._period = 1.0 / frequency
        self._max_velocity = float(self.get_parameter('default_max_velocity').value)
        self._minimum_duration = float(self.get_parameter('minimum_duration').value)
        self._state_timeout = float(self.get_parameter('state_timeout').value)
        self._path_minimum_segment = float(
            self.get_parameter('path_minimum_segment_duration').value)
        self._hold_final = bool(self.get_parameter('hold_final_position').value)
        if (self._max_velocity <= 0.0 or self._minimum_duration <= 0.0
                or self._path_minimum_segment <= 0.0):
            raise ValueError('trajectory timing parameters must be positive')

        robot_description = str(self.get_parameter('robot_description').value)
        self._limits = self._parse_joint_limits(robot_description)
        self._kinematics = {
            'LeftArm': ArmKinematics(robot_description, LEFT_JOINTS),
            'RightArm': ArmKinematics(robot_description, RIGHT_JOINTS),
        }
        self._lock = threading.Lock()
        self._latest_state: Dict[str, float] = {}
        self._latest_state_time = 0.0
        self._joint_names: List[str] = []
        self._waypoints: List[Waypoint] = []
        self._start_time = 0.0
        self._active = False
        self._ws = None
        self._ws_lock = threading.Lock()
        self._request_id = 0
        self._pose_callback_group = MutuallyExclusiveCallbackGroup()
        self._stream_callback_group = MutuallyExclusiveCallbackGroup()

        self._command_publisher = self.create_publisher(
            JointState, '/codroid/arm_command', 1)
        self._status_publisher = self.create_publisher(
            String, '/codroid/trajectory_status', 10)
        self.create_subscription(JointState, '/joint_states', self._state_callback, 10)
        self.create_subscription(
            JointTrajectory, '/codroid/arm_trajectory', self._trajectory_callback, 1)
        self.create_subscription(
            PoseStamped, '/codroid/left_arm/pose_target',
            lambda message: self._pose_callback('LeftArm', message), 1,
            callback_group=self._pose_callback_group)
        self.create_subscription(
            PoseStamped, '/codroid/right_arm/pose_target',
            lambda message: self._pose_callback('RightArm', message), 1,
            callback_group=self._pose_callback_group)
        self.create_subscription(
            PoseArray, '/codroid/left_arm/pose_waypoints',
            lambda message: self._pose_path_callback('LeftArm', message), 1,
            callback_group=self._pose_callback_group)
        self.create_subscription(
            PoseArray, '/codroid/right_arm/pose_waypoints',
            lambda message: self._pose_path_callback('RightArm', message), 1,
            callback_group=self._pose_callback_group)
        self.create_service(Trigger, '~/cancel', self._cancel_callback)
        self.create_timer(
            self._period, self._timer_callback,
            callback_group=self._stream_callback_group)
        self.get_logger().info(
            f'Arm trajectory interpolator ready at {frequency:.1f} Hz')

    def _pose_callback(self, arm_name: str, message: PoseStamped) -> None:
        try:
            joints = self._solve_pose_ik(arm_name, message)
            trajectory = JointTrajectory()
            trajectory.joint_names = LEFT_JOINTS if arm_name == 'LeftArm' else RIGHT_JOINTS
            point = JointTrajectoryPoint()
            trajectory.points.append(point)
            point.positions = joints
            # Zero time requests automatic duration from current joint feedback.
            if not self._trajectory_callback(trajectory):
                raise ValueError('fitted joint trajectory failed safety validation')
        except (ValueError, RuntimeError, OSError, websocket.WebSocketException) as error:
            self.get_logger().error(f'{arm_name} pose target rejected: {error}')
            self._publish_status(f'rejected: {error}')

    def _pose_path_callback(self, arm_name: str, message: PoseArray) -> None:
        try:
            if message.header.frame_id not in ('', 'base_link'):
                raise ValueError('pose waypoint frame must be base_link')
            if not message.poses:
                raise ValueError('pose waypoint array is empty')
            names = LEFT_JOINTS if arm_name == 'LeftArm' else RIGHT_JOINTS
            with self._lock:
                if time.monotonic() - self._latest_state_time > self._state_timeout:
                    raise ValueError('pose path requires fresh /joint_states feedback')
                if not all(name in self._latest_state for name in names):
                    raise ValueError('pose path feedback is incomplete')
                seed = [self._latest_state[name] for name in names]

            joint_points = [list(seed)]
            for index, pose in enumerate(message.poses):
                try:
                    seed = self._solve_local_pose(arm_name, pose, seed)
                except ValueError as error:
                    raise ValueError(f'waypoint {index} IK failed: {error}') from error
                joint_points.append(list(seed))

            times = [0.0]
            for start, end in zip(joint_points, joint_points[1:]):
                max_delta = max(abs(finish - begin) for begin, finish in zip(start, end))
                duration = max(
                    self._path_minimum_segment,
                    2.0 * max_delta / self._max_velocity,
                )
                times.append(times[-1] + duration)

            velocities = [[0.0] * 7 for _ in joint_points]
            for index in range(1, len(joint_points) - 1):
                span = times[index + 1] - times[index - 1]
                velocities[index] = [
                    max(-self._max_velocity, min(
                        self._max_velocity,
                        (joint_points[index + 1][joint] - joint_points[index - 1][joint]) / span))
                    for joint in range(7)
                ]

            trajectory = JointTrajectory()
            trajectory.joint_names = names
            # Current feedback is injected by _prepare_trajectory, so publish only target points.
            for index in range(1, len(joint_points)):
                point = JointTrajectoryPoint()
                point.positions = joint_points[index]
                point.velocities = velocities[index]
                seconds = times[index]
                whole_seconds = int(seconds)
                nanoseconds = int(round((seconds - whole_seconds) * 1e9))
                if nanoseconds >= 1_000_000_000:
                    whole_seconds += 1
                    nanoseconds -= 1_000_000_000
                point.time_from_start.sec = whole_seconds
                point.time_from_start.nanosec = nanoseconds
                trajectory.points.append(point)
            if not self._trajectory_callback(trajectory):
                raise ValueError('fitted joint trajectory failed safety validation')
            self.get_logger().info(
                f'Local IK fitted path accepted for {arm_name}: '
                f'{len(message.poses)} poses, {times[-1]:.3f}s')
        except ValueError as error:
            self.get_logger().error(f'{arm_name} pose path rejected: {error}')
            self._publish_status(f'rejected: {error}')

    def _solve_pose_ik(self, arm_name: str, message: PoseStamped) -> List[float]:
        solver = str(self.get_parameter('ik_solver').value)
        if solver not in ('controller', 'local', 'controller_then_local'):
            raise ValueError(
                'ik_solver must be controller, local, or controller_then_local')
        if solver in ('controller', 'controller_then_local'):
            try:
                return self._solve_controller_ik(
                    arm_name, self._pose_to_xyz_rpy(message))
            except Exception as error:
                if solver == 'controller':
                    raise
                self.get_logger().warning(
                    f'Controller IK failed, using local IK: {error}')
        return self._solve_local_ik(arm_name, message)

    def _solve_local_ik(self, arm_name: str, message: PoseStamped) -> List[float]:
        names = LEFT_JOINTS if arm_name == 'LeftArm' else RIGHT_JOINTS
        with self._lock:
            if time.monotonic() - self._latest_state_time > self._state_timeout:
                raise ValueError('local IK requires fresh /joint_states feedback')
            if not all(name in self._latest_state for name in names):
                raise ValueError('local IK feedback is incomplete')
            seed = [self._latest_state[name] for name in names]
        return self._solve_local_pose(arm_name, message.pose, seed)

    def _solve_local_pose(
        self, arm_name: str, pose: Pose, seed: List[float]
    ) -> List[float]:
        position = pose.position
        orientation = pose.orientation
        rotation = quaternion_matrix(
            orientation.x, orientation.y, orientation.z, orientation.w)
        return self._kinematics[arm_name].inverse(
            [position.x, position.y, position.z], rotation, seed)

    @staticmethod
    def _pose_to_xyz_rpy(message: PoseStamped) -> List[float]:
        position = message.pose.position
        orientation = message.pose.orientation
        qx, qy, qz, qw = (
            float(orientation.x), float(orientation.y),
            float(orientation.z), float(orientation.w))
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0 or not math.isfinite(norm):
            raise ValueError('pose quaternion is invalid')
        qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
        roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
        pitch_term = 2.0 * (qw * qy - qz * qx)
        pitch = math.copysign(math.pi / 2.0, pitch_term) if abs(pitch_term) >= 1.0 else math.asin(pitch_term)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        pose = [float(position.x), float(position.y), float(position.z), roll, pitch, yaw]
        if not all(math.isfinite(value) for value in pose):
            raise ValueError('pose contains a non-finite value')
        return pose

    def _solve_controller_ik(self, arm_name: str, pose: List[float]) -> List[float]:
        robot_ip = str(self.get_parameter('robot_ip').value)
        ws_port = int(self.get_parameter('ws_port').value)
        timeout = float(self.get_parameter('ik_timeout').value)
        with self._ws_lock:
            if self._ws is None or not self._ws.connected:
                self._ws = websocket.create_connection(
                    f'ws://{robot_ip}:{ws_port}', timeout=timeout)
            self._request_id += 1
            request_id = self._request_id
            request = {
                'id': request_id,
                'type': 'RobotCmd',
                'action': 'ik',
                'data': {'armName': arm_name, 'pose': pose},
            }
            self._ws.send(json.dumps(request, separators=(',', ':')))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self._ws.settimeout(max(0.01, deadline - time.monotonic()))
                response = json.loads(self._ws.recv())
                if response.get('id') != request_id:
                    continue
                joints = response.get('data', {}).get('joints')
                if not isinstance(joints, list) or len(joints) != 7:
                    raise RuntimeError(f'IK failed or returned invalid joints: {response}')
                result = [float(value) for value in joints]
                if not all(math.isfinite(value) for value in result):
                    raise RuntimeError('IK returned non-finite joints')
                return result
        raise RuntimeError(f'IK request timed out after {timeout:.1f}s')

    @staticmethod
    def _parse_joint_limits(robot_description: str) -> Dict[str, Tuple[float, float]]:
        limits = {}
        if not robot_description:
            return limits
        root = ET.fromstring(robot_description)
        for joint in root.findall('joint'):
            name = joint.get('name', '')
            if name not in LEFT_JOINTS + RIGHT_JOINTS:
                continue
            limit = joint.find('limit')
            if limit is not None and limit.get('lower') and limit.get('upper'):
                limits[name] = (float(limit.get('lower')), float(limit.get('upper')))
        return limits

    def _state_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        values = {
            name: float(position)
            for name, position in zip(message.name, message.position)
            if name in LEFT_JOINTS + RIGHT_JOINTS and math.isfinite(position)
        }
        if not values:
            return
        with self._lock:
            self._latest_state.update(values)
            self._latest_state_time = time.monotonic()

    def _trajectory_callback(self, message: JointTrajectory) -> bool:
        try:
            joint_names, waypoints = self._prepare_trajectory(message)
        except ValueError as error:
            self.get_logger().error(f'Trajectory rejected: {error}')
            self._publish_status(f'rejected: {error}')
            return False
        with self._lock:
            self._joint_names = joint_names
            self._waypoints = waypoints
            self._start_time = time.monotonic()
            self._active = True
        self._publish_status('active')
        self.get_logger().info(
            f'Accepted {len(waypoints) - 1}-segment trajectory for '
            f'{len(joint_names)} joints, duration={waypoints[-1].time_from_start:.3f}s')
        return True

    def _prepare_trajectory(
        self, message: JointTrajectory
    ) -> Tuple[List[str], List[Waypoint]]:
        names = list(message.joint_names)
        if len(names) != len(set(names)):
            raise ValueError('joint names must be unique')
        valid_sets = (set(LEFT_JOINTS), set(RIGHT_JOINTS), set(LEFT_JOINTS + RIGHT_JOINTS))
        if set(names) not in valid_sets:
            raise ValueError('trajectory must contain one complete 7-joint arm or both arms')
        if not message.points:
            raise ValueError('trajectory contains no points')

        now = time.monotonic()
        with self._lock:
            if now - self._latest_state_time > self._state_timeout:
                raise ValueError('no fresh /joint_states feedback')
            if not all(name in self._latest_state for name in names):
                raise ValueError('feedback does not contain every commanded joint')
            start_positions = tuple(self._latest_state[name] for name in names)

        result = [Waypoint(0.0, start_positions, tuple([0.0] * len(names)))]
        previous_positions = start_positions
        previous_time = 0.0
        single_auto_duration = len(message.points) == 1
        for index, point in enumerate(message.points):
            if len(point.positions) != len(names):
                raise ValueError(f'point {index} position count does not match joint_names')
            positions = tuple(float(value) for value in point.positions)
            if not all(math.isfinite(value) for value in positions):
                raise ValueError(f'point {index} contains a non-finite position')
            self._check_limits(names, positions)
            requested_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if single_auto_duration and requested_time <= 0.0:
                max_delta = max(abs(end - start) for start, end in zip(previous_positions, positions))
                # Cubic smoothstep peak derivative is 1.5, so account for it.
                requested_time = max(
                    self._minimum_duration,
                    1.5 * max_delta / self._max_velocity,
                )
            if requested_time <= previous_time:
                raise ValueError('time_from_start values must be strictly increasing and positive')
            if point.velocities:
                if len(point.velocities) != len(names):
                    raise ValueError(f'point {index} velocity count does not match joint_names')
                velocities = tuple(float(value) for value in point.velocities)
                if not all(math.isfinite(value) for value in velocities):
                    raise ValueError(f'point {index} contains a non-finite velocity')
            else:
                velocities = tuple([0.0] * len(names))
            result.append(Waypoint(requested_time, positions, velocities))
            previous_positions = positions
            previous_time = requested_time
        self._validate_fitted_trajectory(names, result)
        return names, result

    def _validate_fitted_trajectory(
        self, names: List[str], waypoints: List[Waypoint]
    ) -> None:
        for start, end in zip(waypoints, waypoints[1:]):
            duration = end.time_from_start - start.time_from_start
            for sample in range(21):
                progress = sample / 20.0
                positions = self._hermite_positions(start, end, progress)
                self._check_limits(names, positions)
                u2 = progress * progress
                dh00 = 6.0 * u2 - 6.0 * progress
                dh10 = 3.0 * u2 - 4.0 * progress + 1.0
                dh01 = -dh00
                dh11 = 3.0 * u2 - 2.0 * progress
                velocities = [
                    (dh00 * begin + dh01 * finish) / duration
                    + dh10 * begin_velocity + dh11 * finish_velocity
                    for begin, finish, begin_velocity, finish_velocity in zip(
                        start.positions, end.positions, start.velocities, end.velocities)
                ]
                if max(abs(value) for value in velocities) > self._max_velocity * 1.05:
                    raise ValueError('fitted trajectory exceeds default_max_velocity')

    @staticmethod
    def _hermite_positions(
        start: Waypoint, end: Waypoint, progress: float
    ) -> Tuple[float, ...]:
        duration = end.time_from_start - start.time_from_start
        return tuple(
            (2.0 * progress ** 3 - 3.0 * progress ** 2 + 1.0) * begin
            + (progress ** 3 - 2.0 * progress ** 2 + progress) * duration * begin_velocity
            + (-2.0 * progress ** 3 + 3.0 * progress ** 2) * finish
            + (progress ** 3 - progress ** 2) * duration * finish_velocity
            for begin, finish, begin_velocity, finish_velocity in zip(
                start.positions, end.positions, start.velocities, end.velocities)
        )

    def _check_limits(self, names: List[str], positions: Tuple[float, ...]) -> None:
        for name, position in zip(names, positions):
            limits = self._limits.get(name)
            if limits and not limits[0] <= position <= limits[1]:
                raise ValueError(
                    f'{name}={position:.6f} outside [{limits[0]:.6f}, {limits[1]:.6f}]')

    def _timer_callback(self) -> None:
        with self._lock:
            if not self._active:
                return
            names = list(self._joint_names)
            waypoints = list(self._waypoints)
            elapsed = time.monotonic() - self._start_time

        if elapsed >= waypoints[-1].time_from_start:
            self._publish_command(names, waypoints[-1].positions)
            if not self._hold_final:
                with self._lock:
                    self._active = False
            if elapsed < waypoints[-1].time_from_start + self._period * 2.0:
                self._publish_status('completed')
            return

        for start, end in zip(waypoints, waypoints[1:]):
            if elapsed <= end.time_from_start:
                duration = end.time_from_start - start.time_from_start
                progress = (elapsed - start.time_from_start) / duration
                positions = self._hermite_positions(start, end, progress)
                self._publish_command(names, positions)
                return

    def _publish_command(self, names: List[str], positions: Tuple[float, ...]) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = names
        message.position = list(positions)
        self._command_publisher.publish(message)

    def _cancel_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            was_active = self._active
            self._active = False
        response.success = was_active
        response.message = 'trajectory cancelled' if was_active else 'no active trajectory'
        self._publish_status(response.message)
        return response

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_publisher.publish(message)

    def destroy_node(self) -> bool:
        with self._ws_lock:
            if self._ws is not None:
                self._ws.close()
                self._ws = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoDroidArmTrajectory()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
