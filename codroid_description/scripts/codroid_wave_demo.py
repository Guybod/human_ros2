#!/usr/bin/env python3
"""Publish a safe, visualization-only right-arm waving animation."""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


LEFT = [f'J_arm_l_{index:02d}' for index in range(1, 8)]
RIGHT = [f'J_arm_r_{index:02d}' for index in range(1, 8)]
NAMES = LEFT + RIGHT + ['J_head_yaw', 'J_head_pitch']
HOME = [0.0] * 16
# Shoulder raised, elbow bent, wrist ready to wave. All values are within URDF limits.
WAVE_POSE = [0.0] * 7 + [1.15, -0.75, 0.15, 1.55, 0.0, 0.0, 0.0] + [0.0, 0.0]


def smoothstep(progress: float) -> float:
    progress = min(1.0, max(0.0, progress))
    return progress * progress * (3.0 - 2.0 * progress)


class WaveDemo(Node):
    def __init__(self) -> None:
        super().__init__('codroid_wave_demo')
        self.declare_parameter('frequency_hz', 50.0)
        self.declare_parameter('transition_seconds', 2.0)
        self.declare_parameter('wave_seconds', 6.0)
        self.declare_parameter('loop', True)
        frequency = float(self.get_parameter('frequency_hz').value)
        self._transition = float(self.get_parameter('transition_seconds').value)
        self._wave_duration = float(self.get_parameter('wave_seconds').value)
        self._loop = bool(self.get_parameter('loop').value)
        self._publisher = self.create_publisher(JointState, '/joint_states', 10)
        self._started = time.monotonic()
        self.create_timer(1.0 / frequency, self._update)
        self.get_logger().info('Visualization-only right-arm wave started; no hardware commands')

    def _update(self) -> None:
        elapsed = time.monotonic() - self._started
        cycle = 2.0 * self._transition + self._wave_duration
        if self._loop:
            elapsed %= cycle

        if elapsed < self._transition:
            ratio = smoothstep(elapsed / self._transition)
            positions = [start + (end - start) * ratio for start, end in zip(HOME, WAVE_POSE)]
        elif elapsed < self._transition + self._wave_duration:
            positions = list(WAVE_POSE)
            wave_time = elapsed - self._transition
            swing = math.sin(2.0 * math.pi * 0.65 * wave_time)
            # Move shoulder/upper-arm joints (J01/J03), not wrist joints J06/J07.
            positions[7] = WAVE_POSE[7] + 0.38 * swing
            positions[9] = WAVE_POSE[9] - 0.32 * swing
        elif elapsed < cycle:
            ratio = smoothstep((elapsed - self._transition - self._wave_duration) / self._transition)
            positions = [start + (end - start) * ratio for start, end in zip(WAVE_POSE, HOME)]
        else:
            positions = HOME

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = NAMES
        message.position = positions
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaveDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
