#!/usr/bin/env python3
"""ROS 2 bridge for the CoDroid dual-arm UDP real-time interface."""

import json
import socket
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


LEFT_JOINTS = [f'J_arm_l_{index:02d}' for index in range(1, 8)]
RIGHT_JOINTS = [f'J_arm_r_{index:02d}' for index in range(1, 8)]
ARM_JOINTS = LEFT_JOINTS + RIGHT_JOINTS


class CoDroidArmBridge(Node):
    def __init__(self) -> None:
        super().__init__('codroid_arm_bridge')

        self.declare_parameter('robot_ip', '192.168.2.16')
        self.declare_parameter('command_port', 9001)
        self.declare_parameter('feedback_port', 9002)
        self.declare_parameter('command_topic', '/codroid/arm_command')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('auto_connect', True)

        self._robot_address = (
            str(self.get_parameter('robot_ip').value),
            int(self.get_parameter('command_port').value),
        )
        feedback_port = int(self.get_parameter('feedback_port').value)

        self._send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._receive_socket.settimeout(0.2)
        try:
            self._receive_socket.bind(('', feedback_port))
        except OSError as error:
            self._send_socket.close()
            self._receive_socket.close()
            raise RuntimeError(f'Cannot bind UDP feedback port {feedback_port}: {error}') from error

        self._running = True
        self._feedback_thread = threading.Thread(
            target=self._receive_loop, name='codroid_udp_feedback', daemon=True)

        joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        command_topic = str(self.get_parameter('command_topic').value)
        self._joint_state_publisher = self.create_publisher(JointState, joint_state_topic, 10)
        self._status_publisher = self.create_publisher(String, '/codroid/status', 10)
        self.create_subscription(JointState, command_topic, self._command_callback, 10)

        self.create_service(SetBool, '~/connect', self._flag_service('StartFlag'))
        self.create_service(SetBool, '~/enable', self._flag_service('SwitchOn'))
        self.create_service(SetBool, '~/teleoperation', self._flag_service('EnableControl'))
        self.create_service(SetBool, '~/lock', self._flag_service('TryLock'))
        self.create_service(SetBool, '~/left_control', self._control_service('LeftArm'))
        self.create_service(SetBool, '~/right_control', self._control_service('RightArm'))
        self.create_service(Trigger, '~/reset_error', self._reset_error)

        self._feedback_thread.start()
        if bool(self.get_parameter('auto_connect').value):
            self._send({'StartFlag': True})

        self.get_logger().info(
            f'CoDroid arm bridge: commands -> {self._robot_address[0]}:{self._robot_address[1]}, '
            f'feedback <- UDP :{feedback_port}')

    def _send(self, payload: dict) -> None:
        data = json.dumps(payload, separators=(',', ':'), ensure_ascii=False).encode('utf-8')
        self._send_socket.sendto(data, self._robot_address)

    def _flag_service(self, field: str):
        def callback(request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
            try:
                self._send({field: request.data})
                response.success = True
                response.message = f'{field}={request.data} sent'
            except OSError as error:
                response.success = False
                response.message = str(error)
            return response
        return callback

    def _control_service(self, arm_name: str):
        def callback(request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
            field = 'StartControl' if request.data else 'StopControl'
            try:
                self._send({field: {'name': arm_name}})
                response.success = True
                response.message = f'{field} {arm_name} sent'
            except OSError as error:
                response.success = False
                response.message = str(error)
            return response
        return callback

    def _reset_error(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            self._send({'ResetError': True})
            response.success = True
            response.message = 'ResetError sent'
        except OSError as error:
            response.success = False
            response.message = str(error)
        return response

    def _command_callback(self, message: JointState) -> None:
        positions = self._extract_arm_positions(message)
        if not positions:
            return
        self._send({'data': {arm: {'P': values} for arm, values in positions.items()}})

    def _extract_arm_positions(self, message: JointState) -> Dict[str, List[float]]:
        if message.name:
            if len(message.name) != len(message.position):
                self.get_logger().error('Ignoring command: name and position lengths differ')
                return {}
            values = dict(zip(message.name, message.position))
            result = {}
            if all(name in values for name in LEFT_JOINTS):
                result['LeftArm'] = [float(values[name]) for name in LEFT_JOINTS]
            if all(name in values for name in RIGHT_JOINTS):
                result['RightArm'] = [float(values[name]) for name in RIGHT_JOINTS]
            if not result:
                self.get_logger().warning('Ignoring command: no complete 7-joint arm found')
            return result

        if len(message.position) == 14:
            return {
                'LeftArm': [float(value) for value in message.position[:7]],
                'RightArm': [float(value) for value in message.position[7:]],
            }
        self.get_logger().error('Unnamed commands must contain exactly 14 positions')
        return {}

    @staticmethod
    def _seven_values(section: dict, field: str) -> Optional[List[float]]:
        values = section.get(field)
        if not isinstance(values, list) or len(values) < 7:
            return None
        try:
            return [float(value) for value in values[:7]]
        except (TypeError, ValueError):
            return None

    def _receive_loop(self) -> None:
        while self._running:
            try:
                packet, _address = self._receive_socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                payload = json.loads(packet.decode('utf-8'))
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                self.get_logger().warning(f'Invalid UDP feedback ignored: {error}')
                continue
            self._publish_feedback(payload)

    def _publish_feedback(self, payload: dict) -> None:
        data = payload.get('data')
        if not isinstance(data, dict):
            return

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        for arm_name, joint_names in (('LeftArm', LEFT_JOINTS), ('RightArm', RIGHT_JOINTS)):
            section = data.get(arm_name)
            if not isinstance(section, dict):
                continue
            positions = self._seven_values(section, 'AP')
            if positions is None:
                continue
            message.name.extend(joint_names)
            message.position.extend(positions)
            velocities = self._seven_values(section, 'AV')
            efforts = self._seven_values(section, 'AT')
            message.velocity.extend(velocities if velocities is not None else [0.0] * 7)
            message.effort.extend(efforts if efforts is not None else [0.0] * 7)

        if message.name:
            self._joint_state_publisher.publish(message)

        status = String()
        status.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self._status_publisher.publish(status)

    def destroy_node(self) -> bool:
        if self._running:
            try:
                self._send({'StartFlag': False})
            except OSError:
                pass
            self._running = False
            self._receive_socket.close()
            self._send_socket.close()
            self._feedback_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoDroidArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
