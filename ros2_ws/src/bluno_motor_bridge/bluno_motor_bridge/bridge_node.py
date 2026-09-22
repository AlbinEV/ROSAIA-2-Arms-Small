"""ROS 2 node bridging motor commands and Bluno serial telemetry."""

import json
import time

import rclpy
from rclpy.node import Node
import serial
from serial import SerialException
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Int64MultiArray, String

from .protocol import (
    CURRENT_UNAVAILABLE_MA,
    encode_command,
    encode_stop,
    parse_state,
    ProtocolError,
)


class BlunoMotorBridge(Node):
    """Fail-safe, reconnecting serial bridge for two logical primary motors."""

    def __init__(self) -> None:
        super().__init__('bluno_motor_bridge')

        self.declare_parameter('serial_port', '')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('command_rate_hz', 100.0)
        self.declare_parameter('command_timeout_ms', 300)
        self.declare_parameter('reconnect_period_s', 1.0)
        self.declare_parameter('max_abs_command', 0)
        self.declare_parameter('active_axes', [True, False])
        self.declare_parameter('motor_sign', [1, 1])
        self.declare_parameter('axis_names', ['arm_1_primary', 'arm_2_primary'])
        self.declare_parameter('shield_channels', ['M2', 'M1'])
        self.declare_parameter('axis_0_encoder_cpr', -1)
        self.declare_parameter('axis_1_encoder_cpr', -1)
        self.declare_parameter('axis_0_gear_ratio', -1.0)
        self.declare_parameter('axis_1_gear_ratio', -1.0)
        self.declare_parameter('axis_0_current_sensitivity_v_per_a', -1.0)
        self.declare_parameter('axis_1_current_sensitivity_v_per_a', -1.0)

        self._port = str(self.get_parameter('serial_port').value)
        self._baud = int(self.get_parameter('baud_rate').value)
        self._rate_hz = float(self.get_parameter('command_rate_hz').value)
        self._timeout_s = float(self.get_parameter('command_timeout_ms').value) / 1000.0
        self._reconnect_s = float(self.get_parameter('reconnect_period_s').value)
        self._limit = int(self.get_parameter('max_abs_command').value)
        self._active_axes = tuple(bool(v) for v in self.get_parameter('active_axes').value)
        self._motor_sign = tuple(int(v) for v in self.get_parameter('motor_sign').value)
        self._axis_names = tuple(str(v) for v in self.get_parameter('axis_names').value)
        self._shield_channels = tuple(
            str(v) for v in self.get_parameter('shield_channels').value
        )
        self._validate_parameters()

        self._serial = None
        self._rx_buffer = bytearray()
        self._stop_sent = False
        self._sequence = 0
        self._commands = (0, 0)
        self._last_command_time = None
        self._last_connect_attempt = -float('inf')
        self._last_state_time = None
        self._last_status_publish = -float('inf')
        self._invalid_packet_count = 0
        self._last_firmware_error = None

        self._command_sub = self.create_subscription(
            Int16MultiArray, 'motor_command', self._on_command, 10
        )
        self._encoder_pub = self.create_publisher(
            Int64MultiArray, 'encoder_counts', 10
        )
        self._current_pub = self.create_publisher(
            Float32MultiArray, 'motor_current', 10
        )
        self._state_pub = self.create_publisher(String, 'motor_state', 10)
        self._status_pub = self.create_publisher(String, 'bluno/status', 10)
        self._timer = self.create_timer(1.0 / self._rate_hz, self._tick)

        if not self._port:
            self.get_logger().warning(
                'serial_port is empty: bridge is locked in disconnected STOP state'
            )
        if self._limit == 0:
            self.get_logger().warning(
                'max_abs_command is 0: all non-zero motor commands are rejected'
            )

    def _validate_parameters(self) -> None:
        if self._baud <= 0:
            raise ValueError('baud_rate must be positive')
        if self._rate_hz <= 0.0:
            raise ValueError('command_rate_hz must be positive')
        if self._timeout_s <= 0.0 or self._reconnect_s <= 0.0:
            raise ValueError('timeouts must be positive')
        if not 0 <= self._limit <= 255:
            raise ValueError('max_abs_command must be in [0, 255]')
        for name, values in (
            ('active_axes', self._active_axes),
            ('motor_sign', self._motor_sign),
            ('axis_names', self._axis_names),
            ('shield_channels', self._shield_channels),
        ):
            if len(values) != 2:
                raise ValueError(f'{name} must contain exactly two entries')
        if any(sign not in (-1, 1) for sign in self._motor_sign):
            raise ValueError('motor_sign entries must be -1 or 1')
        if self._shield_channels != ('M2', 'M1'):
            raise ValueError('current board contract requires axis mapping [M2, M1]')

    def _on_command(self, message: Int16MultiArray) -> None:
        if len(message.data) != 2:
            self.get_logger().error('motor_command must contain exactly two integers')
            return

        requested = tuple(int(value) for value in message.data)
        if any(abs(value) > self._limit for value in requested):
            self.get_logger().error(
                f'rejected command {requested}: configured limit is {self._limit}'
            )
            return

        self._commands = tuple(
            value * sign if active else 0
            for value, sign, active in zip(
                requested, self._motor_sign, self._active_axes
            )
        )
        self._last_command_time = time.monotonic()

    def _connect_if_due(self, now: float) -> None:
        if self._serial is not None or not self._port:
            return
        if now - self._last_connect_attempt < self._reconnect_s:
            return
        self._last_connect_attempt = now
        try:
            self._serial = serial.Serial(
                self._port,
                self._baud,
                timeout=0,
                write_timeout=min(self._timeout_s, 0.1),
            )
            self._serial.reset_input_buffer()
            self._serial.write(encode_stop())
            self._rx_buffer.clear()
            self._stop_sent = True
            self.get_logger().info(f'connected to {self._port} at {self._baud} baud')
        except (OSError, SerialException) as exc:
            self._serial = None
            self.get_logger().warning(f'cannot open {self._port}: {exc}')

    def _disconnect(self, reason: str) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except (OSError, SerialException):
                pass
        self._serial = None
        self._commands = (0, 0)
        self._last_command_time = None
        self._rx_buffer.clear()
        self._stop_sent = False
        self.get_logger().error(f'serial link closed: {reason}')

    def _tick(self) -> None:
        now = time.monotonic()
        self._connect_if_due(now)

        if self._serial is not None:
            try:
                command_fresh = (
                    self._last_command_time is not None
                    and now - self._last_command_time <= self._timeout_s
                )
                if command_fresh:
                    self._serial.write(encode_command(self._sequence, self._commands))
                    self._sequence = (self._sequence + 1) % (2**31)
                    self._stop_sent = False
                elif not self._stop_sent:
                    self._serial.write(encode_stop())
                    self._stop_sent = True
                self._read_available_state(now)
            except (OSError, SerialException) as exc:
                self._disconnect(str(exc))

        if now - self._last_status_publish >= 1.0:
            self._publish_status(now)
            self._last_status_publish = now

    def _read_available_state(self, now: float) -> None:
        available = self._serial.in_waiting
        if available > 0:
            self._rx_buffer.extend(self._serial.read(available))
        if len(self._rx_buffer) > 4096:
            self._invalid_packet_count += 1
            self._rx_buffer.clear()
            self.get_logger().error('serial receive buffer overflow; data discarded')
            return

        for _ in range(20):
            newline = self._rx_buffer.find(b'\n')
            if newline < 0:
                break
            line = bytes(self._rx_buffer[:newline]).rstrip(b'\r')
            del self._rx_buffer[:newline + 1]
            if not line:
                continue
            if line.startswith((b'BOOT,', b'OK,', b'PONG,')):
                continue
            if line.startswith(b'ERR,'):
                self._last_firmware_error = line.decode('ascii', errors='replace')
                self.get_logger().error(
                    f'firmware rejected a command: {self._last_firmware_error}'
                )
                continue
            try:
                packet = parse_state(line)
            except ProtocolError as exc:
                self._invalid_packet_count += 1
                self.get_logger().warning(f'ignored malformed telemetry: {exc}')
                continue

            self._last_state_time = now
            encoder_message = Int64MultiArray()
            encoder_message.data = list(packet.encoder_counts)
            self._encoder_pub.publish(encoder_message)

            current_message = Float32MultiArray()
            current_message.data = [
                float('nan')
                if value == CURRENT_UNAVAILABLE_MA
                else value / 1000.0
                for value in packet.current_ma
            ]
            self._current_pub.publish(current_message)

            state_message = String()
            payload = packet.as_dict()
            payload['axis_names'] = self._axis_names
            payload['shield_channels'] = self._shield_channels
            state_message.data = json.dumps(payload, separators=(',', ':'))
            self._state_pub.publish(state_message)

    def _publish_status(self, now: float) -> None:
        state_age = (
            None if self._last_state_time is None else now - self._last_state_time
        )
        message = String()
        message.data = json.dumps(
            {
                'connected': self._serial is not None,
                'serial_port': self._port,
                'motion_unlocked': self._limit > 0,
                'active_axes': self._active_axes,
                'last_state_age_s': state_age,
                'invalid_packet_count': self._invalid_packet_count,
                'last_firmware_error': self._last_firmware_error,
            },
            separators=(',', ':'),
        )
        self._status_pub.publish(message)

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._serial.write(encode_stop())
                self._serial.flush()
            except (OSError, SerialException):
                pass
            try:
                self._serial.close()
            except (OSError, SerialException):
                pass
        return super().destroy_node()


def main(args=None) -> None:
    """Run the ROS 2 bridge node."""
    rclpy.init(args=args)
    node = BlunoMotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # A launch process can forward SIGINT while shutdown is in progress.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
