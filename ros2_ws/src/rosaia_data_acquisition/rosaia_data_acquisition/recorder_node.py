"""Record synchronized visual and motor samples to an analysis-ready CSV."""

from __future__ import annotations

import csv
from datetime import datetime
import json
from pathlib import Path
import platform

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud

from std_msgs.msg import Int16MultiArray, String

from .records import point_cloud_values, stamp_to_nanoseconds


FIELDNAMES = [
    'receive_time_ns', 'camera_time_ns', 'arm', 'axis',
    'mcu_time_us', 'telemetry_sequence', 'telemetry_age_ms',
    'encoder_raw', 'q_operational', 'command', 'adc_raw',
    'current_ma', 'firmware_flags', 'x1', 'y1', 'x2', 'y2', 'x3', 'y3',
]

TELEMETRY_FIELDNAMES = [
    'receive_time_ns', 'mcu_time_us', 'telemetry_sequence',
    'encoder_0_raw', 'encoder_1_raw', 'command_0', 'command_1',
    'adc_0_raw', 'adc_1_raw', 'current_0_ma', 'current_1_ma',
    'firmware_flags',
]


class SessionRecorder(Node):
    """Sample motor telemetry whenever a complete arm state arrives."""

    def __init__(self) -> None:
        """Declare the acquisition contract and open a unique session."""
        super().__init__('session_recorder')
        self.declare_parameter('output_root', 'data/sessions')
        self.declare_parameter('session_name', '')
        self.declare_parameter('arm_names', ['arm_1', 'arm_2'])
        self.declare_parameter(
            'state_topics', ['/arm_1/aruco_state', '/arm_2/aruco_state']
        )
        self.declare_parameter('axis_indices', [0, 1])
        self.declare_parameter('encoder_signs', [-1, -1])
        self.declare_parameter('maximum_telemetry_age_ms', 100.0)
        self.declare_parameter('flush_every_rows', 30)

        names = [
            str(value) for value in self.get_parameter('arm_names').value
        ]
        topics = [
            str(value) for value in self.get_parameter('state_topics').value
        ]
        axes = [
            int(value) for value in self.get_parameter('axis_indices').value
        ]
        signs = [
            int(value) for value in self.get_parameter('encoder_signs').value
        ]
        lengths = (len(names), len(topics), len(axes), len(signs))
        if not names or len(set(lengths)) != 1:
            raise ValueError(
                'arm_names, state_topics, axis_indices and signs must match'
            )
        if any(axis not in (0, 1) for axis in axes):
            raise ValueError('axis_indices entries must be 0 or 1')
        if any(sign not in (-1, 1) for sign in signs):
            raise ValueError('encoder_signs entries must be -1 or 1')
        self._maximum_age_ms = float(
            self.get_parameter('maximum_telemetry_age_ms').value
        )
        self._flush_every = int(self.get_parameter('flush_every_rows').value)
        if self._maximum_age_ms <= 0.0 or self._flush_every <= 0:
            raise ValueError('age and flush limits must be positive')

        root = Path(str(self.get_parameter('output_root').value)).expanduser()
        requested_name = str(self.get_parameter('session_name').value).strip()
        session_name = requested_name or datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )
        self._session_path = self._unique_session_path(root, session_name)
        self._session_path.mkdir(parents=True, exist_ok=False)
        self._csv_file = (self._session_path / 'samples.csv').open(
            'w', newline='', encoding='utf-8'
        )
        self._writer = csv.DictWriter(self._csv_file, fieldnames=FIELDNAMES)
        self._writer.writeheader()
        self._telemetry_file = (self._session_path / 'telemetry.csv').open(
            'w', newline='', encoding='utf-8'
        )
        self._telemetry_writer = csv.DictWriter(
            self._telemetry_file, fieldnames=TELEMETRY_FIELDNAMES
        )
        self._telemetry_writer.writeheader()
        self._rows = 0
        self._telemetry_rows = 0
        self._latest_motor_state = None
        self._latest_motor_state_received_ns = None
        self._commands = [0, 0]

        metadata = {
            'schema_version': 1,
            'created_local': datetime.now().astimezone().isoformat(),
            'host': platform.node(),
            'arms': [
                {
                    'name': name,
                    'state_topic': topic,
                    'axis': axis,
                    'encoder_sign': sign,
                }
                for name, topic, axis, sign in zip(names, topics, axes, signs)
            ],
            'sampling_rule': 'one row per complete arm PointCloud',
            'raw_telemetry_file': (
                'telemetry.csv contains every received STATE'
            ),
            'motor_state_topic': '/motor_state',
            'command_topic': '/motor_command',
            'maximum_telemetry_age_ms': self._maximum_age_ms,
        }
        (self._session_path / 'metadata.json').write_text(
            json.dumps(metadata, indent=2) + '\n', encoding='utf-8'
        )

        self._subscriptions = [
            self.create_subscription(
                String, '/motor_state', self._on_motor_state, 50
            ),
            self.create_subscription(
                Int16MultiArray, '/motor_command', self._on_command, 20
            ),
        ]
        for name, topic, axis, sign in zip(names, topics, axes, signs):
            self._subscriptions.append(
                self.create_subscription(
                    PointCloud,
                    topic,
                    lambda message, n=name, a=axis, s=sign: self._on_shape(
                        n, a, s, message
                    ),
                    qos_profile_sensor_data,
                )
            )
        self.get_logger().info(f'recording session in {self._session_path}')

    @staticmethod
    def _unique_session_path(root: Path, name: str) -> Path:
        candidate = root / name
        suffix = 1
        while candidate.exists():
            candidate = root / f'{name}_{suffix:02d}'
            suffix += 1
        return candidate

    def _on_motor_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not all(
                key in payload
                for key in (
                    'sequence', 'mcu_time_us', 'encoder_counts', 'raw_adc',
                    'current_ma', 'flags'
                )
            ):
                raise ValueError('missing telemetry field')
            if not all(len(payload[key]) == 2 for key in (
                'encoder_counts', 'raw_adc', 'current_ma'
            )):
                raise ValueError('telemetry arrays must contain two values')
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f'ignored invalid motor_state: {error}')
            return
        self._latest_motor_state = payload
        self._latest_motor_state_received_ns = (
            self.get_clock().now().nanoseconds
        )
        self._telemetry_writer.writerow({
            'receive_time_ns': self._latest_motor_state_received_ns,
            'mcu_time_us': int(payload['mcu_time_us']),
            'telemetry_sequence': int(payload['sequence']),
            'encoder_0_raw': int(payload['encoder_counts'][0]),
            'encoder_1_raw': int(payload['encoder_counts'][1]),
            'command_0': self._commands[0],
            'command_1': self._commands[1],
            'adc_0_raw': int(payload['raw_adc'][0]),
            'adc_1_raw': int(payload['raw_adc'][1]),
            'current_0_ma': int(payload['current_ma'][0]),
            'current_1_ma': int(payload['current_ma'][1]),
            'firmware_flags': int(payload['flags']),
        })
        self._telemetry_rows += 1
        if self._telemetry_rows % self._flush_every == 0:
            self._telemetry_file.flush()

    def _on_command(self, message: Int16MultiArray) -> None:
        if len(message.data) == 2:
            self._commands = [int(message.data[0]), int(message.data[1])]

    def _on_shape(
        self, arm_name: str, axis: int, encoder_sign: int, message: PointCloud
    ) -> None:
        try:
            state = point_cloud_values(message)
        except ValueError as error:
            self.get_logger().warning(
                f'{arm_name}: ignored invalid shape: {error}'
            )
            return
        if self._latest_motor_state is None:
            self.get_logger().warning(
                f'{arm_name}: no motor telemetry yet; '
                'shape sample not recorded'
            )
            return

        receive_time_ns = self.get_clock().now().nanoseconds
        age_ms = (
            receive_time_ns - self._latest_motor_state_received_ns
        ) / 1_000_000.0
        if age_ms > self._maximum_age_ms:
            self.get_logger().warning(
                f'{arm_name}: telemetry is {age_ms:.1f} ms old; '
                'sample rejected'
            )
            return
        telemetry = self._latest_motor_state
        encoder_raw = int(telemetry['encoder_counts'][axis])
        row = {
            'receive_time_ns': receive_time_ns,
            'camera_time_ns': stamp_to_nanoseconds(message.header.stamp),
            'arm': arm_name,
            'axis': axis,
            'mcu_time_us': int(telemetry['mcu_time_us']),
            'telemetry_sequence': int(telemetry['sequence']),
            'telemetry_age_ms': f'{age_ms:.6f}',
            'encoder_raw': encoder_raw,
            'q_operational': encoder_sign * encoder_raw,
            'command': self._commands[axis],
            'adc_raw': int(telemetry['raw_adc'][axis]),
            'current_ma': int(telemetry['current_ma'][axis]),
            'firmware_flags': int(telemetry['flags']),
            'x1': repr(state[0]), 'y1': repr(state[1]),
            'x2': repr(state[2]), 'y2': repr(state[3]),
            'x3': repr(state[4]), 'y3': repr(state[5]),
        }
        self._writer.writerow(row)
        self._rows += 1
        if self._rows % self._flush_every == 0:
            self._csv_file.flush()

    def destroy_node(self):
        """Flush and close both immutable session streams."""
        if hasattr(self, '_csv_file') and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
            self._telemetry_file.flush()
            self._telemetry_file.close()
            if rclpy.ok():
                self.get_logger().info(
                    f'closed {self._session_path} after {self._rows} visual '
                    f'rows and {self._telemetry_rows} telemetry rows'
                )
        return super().destroy_node()


def main(args=None) -> None:
    """Run the synchronized session recorder."""
    rclpy.init(args=args)
    node = None
    try:
        node = SessionRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (KeyboardInterrupt, RuntimeError, ValueError):
                # Jazzy may invalidate subscriptions while propagating SIGINT.
                # Session files are closed before the rclpy teardown begins.
                pass
        if rclpy.ok():
            rclpy.shutdown()
