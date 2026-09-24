"""Record synchronized visual and motor samples to an analysis-ready CSV."""

from __future__ import annotations

import csv
from datetime import datetime
import json
import math
from pathlib import Path
import platform

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud

from std_msgs.msg import Float64, String

from .records import (
    host_current_values,
    point_cloud_coordinates,
    stamp_to_nanoseconds,
)


FIELDNAMES = [
    'receive_time_ns', 'camera_time_ns', 'arm', 'axis',
    'mcu_time_us', 'telemetry_sequence', 'telemetry_age_ms',
    'encoder_raw', 'encoder_zero_raw', 'q_operational', 'q_session',
    'velocity_reference', 'command', 'adc_raw',
    'current_ma', 'current_a', 'firmware_flags',
    'x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'x3', 'y3', 'z3',
]

TELEMETRY_FIELDNAMES = [
    'receive_time_ns', 'mcu_time_us', 'telemetry_sequence',
    'encoder_0_raw', 'encoder_1_raw', 'command_0', 'command_1',
    'requested_command_0', 'requested_command_1',
    'velocity_reference_0', 'velocity_reference_1',
    'applied_command_source',
    'adc_0_raw', 'adc_1_raw', 'current_0_ma', 'current_1_ma',
    'current_0_a', 'current_1_a',
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
        self.declare_parameter('velocity_reference_topics', [
            '/arm_1/encoder_velocity_reference',
            '/arm_2/encoder_velocity_reference',
        ])
        self.declare_parameter('axis_indices', [0, 1])
        self.declare_parameter('encoder_signs', [1, -1])
        self.declare_parameter('maximum_telemetry_age_ms', 100.0)
        self.declare_parameter('flush_every_rows', 30)
        self.declare_parameter('motor_supply_voltage_v', 5.6)
        self.declare_parameter('motor_supply_current_limit_a', -1.0)
        self.declare_parameter('motor_supply_source', '5.6 V transformer')

        names = [
            str(value) for value in self.get_parameter('arm_names').value
        ]
        topics = [
            str(value) for value in self.get_parameter('state_topics').value
        ]
        reference_topics = [
            str(value)
            for value in self.get_parameter('velocity_reference_topics').value
        ]
        axes = [
            int(value) for value in self.get_parameter('axis_indices').value
        ]
        signs = [
            int(value) for value in self.get_parameter('encoder_signs').value
        ]
        lengths = (
            len(names), len(topics), len(reference_topics), len(axes), len(signs)
        )
        if not names or len(set(lengths)) != 1:
            raise ValueError(
                'arm names, state/reference topics, axis indices and signs '
                'must match'
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
        self._encoder_zeros = None
        self._velocity_references = [0.0, 0.0]

        self._metadata_path = self._session_path / 'metadata.json'
        self._metadata = {
            'schema_version': 5,
            'created_local': datetime.now().astimezone().isoformat(),
            'host': platform.node(),
            'motor_supply': {
                'voltage_v': float(
                    self.get_parameter('motor_supply_voltage_v').value
                ),
                'current_limit_a': float(
                    self.get_parameter(
                        'motor_supply_current_limit_a'
                    ).value
                ),
                'source': str(
                    self.get_parameter('motor_supply_source').value
                ),
            },
            'arms': [
                {
                    'name': name,
                    'state_topic': topic,
                    'velocity_reference_topic': reference_topic,
                    'axis': axis,
                    'encoder_sign': sign,
                    'encoder_zero_raw': None,
                }
                for name, topic, reference_topic, axis, sign in zip(
                    names, topics, reference_topics, axes, signs
                )
            ],
            'sampling_rule': 'one row per complete arm PointCloud',
            'raw_telemetry_file': (
                'telemetry.csv contains every received STATE'
            ),
            'motor_state_topic': '/motor_state',
            'velocity_reference_semantics': (
                'latest ROS encoder-velocity reference sampled with each row'
            ),
            'command_semantics': (
                'firmware-reported PWM command after watchdog and limits'
            ),
            'current_semantics': (
                'current_a is host-calibrated ACS712 current; current_ma is '
                'the firmware field and may contain its unavailable sentinel'
            ),
            'maximum_telemetry_age_ms': self._maximum_age_ms,
        }
        self._write_metadata()

        self._subscriptions = [
            self.create_subscription(
                String, '/motor_state', self._on_motor_state, 50
            ),
        ]
        for axis, topic in zip(axes, reference_topics):
            self._subscriptions.append(self.create_subscription(
                Float64,
                topic,
                lambda message, selected=axis: self._on_velocity_reference(
                    selected, message
                ),
                20,
            ))
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

    def _on_velocity_reference(self, axis: int, message: Float64) -> None:
        value = float(message.data)
        if not math.isfinite(value):
            self.get_logger().warning(
                f'axis {axis}: ignored non-finite velocity reference'
            )
            return
        self._velocity_references[axis] = value

    @staticmethod
    def _unique_session_path(root: Path, name: str) -> Path:
        candidate = root / name
        suffix = 1
        while candidate.exists():
            candidate = root / f'{name}_{suffix:02d}'
            suffix += 1
        return candidate

    def _write_metadata(self) -> None:
        temporary = self._metadata_path.with_suffix('.json.tmp')
        temporary.write_text(
            json.dumps(self._metadata, indent=2) + '\n', encoding='utf-8'
        )
        temporary.replace(self._metadata_path)

    def _on_motor_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not all(
                key in payload
                for key in (
                    'sequence', 'mcu_time_us', 'encoder_counts', 'raw_adc',
                    'current_ma', 'flags', 'applied_commands',
                    'requested_commands', 'applied_command_source',
                    'current_a_host',
                )
            ):
                raise ValueError('missing telemetry field')
            if not all(len(payload[key]) == 2 for key in (
                'encoder_counts', 'raw_adc', 'current_ma',
                'current_a_host', 'applied_commands', 'requested_commands',
            )):
                raise ValueError('telemetry arrays must contain two values')
            host_current = host_current_values(payload)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f'ignored invalid motor_state: {error}')
            return
        self._latest_motor_state = payload
        self._latest_motor_state_received_ns = (
            self.get_clock().now().nanoseconds
        )
        if self._encoder_zeros is None:
            self._encoder_zeros = tuple(
                int(value) for value in payload['encoder_counts']
            )
            for arm in self._metadata['arms']:
                arm['encoder_zero_raw'] = self._encoder_zeros[arm['axis']]
            self._metadata['encoder_zero_rule'] = (
                'first valid motor_state received in this session'
            )
            self._write_metadata()
            self.get_logger().info(
                f'session encoder zero captured: {self._encoder_zeros}'
            )
        self._telemetry_writer.writerow({
            'receive_time_ns': self._latest_motor_state_received_ns,
            'mcu_time_us': int(payload['mcu_time_us']),
            'telemetry_sequence': int(payload['sequence']),
            'encoder_0_raw': int(payload['encoder_counts'][0]),
            'encoder_1_raw': int(payload['encoder_counts'][1]),
            'command_0': int(payload['applied_commands'][0]),
            'command_1': int(payload['applied_commands'][1]),
            'requested_command_0': int(payload['requested_commands'][0]),
            'requested_command_1': int(payload['requested_commands'][1]),
            'velocity_reference_0': repr(self._velocity_references[0]),
            'velocity_reference_1': repr(self._velocity_references[1]),
            'applied_command_source': str(payload['applied_command_source']),
            'adc_0_raw': int(payload['raw_adc'][0]),
            'adc_1_raw': int(payload['raw_adc'][1]),
            'current_0_ma': int(payload['current_ma'][0]),
            'current_1_ma': int(payload['current_ma'][1]),
            'current_0_a': repr(host_current[0]),
            'current_1_a': repr(host_current[1]),
            'firmware_flags': int(payload['flags']),
        })
        self._telemetry_rows += 1
        if self._telemetry_rows % self._flush_every == 0:
            self._telemetry_file.flush()

    def _on_shape(
        self, arm_name: str, axis: int, encoder_sign: int, message: PointCloud
    ) -> None:
        try:
            state = point_cloud_coordinates(message)
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
        encoder_zero = self._encoder_zeros[axis]
        q_session = encoder_sign * (encoder_raw - encoder_zero)
        row = {
            'receive_time_ns': receive_time_ns,
            'camera_time_ns': stamp_to_nanoseconds(message.header.stamp),
            'arm': arm_name,
            'axis': axis,
            'mcu_time_us': int(telemetry['mcu_time_us']),
            'telemetry_sequence': int(telemetry['sequence']),
            'telemetry_age_ms': f'{age_ms:.6f}',
            'encoder_raw': encoder_raw,
            'encoder_zero_raw': encoder_zero,
            'q_operational': q_session,
            'q_session': q_session,
            'velocity_reference': repr(self._velocity_references[axis]),
            'command': int(telemetry['applied_commands'][axis]),
            'adc_raw': int(telemetry['raw_adc'][axis]),
            'current_ma': int(telemetry['current_ma'][axis]),
            'current_a': repr(float(telemetry['current_a_host'][axis])),
            'firmware_flags': int(telemetry['flags']),
            'x1': repr(state[0]), 'y1': repr(state[1]), 'z1': repr(state[2]),
            'x2': repr(state[3]), 'y2': repr(state[4]), 'z2': repr(state[5]),
            'x3': repr(state[6]), 'y3': repr(state[7]), 'z3': repr(state[8]),
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
