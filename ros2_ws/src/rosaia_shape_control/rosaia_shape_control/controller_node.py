"""ROS 2 wrapper for independent single-input visual shape controllers."""

from __future__ import annotations

from dataclasses import dataclass
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64, Float64MultiArray, String

from .control import resolved_rate_scalar


@dataclass
class ArmRuntime:
    name: str
    state: np.ndarray | None = None
    goal: np.ndarray | None = None
    jacobian: np.ndarray | None = None
    state_received_at: float | None = None
    jacobian_received_at: float | None = None
    last_reason: str = 'waiting_for_inputs'


def point_cloud_vector(message: PointCloud) -> np.ndarray:
    """Convert exactly three finite planar points to the ordered six-state."""
    if len(message.points) != 3:
        raise ValueError('shape PointCloud must contain exactly three points')
    values = np.array(
        [[point.x, point.y] for point in message.points], dtype=np.float64
    ).reshape(6)
    if not np.isfinite(values).all():
        raise ValueError('shape PointCloud contains non-finite coordinates')
    return values


class ShapeController(Node):
    """Evaluate one damped resolved-rate controller per deformable arm."""

    def __init__(self) -> None:
        super().__init__('shape_controller')
        self.declare_parameter('arm_names', ['arm_1'])
        self.declare_parameter('state_topics', ['/arm_1/aruco_state'])
        self.declare_parameter('goal_topics', ['/arm_1/shape_goal'])
        self.declare_parameter('jacobian_topics', ['/arm_1/jacobian'])
        self.declare_parameter(
            'velocity_reference_topics', ['/arm_1/encoder_velocity_reference']
        )
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('gain', 1.0)
        self.declare_parameter('damping', 0.01)
        self.declare_parameter('minimum_jacobian_norm', 1e-6)
        self.declare_parameter('maximum_abs_velocity', 20.0)
        self.declare_parameter('maximum_state_age_s', 0.15)
        self.declare_parameter('maximum_jacobian_age_s', 0.15)

        names = [str(value) for value in self.get_parameter('arm_names').value]
        topic_groups = [
            [str(value) for value in self.get_parameter(parameter).value]
            for parameter in (
                'state_topics',
                'goal_topics',
                'jacobian_topics',
                'velocity_reference_topics',
            )
        ]
        if not names or any(len(group) != len(names) for group in topic_groups):
            raise ValueError('arm names and topic arrays must have equal non-zero length')
        rate = float(self.get_parameter('control_rate_hz').value)
        self._gain = float(self.get_parameter('gain').value)
        self._damping = float(self.get_parameter('damping').value)
        self._minimum_norm = float(
            self.get_parameter('minimum_jacobian_norm').value
        )
        self._maximum_velocity = float(
            self.get_parameter('maximum_abs_velocity').value
        )
        self._maximum_state_age = float(
            self.get_parameter('maximum_state_age_s').value
        )
        self._maximum_jacobian_age = float(
            self.get_parameter('maximum_jacobian_age_s').value
        )
        if rate <= 0.0 or self._maximum_state_age <= 0.0:
            raise ValueError('control rate and freshness limits must be positive')

        self._arms = {name: ArmRuntime(name=name) for name in names}
        self._velocity_publishers = {}
        self._arm_subscriptions = []
        state_topics, goal_topics, jacobian_topics, output_topics = topic_groups
        for index, name in enumerate(names):
            self._velocity_publishers[name] = self.create_publisher(
                Float64, output_topics[index], 10
            )
            self._arm_subscriptions.extend([
                self.create_subscription(
                    PointCloud,
                    state_topics[index],
                    lambda message, arm=name: self._state_callback(arm, message),
                    qos_profile_sensor_data,
                ),
                self.create_subscription(
                    PointCloud,
                    goal_topics[index],
                    lambda message, arm=name: self._goal_callback(arm, message),
                    10,
                ),
                self.create_subscription(
                    Float64MultiArray,
                    jacobian_topics[index],
                    lambda message, arm=name: self._jacobian_callback(arm, message),
                    qos_profile_sensor_data,
                ),
            ])
        self._status_publisher = self.create_publisher(String, 'shape_control/status', 10)
        self._cycle = 0
        self._timer = self.create_timer(1.0 / rate, self._control_cycle)
        self.get_logger().info(
            f'Single-input shape controller active for {", ".join(names)}; '
            'outputs are encoder-velocity references, not PWM'
        )

    def _state_callback(self, arm_name: str, message: PointCloud) -> None:
        try:
            state = point_cloud_vector(message)
        except ValueError as error:
            self.get_logger().warning(f'{arm_name}: rejected state: {error}')
            return
        arm = self._arms[arm_name]
        arm.state = state
        arm.state_received_at = time.monotonic()

    def _goal_callback(self, arm_name: str, message: PointCloud) -> None:
        try:
            self._arms[arm_name].goal = point_cloud_vector(message)
        except ValueError as error:
            self.get_logger().warning(f'{arm_name}: rejected goal: {error}')

    def _jacobian_callback(
        self, arm_name: str, message: Float64MultiArray
    ) -> None:
        jacobian = np.asarray(message.data, dtype=np.float64).reshape(-1)
        if jacobian.shape != (6,) or not np.isfinite(jacobian).all():
            self.get_logger().warning(
                f'{arm_name}: Jacobian must contain six finite values'
            )
            return
        arm = self._arms[arm_name]
        arm.jacobian = jacobian
        arm.jacobian_received_at = time.monotonic()

    def _evaluate(self, arm: ArmRuntime, now: float) -> tuple[float, str]:
        if arm.state is None or arm.goal is None or arm.jacobian is None:
            return 0.0, 'waiting_for_inputs'
        if now - arm.state_received_at > self._maximum_state_age:
            return 0.0, 'stale_state'
        if now - arm.jacobian_received_at > self._maximum_jacobian_age:
            return 0.0, 'stale_jacobian'
        try:
            velocity = resolved_rate_scalar(
                arm.state,
                arm.goal,
                arm.jacobian,
                gain=self._gain,
                damping=self._damping,
                maximum_abs_velocity=self._maximum_velocity,
                minimum_jacobian_norm=self._minimum_norm,
            )
        except ValueError:
            return 0.0, 'invalid_numeric_input'
        if velocity == 0.0 and np.linalg.norm(arm.jacobian) < self._minimum_norm:
            return 0.0, 'degenerate_jacobian'
        return velocity, 'active'

    def _control_cycle(self) -> None:
        now = time.monotonic()
        status = {}
        for name, arm in self._arms.items():
            velocity, reason = self._evaluate(arm, now)
            message = Float64()
            message.data = velocity
            self._velocity_publishers[name].publish(message)
            arm.last_reason = reason
            status[name] = {'velocity_reference': velocity, 'state': reason}
        self._cycle += 1
        if self._cycle % 50 == 0:
            message = String()
            message.data = json.dumps(status, separators=(',', ':'))
            self._status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ShapeController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
