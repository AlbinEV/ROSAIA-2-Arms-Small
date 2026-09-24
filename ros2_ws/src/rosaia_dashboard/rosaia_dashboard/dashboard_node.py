"""Serve a live web dashboard backed by ROS 2 sensor and goal topics."""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosaia_learning.model import NumpyMlpModel
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String

from .projection import project_planar_goal


def _cloud_payload(message: PointCloud) -> dict:
    """Convert a complete three-marker cloud into JSON-compatible values."""
    if len(message.points) != 3:
        raise ValueError('shape cloud must contain exactly three points')
    points = [
        {'x': float(point.x), 'y': float(point.y), 'z': float(point.z)}
        for point in message.points
    ]
    if not all(math.isfinite(value) for point in points for value in point.values()):
        raise ValueError('shape cloud contains non-finite coordinates')
    return {'frame_id': message.header.frame_id, 'points': points}


def _model_summary(directory: str, model) -> dict:
    """Read compact validation and approval information for the UI."""
    summary = {
        'available': model is not None,
        'approved': False,
        'converged': None,
        'direction': None,
        'validation_rmse_mm': None,
        'validation_p95_mm': None,
        'q_min': None if model is None else model.q_min,
        'q_max': None if model is None else model.q_max,
    }
    if model is None:
        return summary
    try:
        metadata = json.loads((Path(directory) / 'metadata.json').read_text())
    except (OSError, json.JSONDecodeError):
        return summary
    training = metadata.get('training', {})
    approval = metadata.get('approval', {})
    summary.update({
        'approved': approval.get('status') == 'approved',
        'converged': training.get('converged'),
        'direction': training.get('direction'),
        'validation_rmse_mm': 1000.0 * float(
            training['validation_rmse_total']
        ) if training.get('validation_rmse_total') is not None else None,
        'validation_p95_mm': 1000.0 * float(
            training['validation_p95_total']
        ) if training.get('validation_p95_total') is not None else None,
    })
    return summary


class DashboardNode(Node):
    """Aggregate live sensors and expose feasible visual goals over HTTP."""

    def __init__(self) -> None:
        super().__init__('rosaia_dashboard')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8765)
        self.declare_parameter('arm_names', ['arm_1', 'arm_2'])
        self.declare_parameter(
            'state_topics', ['/arm_1/aruco_state', '/arm_2/aruco_state']
        )
        self.declare_parameter(
            'goal_topics', ['/arm_1/shape_goal', '/arm_2/shape_goal']
        )
        self.declare_parameter('model_directories', ['', ''])
        self.declare_parameter('arm_1_model_directory', '')
        self.declare_parameter('arm_2_model_directory', '')
        self.declare_parameter('enable_goal_publish', False)

        names = [str(value) for value in self.get_parameter('arm_names').value]
        state_topics = [
            str(value) for value in self.get_parameter('state_topics').value
        ]
        goal_topics = [
            str(value) for value in self.get_parameter('goal_topics').value
        ]
        directories = [
            str(value)
            for value in self.get_parameter('model_directories').value
        ]
        directory_overrides = [
            str(self.get_parameter('arm_1_model_directory').value),
            str(self.get_parameter('arm_2_model_directory').value),
        ]
        directories = [
            override or directory
            for override, directory in zip(directory_overrides, directories)
        ]
        lengths = {len(names), len(state_topics), len(goal_topics), len(directories)}
        if not names or len(lengths) != 1:
            raise ValueError('per-arm dashboard parameter lengths must match')

        self._lock = threading.Lock()
        self._goal_enabled = bool(
            self.get_parameter('enable_goal_publish').value
        )
        self._state = {
            'goal_publish_enabled': self._goal_enabled,
            'vision': None,
            'motor': None,
            'arms': {},
        }
        self._models = {}
        self._model_summaries = {}
        self._goal_publishers = {}
        self._owned_subscriptions = []
        for name, state_topic, goal_topic, directory in zip(
            names, state_topics, goal_topics, directories
        ):
            model = NumpyMlpModel.load(directory) if directory else None
            self._models[name] = model
            self._model_summaries[name] = _model_summary(directory, model)
            self._goal_publishers[name] = self.create_publisher(
                PointCloud, goal_topic, 10
            )
            self._state['arms'][name] = {
                'current': None,
                'requested': None,
                'feasible': None,
                'projection': None,
                'model': self._model_summaries[name],
            }
            self._owned_subscriptions.append(self.create_subscription(
                PointCloud,
                state_topic,
                lambda message, arm=name: self._on_shape(arm, message),
                qos_profile_sensor_data,
            ))
        self._owned_subscriptions.extend([
            self.create_subscription(String, '/motor_state', self._on_motor, 20),
            self.create_subscription(String, '/aruco/status', self._on_vision, 10),
        ])

        web_root = Path(get_package_share_directory('rosaia_dashboard')) / 'web'
        self._index = (web_root / 'index.html').read_bytes()
        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        if not 0 < port < 65536:
            raise ValueError('dashboard port must be in 1..65535')
        self._server = ThreadingHTTPServer(
            (host, port), self._handler_class()
        )
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name='rosaia_dashboard_http',
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(
            f'dashboard ready at http://{host}:{port}; '
            f'goal publishing enabled={self._goal_enabled}'
        )

    def _on_shape(self, arm_name: str, message: PointCloud) -> None:
        try:
            payload = _cloud_payload(message)
        except ValueError as error:
            self.get_logger().warning(f'{arm_name}: {error}')
            return
        with self._lock:
            self._state['arms'][arm_name]['current'] = payload

    def _on_motor(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._state['motor'] = payload

    def _on_vision(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._state['vision'] = payload

    def snapshot(self) -> dict:
        """Return an isolated JSON round-trip copy of current dashboard state."""
        with self._lock:
            return json.loads(json.dumps(self._state))

    def set_goal(self, payload: dict) -> tuple[int, dict]:
        """Project a requested planar shape and optionally publish it."""
        arm_name = str(payload.get('arm', ''))
        if arm_name not in self._models:
            return HTTPStatus.BAD_REQUEST, {'error': 'unknown arm'}
        points = payload.get('points')
        if not isinstance(points, list) or len(points) != 3:
            return HTTPStatus.BAD_REQUEST, {'error': 'exactly three points required'}
        try:
            values = np.array([
                [float(point['x']), float(point['y'])] for point in points
            ], dtype=np.float64).reshape(6)
        except (KeyError, TypeError, ValueError):
            return HTTPStatus.BAD_REQUEST, {'error': 'invalid point coordinates'}
        if not np.isfinite(values).all():
            return HTTPStatus.BAD_REQUEST, {'error': 'coordinates must be finite'}
        model = self._models[arm_name]
        if model is None:
            return HTTPStatus.CONFLICT, {'error': 'no validated model configured'}

        result = project_planar_goal(model, values)
        current = self.snapshot()['arms'][arm_name]['current']
        z_values = (
            [point['z'] for point in current['points']]
            if current is not None else [0.0, 0.0, 0.0]
        )
        requested = [
            {'x': float(values[2 * index]), 'y': float(values[2 * index + 1]),
             'z': float(z_values[index])}
            for index in range(3)
        ]
        feasible = [
            {'x': float(result.state[2 * index]),
             'y': float(result.state[2 * index + 1]),
             'z': float(z_values[index])}
            for index in range(3)
        ]
        projection = {
            'q': result.q,
            'weighted_error_m': result.weighted_error,
        }
        with self._lock:
            arm_state = self._state['arms'][arm_name]
            arm_state['requested'] = {'points': requested}
            arm_state['feasible'] = {'points': feasible}
            arm_state['projection'] = projection

        publish = bool(payload.get('publish', False))
        if publish:
            if not self._goal_enabled:
                return HTTPStatus.FORBIDDEN, {
                    'error': 'goal publishing is disabled',
                    'projection': projection,
                    'feasible': feasible,
                }
            if not self._model_summaries[arm_name]['approved']:
                return HTTPStatus.FORBIDDEN, {
                    'error': 'model is not approved for online control',
                    'projection': projection,
                    'feasible': feasible,
                }
            message = PointCloud()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = (
                current['frame_id'] if current is not None
                else f'{arm_name}_base'
            )
            from geometry_msgs.msg import Point32
            message.points = [Point32(**point) for point in feasible]
            self._goal_publishers[arm_name].publish(message)
        return HTTPStatus.OK, {
            'projection': projection,
            'requested': requested,
            'feasible': feasible,
            'published': publish,
        }

    def _handler_class(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def _json(self, status, payload):
                encoded = json.dumps(payload, separators=(',', ':')).encode()
                self.send_response(int(status))
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def do_GET(self):
                if self.path == '/':
                    self.send_response(HTTPStatus.OK)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(node._index)))
                    self.end_headers()
                    self.wfile.write(node._index)
                elif self.path == '/api/state':
                    self._json(HTTPStatus.OK, node.snapshot())
                else:
                    self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})

            def do_POST(self):
                if self.path != '/api/goal':
                    self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if length <= 0 or length > 65536:
                        raise ValueError('invalid body length')
                    payload = json.loads(self.rfile.read(length))
                except (ValueError, json.JSONDecodeError):
                    self._json(HTTPStatus.BAD_REQUEST, {'error': 'invalid JSON'})
                    return
                status, response = node.set_goal(payload)
                self._json(status, response)

            def log_message(self, *_args):
                return

        return Handler

    def destroy_node(self):
        if hasattr(self, '_server'):
            self._server.shutdown()
            self._server.server_close()
            self._server_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the ROS-backed HTTP dashboard."""
    rclpy.init(args=args)
    node = None
    try:
        node = DashboardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
