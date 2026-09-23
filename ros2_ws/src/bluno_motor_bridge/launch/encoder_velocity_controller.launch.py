"""Launch the encoder velocity controller."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Load the calibrated two-axis velocity-loop configuration."""
    share = Path(get_package_share_directory('bluno_motor_bridge'))
    return LaunchDescription([
        Node(
            package='bluno_motor_bridge',
            executable='encoder_velocity_controller',
            name='encoder_velocity_controller',
            output='screen',
            parameters=[str(share / 'config' / 'velocity_controller.yaml')],
        )
    ])
