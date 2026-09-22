"""Launch the Bluno bridge with the checked-in fail-safe configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the bridge launch description."""
    default_config = os.path.join(
        get_package_share_directory('bluno_motor_bridge'),
        'config',
        'controller.yaml',
    )
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the bridge parameter YAML file',
    )
    bridge = Node(
        package='bluno_motor_bridge',
        executable='bluno_motor_bridge_node',
        name='bluno_motor_bridge',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )
    return LaunchDescription([config_arg, bridge])
