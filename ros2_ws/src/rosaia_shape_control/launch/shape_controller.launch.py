from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('rosaia_shape_control'))
    return LaunchDescription([
        Node(
            package='rosaia_shape_control',
            executable='shape_controller',
            name='shape_controller',
            output='screen',
            parameters=[str(share / 'config' / 'two_arms.yaml')],
        )
    ])
