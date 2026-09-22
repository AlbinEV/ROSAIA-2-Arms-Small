"""Launch the timestamp-aware experiment recorder."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Create a recorder with overridable session name and output root."""
    share = Path(get_package_share_directory('rosaia_data_acquisition'))
    name_argument = DeclareLaunchArgument('session_name', default_value='')
    root_argument = DeclareLaunchArgument(
        'output_root', default_value='data/sessions'
    )
    recorder = Node(
        package='rosaia_data_acquisition',
        executable='session_recorder',
        name='session_recorder',
        output='screen',
        parameters=[
            str(share / 'config' / 'recorder.yaml'),
            {
                'session_name': LaunchConfiguration('session_name'),
                'output_root': LaunchConfiguration('output_root'),
            },
        ],
    )
    return LaunchDescription([name_argument, root_argument, recorder])
