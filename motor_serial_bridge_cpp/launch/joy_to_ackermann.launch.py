from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('motor_serial_bridge_cpp')
    default_config = os.path.join(pkg_share, 'config', 'joy_to_ackermann.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to joy_to_ackermann yaml config file'
    )

    node = Node(
        package='motor_serial_bridge_cpp',
        executable='joy_to_ackermann_node',
        name='joy_to_ackermann_node',
        output='screen',
        parameters=[LaunchConfiguration('config')]
    )

    return LaunchDescription([
        config_arg,
        node,
    ])
