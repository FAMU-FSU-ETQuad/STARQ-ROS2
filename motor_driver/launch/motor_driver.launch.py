from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    motor_driver_dir = get_package_share_directory('motor_driver')

    declare_config_file_argument = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(motor_driver_dir, 'config', 'motors.yaml'),
        description='Path to the config file for motors'
    )

    motor_driver_node = Node(
        package='motor_driver',
        executable='motor_driver_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        declare_config_file_argument,
        motor_driver_node,
    ])