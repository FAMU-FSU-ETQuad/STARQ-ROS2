from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('motor_driver'),
        'config',
        'motors.yaml'
        )

    motor_driver_node = Node(
        package='motor_driver',
        executable='motor_driver_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        motor_driver_node
    ])