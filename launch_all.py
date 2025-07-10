from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='drone_controller', executable='waypoint_controller', output='screen'),
        Node(package='drone_controller', executable='mqtt_listener', output='screen'),
    ])
