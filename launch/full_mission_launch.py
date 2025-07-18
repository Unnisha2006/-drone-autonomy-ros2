from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_mission',
            executable='mission_controller',
            name='mission_controller',
            output='screen'
        ),
        Node(
            package='drone_mission',
            executable='mqtt_listener',
            name='mqtt_listener',
            output='screen'
        ),
        Node(
            package='drone_mission',
            executable='telemetry_publisher',
            name='telemetry_publisher',
            output='screen'
        )
    ])
