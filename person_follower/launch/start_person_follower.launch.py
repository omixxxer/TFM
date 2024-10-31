from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='person_follower',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
        Node(
            package='person_follower',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='person_follower',
            executable='detection_node',
            name='detection_node',
            output='screen'
        ),
        Node(
            package='person_follower',
            executable='tracking_node',
            name='tracking_node',
            output='screen',
            remappings=[('/cmd_vel', '/commands/velocity')]  # Ajustar el topic si es necesario
        ),
        Node(
            package='person_follower',
            executable='collision_handling_node',
            name='collision_handling_node',
            output='screen'
        ),
        Node(
            package='person_follower',
            executable='user_interface_node',
            name='user_interface_node',
            output='screen'
        )
    ])
