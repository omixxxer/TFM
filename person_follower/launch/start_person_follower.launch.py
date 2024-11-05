# start_person_follower.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='person_follower',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[
                {'collision_enabled': False},  # Activa o desactiva el nodo de colisión aquí
                {'tracking_enabled': True},   # Activa o desactiva el nodo de seguimiento
                {'camera_enabled': True},     # Activa o desactiva el nodo de cámara
                {'ui_enabled': True}          # Activa o desactiva el nodo de interfaz de usuario
            ],
            remappings=[('/cmd_vel', '/commands/velocity')]
        ),
        Node(
            package='person_follower',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{'enabled': True}]
        ),
        Node(
            package='person_follower',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[{'enabled': True}]
        ),
        Node(
            package='person_follower',
            executable='tracking_node',
            name='tracking_node',
            output='screen',
            parameters=[{'enabled': True}],
            remappings=[('/cmd_vel', '/commands/velocity')]
        ),
        Node(
            package='person_follower',
            executable='collision_handling_node',
            name='collision_handling_node',
            output='screen',
            remappings=[('/cmd_vel', '/commands/velocity')]
        ),
        Node(
            package='person_follower',
            executable='user_interface_node',
            name='user_interface_node',
            output='screen',
            parameters=[{'enabled': True}]
        )
    ])

