from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('person_follower'),
        'config',
        'config.yaml'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('person_follower'),
        'config',
        'tb2.rviz'  # Archivo de configuración de RViz 2
    )

    return LaunchDescription([
        Node(
            package='person_follower',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[config_path],
            remappings=[('/cmd_vel', '/commands/velocity')]
        ),
        Node(
            package='person_follower',
            executable='tracking_node',
            name='tracking_node',
            output='screen',
            parameters=[config_path],
            remappings=[('/cmd_vel', '/commands/velocity')]
        ),
        Node(
            package='person_follower',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='person_follower',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='person_follower',
            executable='user_interface_node',
            name='user_interface_node',
            output='screen',
            parameters=[config_path]
        )
    ])
