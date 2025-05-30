from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Rutas del paquete
    package_share_directory = get_package_share_directory('person_follower')

    # Ruta del archivo de configuración
    config_path = os.path.join(package_share_directory, 'config', 'config.yaml')

    # Ruta del archivo de configuración de RViz
    rviz_config_path = os.path.join(package_share_directory, 'config', 'tb2.rviz')

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
            executable='visual_detection_node',
            name='visual_detection_node',
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
