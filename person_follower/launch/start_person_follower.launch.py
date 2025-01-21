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

    # Rutas de los modelos YOLO
    model_path = os.path.join(package_share_directory, 'model')
    yolov4_weights_path = os.path.join(model_path, 'yolov4-tiny.weights')
    yolov4_cfg_path = os.path.join(model_path, 'yolov4-tiny.cfg')
    coco_names_path = os.path.join(model_path, 'coco.names')

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
            parameters=[{
                'enabled': True,
                'yolov4_weights_path': yolov4_weights_path,
                'yolov4_cfg_path': yolov4_cfg_path,
                'coco_names_path': coco_names_path
            }]
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
        ),
        Node(
            package='person_follower',
            executable='slam_node',
            name='slam_node',
            output='screen',
            parameters=[config_path]
        )
    ])
