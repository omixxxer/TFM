# TFM - Person Follower (ROS 2)

## Autoría

- Autor principal del proyecto: omixxxer.
- Repositorio académico asociado al Trabajo de Fin de Grado/Máster (TFM).
- La identidad canónica de Git para nuevas contribuciones está definida en `.mailmap`.

## Contexto del TFM

Este repositorio contiene el desarrollo de un sistema de seguimiento de personas para TurtleBot2 usando ROS 2, sensores LIDAR/cámara y nodos de control, detección, seguimiento e interfaz.

## Licencia y uso permitido

- Este proyecto se distribuye bajo licencia Apache-2.0.
- Consulta los archivos `LICENSE` y `NOTICE` para permisos, obligaciones de atribución y limitaciones.
- Se permite reutilización y forks (incluyendo uso académico), manteniendo aviso de copyright y licencia.

## Cita académica

Si usas este repositorio en docencia, investigación o derivados, cita el proyecto con `CITATION.cff`.

## Verificación rápida de autoría y trazabilidad

1. Comprobar licencia:

       ls LICENSE NOTICE

2. Comprobar consolidación de identidades:

       git shortlog -sne --all

3. Comprobar procedencia por commit:

       git log --all --pretty=format:"%h|%an|%ae|%s" --date=short

4. Comprobar firmas en commits recientes:

       git log --all --pretty=format:"%h|%G?|%GK|%an|%ae|%s" -n 50

## Política de commits (recomendado)

- Usar un email único y estable para nuevos commits.
- Firmar commits nuevos (GPG o firma de GitHub).
- No subir secretos ni contraseñas al repositorio.

## Seguridad operativa

- No se incluyen credenciales en este README.
- Para acceso SSH a robots reales, usar credenciales gestionadas localmente (variables de entorno, gestor de secretos o archivos fuera de Git).


## Guía operativa TurtleBot2

### Encendido

    Turn on the switch on the Kobuki base
    Turn on the external battery pack
    Set the voltage of the battery pack to 20V
    Turn on the NUC
    Plug in the Kobuki USB cable to the NUC (first)
    Plug in the Lidar USB cable to the NUC (second)
    Plug in the Logitech Camera USB cable to the NUC (third)

### Apagado

    Unplug the USB cables of the Kobuki, Camera and the Lidar
    Turn off the NUC (the battery pack will go off automatically)
    Turn off the switch on the Kobuki base
    Put the Kobuki on a charging station
    If necessary, charge the external battery pack

### Requisitos

    The laptop must be connected to the WiFi PIROBOTNET6 or PIROBOTNET6_5G
    The laptop must have ROS 2 Humble installed
    The laptop must have Mediapipe installed

### Instalación de MediaPipe

    sudo apt update
    sudo apt install python3-pip
    pip install mediapipe

### Conexión SSH

    ssh user@192.168.0.224

Cada robot tiene una IP distinta 192.168.0.XXX terminada en 224, 225, 226 o 227 (ver etiqueta en la NUC).

### Lanzar nodo Kobuki

    source /opt/ros/humble/setup.bash
    source ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27 según el robot
    ros2 launch kobuki_node kobuki_node-launch.py

### Teleoperación

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/commands/velocity

### Lanzar nodo Lidar

    source /opt/ros/humble/setup.bash
    source ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_port:=/dev/rplidar

Publicar transform estática:

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    ros2 run tf2_ros static_transform_publisher 0 0 0 3.141592 0 0 base_footprint laser

Comprobar `/scan`:

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    ros2 topic hz /scan

### Lanzar nodo Cámara

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    ros2 run usb_cam usb_cam_node_exe --ros-args -p image_width:=320 -p image_height:=240 -p framerate:=10.0

Comprobar `/image_raw`:

    source /opt/ros/humble/setup.bash
    ros2 topic hz /image_raw

### Crear workspace ROS 2 y compilar

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/omixxxer/TFM.git .
    cd ..
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install

### Visualización

Pose y Lidar:

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    rviz2 -d ros2_ws/rviz/tb2.rviz

Cámara en RViz:

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    rviz2 -d ros2_ws/rviz/camera.rviz

Cámara en terminal:

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 run person_follower camera_viewer.py

### Ejecutar seguimiento de persona

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=24  # or 25, 26, 27
    ros2 launch person_follower start_person_follower.launch.py


