import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker  
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math
import numpy as np


class SLAMNode(Node):
    def __init__(self):
        super().__init__('SLAM_node')

        # Declaración de parámetros ajustables para el nodo 
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de SLAM desactivado.")
            return

        # Inicializar lógica de cierre
        self.initialize_shutdown_listener()

        # Inicializar variables del mapa
        self.map_resolution = 0.05  # Resolución en metros/celda
        self.map_width = int((10 * 2) / self.map_resolution)  # 10 metros a ambos lados
        self.map_height = int((10 * 2) / self.map_resolution)
        self.map_origin_x = -10.0  # Centro del mapa en (0,0)
        self.map_origin_y = -10.0
        self.map_data = np.zeros((self.map_height, self.map_width), dtype=int)

        # Publicadores
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.shutdown_subscription = self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.marker_publisher = self.create_publisher(Marker, '/person_marker', 10)     #Publicador del marker para situar a la persona en el mapa

        # Suscripciones 
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)       #Suscripcion a los datos del lidar
        self.person_position_subscription = self.create_subscription(Point, '/person_position', self.person_position_callback, 10)      #Suscripcion a la posicion de la persona detectada

        # Broadcaster para transformaciones estáticas
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # Timer para publicar el mapa periódicamente
        self.create_timer(1.0, self.publish_map)

        # Lista para guardar las posiciones de las personas detectadas
        self.person_positions = {}


    def initialize_shutdown_listener(self):
        """Inicializa el suscriptor para manejar el cierre del sistema."""
        self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.shutdown_confirmation_publisher = self.create_publisher(Bool, '/shutdown_confirmation', 10)

    def shutdown_callback(self, msg):
        """Callback para manejar la notificación de cierre del sistema."""
        if msg.data:
            self.get_logger().info("Cierre del sistema detectado. Enviando confirmación.")
            try:
                self.shutdown_confirmation_publisher.publish(Bool(data=True))
            except Exception as e:
                self.get_logger().error(f"Error al publicar confirmación de apagado: {e}")
            finally:
                self.destroy_node()

    def person_position_callback(self, msg):
        """Callback para recibir la posición de la persona detectada."""
        person_id = int(msg.z)  # Usamos z como identificador único
        #self.get_logger().info(f"Posición de la persona detectada: ({msg.x}, {msg.y}), ID: {person_id}")

        # Convertir las coordenadas de la persona al sistema de coordenadas del mapa
        map_x = int((msg.x - self.map_origin_x) / self.map_resolution)
        map_y = int((msg.y - self.map_origin_y) / self.map_resolution)

        # Verificar si la posición está dentro de los límites del mapa
        if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
            self.map_data[map_y, map_x] = 100  # Marca la posición como ocupada

            # Guardar la posición en el diccionario de personas
            self.person_positions[person_id] = (msg.x, msg.y)

            # Publicar un Marker para la persona detectada (círculo)
            self.publish_person_marker(msg.x, msg.y)

    def listener_callback(self, msg):
        """Procesa los datos del LIDAR y actualiza el mapa ocupacional."""
        #self.get_logger().info("Recibiendo datos del LIDAR...")

        # Limpia el mapa anterior
        self.map_data.fill(0)

        angle = msg.angle_min
        for r in msg.ranges:
            # Ignorar valores fuera del rango del LIDAR
            if msg.range_min < r < msg.range_max:
                # Convertir coordenadas polares a cartesianas
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                # Convertir coordenadas a índices del mapa
                map_x = int((x - self.map_origin_x) / self.map_resolution)
                map_y = int((y - self.map_origin_y) / self.map_resolution)

                # Verificar si los índices están dentro del mapa
                if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                    self.map_data[map_y, map_x] = 100  # Marca como ocupado

            # Incrementar el ángulo en cada iteración
            angle += msg.angle_increment

        #self.get_logger().info("Actualización del mapa completada.")
    

    def publish_person_marker(self, x, y):
        """Publica un Marker (círculo) para la persona detectada."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0  # El identificador del marcador puede variar si tienes múltiples personas
        marker.type = Marker.SPHERE  # Usar un marcador de tipo esfera
        marker.action = Marker.ADD

        # Tamaño del círculo
        marker.scale.x = 0.5  # Radio en el eje X
        marker.scale.y = 0.5  # Radio en el eje Y
        marker.scale.z = 0.1  # Puedes hacer que sea más plano si deseas

        # Color del círculo
        marker.color.r = 0.0  # Rojo
        marker.color.g = 1.0  # Verde
        marker.color.b = 1.0  # Azul
        marker.color.a = 1.0  # Opacidad (1.0 es completamente visible)

        # Posición del marcador (en el espacio del mapa)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  # Altura del marcador (puedes ajustarlo si es necesario)

        # Publicar el marcador
        self.marker_publisher.publish(marker)


    def publish_map(self):
        """Publica el mapa actualizado basado en los datos del LIDAR."""
        self.get_logger().debug(f"Publicando mapa con datos únicos: {np.unique(self.map_data)}")

        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Aplanar el mapa y publicarlo
        map_msg.data = self.map_data.flatten().tolist()
        self.map_publisher.publish(map_msg)

    def publish_static_transform(self):
        """Publica la transformación estática entre 'map' y 'odom'."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Transformación estática publicada entre 'map' y 'odom'.")

    def destroy_node(self):
        self.get_logger().info("Nodo SLAM detenido.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo de SLAM detenido manualmente.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()