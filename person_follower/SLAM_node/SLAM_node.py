import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math
import numpy as np


class SLAMNode(Node):
    def __init__(self):
        super().__init__('SLAM_node')

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

        # Suscripción al LIDAR
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)

        # Broadcaster para transformaciones estáticas
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # Timer para publicar el mapa periódicamente
        self.create_timer(1.0, self.publish_map)

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

    def listener_callback(self, msg):
        """Procesa los datos del LIDAR y actualiza el mapa ocupacional."""
        self.get_logger().info("Recibiendo datos del LIDAR...")

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

        self.get_logger().info("Actualización del mapa completada.")

    def publish_map(self):
        """Publica el mapa actualizado basado en los datos del LIDAR."""
        self.get_logger().info(f"Publicando mapa con datos únicos: {np.unique(self.map_data)}")

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
