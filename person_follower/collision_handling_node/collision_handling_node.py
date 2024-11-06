import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import String

class CollisionHandlingNode(Node):
    def __init__(self):
        super().__init__('collision_handling_node')
        
        # Declaración del parámetro 'enabled' y verificación de activación
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Manejo de Colisiones desactivado.")
            self.publish_status("Nodo de Manejo de Colisiones desactivado.")
            return

        # Inicialización de suscriptores y publicadores solo si el nodo está activo
        self.status_publisher = self.create_publisher(String, '/collision_handling/status', 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.collision_publisher = self.create_publisher(Bool, '/collision_detected', 10)
        self.min_distance = 0.4  # Distancia mínima para detección de colisiones
        self.collision_active = False
        self.get_logger().info("Nodo de Manejo de Colisiones iniciado")
        self.publish_status("Nodo de Manejo de Colisiones iniciado.")
        
    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))

    def lidar_callback(self, msg):
        # Detecta si hay un obstáculo a menos de `min_distance`
        closest_distance = min(msg.ranges)
        collision_detected = closest_distance < self.min_distance
        if collision_detected != self.collision_active:
            self.collision_active = collision_detected
            self.collision_publisher.publish(Bool(data=self.collision_active))
            if collision_detected:
                self.get_logger().warn("Posible colisión detectada, informando al Nodo de Control")
            else:
                self.get_logger().info("Colisión resuelta, informando al Nodo de Control")

        # Publicar la distancia más cercana detectada
        self.publish_closest_distance(closest_distance)

    def publish_closest_distance(self, distance):
        self.get_logger().info(f"Distancia más cercana: {distance:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = CollisionHandlingNode()
    if node.enabled:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

