# collision_handling_node/collision_handling_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class CollisionHandlingNode(Node):
    def __init__(self):
        super().__init__('collision_handling_node')
        
        # Declaración del parámetro 'enabled' y verificación de activación
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Manejo de Colisiones desactivado.")
            return  # Salir si el nodo está desactivado

        # Inicialización de suscriptores y publicadores solo si el nodo está activo
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.collision_publisher = self.create_publisher(Bool, '/collision_detected', 10)
        self.min_distance = 0.2  # Distancia mínima para activar colisión
        self.collision_active = False  # Estado de la colisión
        self.get_logger().info("Nodo de Manejo de Colisiones iniciado")

    def lidar_callback(self, msg):
        # Detecta si hay un obstáculo a menos de `min_distance`
        collision_detected = min(msg.ranges) < self.min_distance
        if collision_detected != self.collision_active:
            self.collision_active = collision_detected
            self.collision_publisher.publish(Bool(data=self.collision_active))
            if collision_detected:
                self.get_logger().warn("Colisión detectada, informando al Nodo de Control")
            else:
                self.get_logger().info("Colisión resuelta, informando al Nodo de Control")

def main(args=None):
    rclpy.init(args=args)
    node = CollisionHandlingNode()
    if node.enabled:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

