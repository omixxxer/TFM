# control_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.get_logger().info("Nodo de Control iniciado")
        
        # Aquí puedes suscribirte o publicar a topics de otros nodos según lo necesites
        # Ejemplo de suscripción
        self.detection_subscriber = self.create_subscription(String, '/detection_info', self.detection_callback, 10)

    def detection_callback(self, msg):
        self.get_logger().info(f"Información de detección recibida: {msg.data}")
        # Lógica para tomar decisiones basada en la detección

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

