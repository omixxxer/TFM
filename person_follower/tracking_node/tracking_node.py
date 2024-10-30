import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detection_subscriber = self.create_subscription(String, '/detection_info', self.detection_callback, 10)
        self.get_logger().info("Nodo de Seguimiento iniciado")

    def detection_callback(self, msg):
        # Ejemplo básico de comando de seguimiento
        twist = Twist()
        twist.linear.x = 0.2  # Velocidad hacia adelante
        twist.angular.z = 0.0  # Ajustar dirección según detección
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

