import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class CollisionHandlingNode(Node):
    def __init__(self):
        super().__init__('collision_handling_node')
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Nodo de Manejo de Colisiones iniciado")

    def lidar_callback(self, msg):
        if min(msg.ranges) < 0.4:  # Si un obstáculo está cerca
            twist = Twist()
            twist.linear.x = 0.0  # Detener el robot
            self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CollisionHandlingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

