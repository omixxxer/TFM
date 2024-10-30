import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('user_interface_node')
        self.get_logger().info("Nodo de Interfaz de Usuario iniciado")

def main(args=None):
    rclpy.init(args=args)
    node = UserInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

