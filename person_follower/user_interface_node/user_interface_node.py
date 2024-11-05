# user_interface_node/user_interface_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('user_interface_node')
        
        # Verificar si el nodo está habilitado
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Interfaz de Usuario desactivado.")
            return  # Salir si el nodo está desactivado
        
        self.get_logger().info("Nodo de Interfaz de Usuario iniciado")

def main(args=None):
    rclpy.init(args=args)
    node = UserInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

