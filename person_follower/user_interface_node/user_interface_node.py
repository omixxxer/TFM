import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('user_interface_node')

        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Interfaz de Usuario desactivado.")
            return

        # Suscripciones
        self.cluster_subscription = self.create_subscription(MarkerArray, '/detection/clusters', self.cluster_callback, 10)
        self.camera_subscription = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.state_subscription = self.create_subscription(String, '/robot_state', self.state_callback, 10)

        self.bridge = CvBridge()
        self.last_state = None
        self.get_logger().info("Nodo de Interfaz de Usuario iniciado")

    def cluster_callback(self, msg):
        self.get_logger().info("Datos de clusters recibidos.")
        # No es necesario procesar aquí, ya que RViz visualiza directamente desde el topic.

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Vista de la cámara", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")

    def state_callback(self, msg):
        if msg.data != self.last_state:
            self.last_state = msg.data
            self.get_logger().info(f"Estado del robot: {msg.data}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UserInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
