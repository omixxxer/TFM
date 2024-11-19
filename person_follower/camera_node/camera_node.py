import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declarar parámetros
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de Cámara desactivado.")
            return

        # Inicializar cámara y publicadores
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.status_publisher = self.create_publisher(String, '/camera/status', 10)
        self.timer = self.create_timer(0.1, self.publish_image)

        # Inicializar lógica de cierre
        self.initialize_shutdown_listener()

        self.get_logger().info("Nodo de Cámara iniciado, publicando imágenes.")

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error al publicar la imagen: {e}")

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


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo de Cámara detenido manualmente.")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
