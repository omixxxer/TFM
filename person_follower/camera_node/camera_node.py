import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
import time


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declarar parámetros
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de Cámara desactivado.")
            return

        # Inicializar suscriptor al topic de imágenes publicadas por usb_cam_node_exe
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            '/image_raw',  # Ajusta el topic si es necesario
            self.process_image_callback,
            10
        )

        # Publicador de estado
        self.status_publisher = self.create_publisher(String, '/camera/status', 10)

        # Inicializar lógica de cierre
        self.initialize_shutdown_listener()

        # Control de frecuencia
        self.last_processed_time = time.time()
        self.processing_interval = 0.5  # Procesar una imagen cada 0.5 segundos

        self.get_logger().info("Nodo de Cámara personalizado iniciado, procesando imágenes.")

    def process_image_callback(self, msg):
        """Callback para procesar las imágenes recibidas."""
        current_time = time.time()
        if current_time - self.last_processed_time < self.processing_interval:
            return  # Saltar procesamiento si no ha pasado suficiente tiempo

        self.last_processed_time = current_time
        try:
            self.get_logger().info("Imagen recibida. Iniciando procesamiento...")

            # Convertir mensaje ROS a imagen OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.get_logger().info("Imagen convertida a formato OpenCV.")

            # Procesar la imagen (ejemplo: convertir a escala de grises)
            processed_frame = self.process_image(frame)
            self.get_logger().info("Procesamiento de imagen completado.")

            # Mostrar la imagen procesada
            cv2.imshow("Processed Image", processed_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error procesando la imagen: {e}")

    def process_image(self, frame):
        """Realiza el procesamiento adicional de la imagen."""
        # Ejemplo: Convertir la imagen a escala de grises
        self.get_logger().debug("Convirtiendo imagen a escala de grises...")
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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
                cv2.destroyAllWindows()  # Liberar ventanas de OpenCV al cerrar el nodo


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido manualmente.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
