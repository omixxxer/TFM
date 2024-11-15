import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Declaración de parámetros con valores predeterminados
        self.declare_parameter('tracking_enabled', True)
        self.declare_parameter('camera_enabled', True)
        self.declare_parameter('ui_enabled', True)
        self.declare_parameter('detection_enabled', True)

        # Inicialización de parámetros
        self.tracking_enabled = self.get_parameter('tracking_enabled').value
        self.camera_enabled = self.get_parameter('camera_enabled').value
        self.ui_enabled = self.get_parameter('ui_enabled').value
        self.detection_enabled = self.get_parameter('detection_enabled').value

        # Estado del sistema
        self.person_detected = False
        self.tracking_service_ready = False

        # Subscripciones
        self.create_subscription(String, '/camera/status', self.camera_status_callback, 10)
        self.create_subscription(String, '/detection/status', self.detection_status_callback, 10)
        self.create_subscription(String, '/tracking/status', self.tracking_status_callback, 10)
        self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)

        # Cliente para activar/desactivar el seguimiento
        self.tracking_client = self.create_client(SetBool, 'enable_tracking')
        self.wait_for_service(self.tracking_client, 'enable_tracking')

        self.get_logger().info("Nodo de Control iniciado con las funcionalidades configuradas.")

    def wait_for_service(self, client, service_name):
        """Espera hasta que el servicio esté disponible."""
        while not client.service_is_ready():
            self.get_logger().info(f"Esperando el servicio {service_name}...")
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info(f"Servicio {service_name} listo.")
        self.tracking_service_ready = True

    def camera_status_callback(self, msg):
        self.get_logger().info(f"Estado de la cámara: {msg.data}")

    def detection_status_callback(self, msg):
        self.get_logger().info(f"Estado del nodo de detección: {msg.data}")

    def tracking_status_callback(self, msg):
        self.get_logger().info(f"Estado del nodo de seguimiento: {msg.data}")

    def person_detected_callback(self, msg):
        try:
            if msg.data != self.person_detected:
                self.person_detected = msg.data
                self.get_logger().info(f"Persona {'detectada' if self.person_detected else 'no detectada'}.")
                self.toggle_tracking(self.person_detected)
        except Exception as e:
            self.get_logger().error(f"Error al procesar la detección de persona: {e}")

    def toggle_tracking(self, enable):
        """Habilita o deshabilita el seguimiento según la detección de personas."""
        if not self.tracking_service_ready:
            self.get_logger().warn("El servicio de seguimiento aún no está listo.")
            return

        request = SetBool.Request()
        request.data = enable
        future = self.tracking_client.call_async(request)
        future.add_done_callback(self.handle_tracking_response)

    def handle_tracking_response(self, future):
        """Maneja la respuesta del servicio de seguimiento."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Seguimiento {'habilitado' if response.message == 'enabled' else 'deshabilitado'}: {response.message}")
            else:
                self.get_logger().warn(f"No se pudo cambiar el estado del seguimiento: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Error en la respuesta del servicio de seguimiento: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo de Control detenido manualmente.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
