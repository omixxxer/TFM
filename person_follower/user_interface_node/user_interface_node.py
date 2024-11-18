import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('user_interface_node')

        # Verificar si el nodo está habilitado
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Interfaz de Usuario desactivado.")
            return  # Salir si el nodo está desactivado

        # Subscripciones para obtener estados
        self.create_subscription(String, '/camera/status', self.camera_status_callback, 10)
        self.create_subscription(String, '/detection/status', self.detection_status_callback, 10)
        self.create_subscription(String, '/tracking/status', self.tracking_status_callback, 10)
        self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        self.shutdown_subscription = self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)

        # Estado del sistema
        self.person_detected = False
        self.camera_status = "Desconocido"
        self.detection_status = "Desconocido"
        self.tracking_status = "Desconocido"

        # Timer para mostrar el estado en la terminal cada 5 segundos
        self.timer = self.create_timer(5.0, self.display_status)

        self.get_logger().info("Nodo de Interfaz de Usuario iniciado.")

    def initialize_shutdown_listener(self):
        """Inicializa el suscriptor para manejar el cierre del sistema."""
        self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.shutdown_confirmation_publisher = self.create_publisher(Bool, '/shutdown_confirmation', 10)

    
    def shutdown_callback(self, msg):
        """Callback para manejar la notificación de cierre del sistema."""
        if msg.data:
            self.get_logger().info("Cierre del sistema detectado. Enviando confirmación.")
            self.shutdown_confirmation_publisher.publish(Bool(data=True))
            self.destroy_node()
    		
    def camera_status_callback(self, msg):
        self.camera_status = msg.data

    def detection_status_callback(self, msg):
        self.detection_status = msg.data

    def tracking_status_callback(self, msg):
        self.tracking_status = msg.data

    def person_detected_callback(self, msg):
        self.person_detected = msg.data

    def display_status(self):
        """Muestra el estado actual del sistema en la terminal."""
        print("\n=== Estado del Sistema ===")
        print(f"Cámara: {self.camera_status}")
        print(f"Detección: {self.detection_status}")
        print(f"Seguimiento: {self.tracking_status}")
        print(f"Persona detectada: {'Sí' if self.person_detected else 'No'}")
        print("==========================")

        
def main(args=None):
    rclpy.init(args=args)
    node = UserInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo Interfaz de Usuario detenido manualmente.")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
