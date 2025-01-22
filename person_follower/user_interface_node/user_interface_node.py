import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import subprocess


class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('user_interface_node')

        # Verificar si el nodo está habilitado
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Interfaz de Usuario desactivado.")
            return

        self.initialize_shutdown_listener()

        # Subscripciones para obtener estados
        self.create_subscription(String, '/camera/status', self.camera_status_callback, 10)
        self.create_subscription(String, '/detection/status', self.detection_status_callback, 10)
        self.create_subscription(String, '/tracking/status', self.tracking_status_callback, 10)
        self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)

        # Estado del sistema
        self.person_detected = False
        self.camera_status = "Desconocido"
        self.detection_status = "Desconocido"
        self.tracking_status = "Desconocido"
        self.previous_status = {}

        # Iniciar RViz2 al iniciar el nodo
        self.start_rviz()

        # Timer para mostrar el estado en la terminal cada 5 segundos
        self.timer = self.create_timer(5.0, self.display_status)

        self.get_logger().info("Nodo de Interfaz de Usuario iniciado.")

    def start_rviz(self):
        """Inicia RViz2 con un archivo de configuración predeterminado."""
        rviz_config_path = '/home/usuario/ros2_ws/src/rviz/config.rviz'  # Ruta de tu archivo config.rviz
        try:
            self.rviz_process = subprocess.Popen(['rviz2', '-d', rviz_config_path])
            self.get_logger().info(f"RViz2 iniciado con configuración: {rviz_config_path}")
        except FileNotFoundError as e:
            self.get_logger().error(f"No se pudo iniciar RViz2. Verifica la instalación. Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error al iniciar RViz2: {e}")

    def stop_rviz(self):
        """Detiene el proceso de RViz2 si está en ejecución."""
        if hasattr(self, 'rviz_process') and self.rviz_process:
            self.rviz_process.terminate()
            self.get_logger().info("RViz2 detenido.")

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
        current_status = {
            "Cámara": self.camera_status,
            "Detección": self.detection_status,
            "Seguimiento": self.tracking_status,
            "Persona detectada": "Sí" if self.person_detected else "No"
        }

        # Mostrar el estado solo si hay cambios
        if current_status != self.previous_status:
            print("\n=== Estado del Sistema ===")
            for key, value in current_status.items():
                print(f"{key}: {value}")
            print("==========================")
            self.previous_status = current_status

    def initialize_shutdown_listener(self):
        """Inicializa el suscriptor para manejar el cierre del sistema."""
        self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.shutdown_confirmation_publisher = self.create_publisher(Bool, '/shutdown_confirmation', 10)

    def shutdown_callback(self, msg):
        """Callback para manejar la notificación de cierre del sistema."""
        if msg.data:
            self.get_logger().info("Cierre del sistema detectado. Enviando confirmación.")
            self.stop_rviz()
            try:
                self.shutdown_confirmation_publisher.publish(Bool(data=True))
            except Exception as e:
                self.get_logger().error(f"Error al publicar confirmación de apagado: {e}")
            finally:
                self.destroy_node()

    def destroy_node(self):
        self.stop_rviz()
        self.get_logger().info("Nodo de Interfaz de Usuario detenido.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UserInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo Interfaz de Usuario detenido manualmente.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
