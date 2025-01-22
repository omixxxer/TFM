import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Declaración de parámetros con valores predeterminados
        self.declare_parameter('tracking_enabled', True)
        self.declare_parameter('slam_enabled', True)

        # Inicialización de parámetros
        self.tracking_enabled = self.get_parameter('tracking_enabled').value
        self.slam_enabled = self.get_parameter('slam_enabled').value

        # Estados de la FSM
        self.states = ['INIT', 'IDLE', 'TRACKING', 'SHUTDOWN']
        self.current_state = 'INIT'

        # Estado del sistema
        self.person_detected = False
        self.tracking_service_ready = False

        # Subscripciones
        self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        self.create_subscription(Bool, '/shutdown_confirmation', self.shutdown_confirmation_callback, 10)
        self.velocity_subscription = self.create_subscription(Twist, '/tracking/velocity_cmd', self.velocity_callback, 10)
        if self.slam_enabled:
            self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Publicador de cierre de nodos secundarios
        self.shutdown_publisher = self.create_publisher(Bool, '/system_shutdown', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Cliente para activar/desactivar el seguimiento
        self.tracking_client = self.create_client(SetBool, 'enable_tracking')
        self.wait_for_service(self.tracking_client, 'enable_tracking')

        self.get_logger().info("Nodo de Control iniciado.")
        self.transition_to('IDLE')

    def wait_for_service(self, client, service_name):
        """Espera hasta que el servicio esté disponible."""
        while not client.service_is_ready():
            self.get_logger().info(f"Esperando el servicio {service_name}...")
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info(f"Servicio {service_name} listo.")
        self.tracking_service_ready = True

    def velocity_callback(self, msg):
        """Publica las velocidades recibidas desde el nodo de seguimiento en /cmd_vel."""
        if self.current_state == 'TRACKING':
            self.cmd_vel_publisher.publish(msg)
            self.get_logger().info(f"Velocidad publicada: lineal={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")
        else:
            # Si no está en TRACKING, detén el robot
            self.stop_robot()

    def map_callback(self, msg):
        """Callback para manejar el mapa recibido del nodo SLAM."""
        self.get_logger().info("Mapa recibido del nodo SLAM.")
        # Validar el contenido del mapa antes de proceder
        if not msg.data:
            self.get_logger().warn("Mapa recibido vacío. Esperando un mapa válido.")
            self.transition_to('IDLE')
        else:
            self.get_logger().info("Mapa válido recibido.")

    def stop_robot(self):
        """Detiene el robot publicando un mensaje de velocidad cero."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info("Robot detenido.")

    def transition_to(self, new_state):
        """Gestiona las transiciones de estado de la FSM."""
        if new_state not in self.states:
            self.get_logger().error(f"Estado inválido: {new_state}")
            return

        self.get_logger().info(f"Transición de {self.current_state} a {new_state}")
        self.current_state = new_state

        # Acciones asociadas con el nuevo estado
        if new_state == 'IDLE':
            self.handle_idle()
        elif new_state == 'TRACKING':
            self.start_tracking()
        elif new_state == 'SHUTDOWN':
            self.notify_shutdown()

    def handle_idle(self):
        """Acciones mientras el sistema está en espera."""
        self.get_logger().info("Sistema en modo IDLE. Esperando detecciones...")

    def start_tracking(self):
        """Acciones para iniciar el seguimiento."""
        if self.tracking_service_ready and self.person_detected:
            self.toggle_tracking(True)
            self.get_logger().info("Iniciando seguimiento...")
        else:
            self.get_logger().error("Seguimiento no iniciado. Verifica que el servicio esté listo y haya una detección válida.")
            self.transition_to('IDLE')

    def notify_shutdown(self):
        """Notificar a los nodos secundarios para que se cierren."""
        self.shutdown_publisher.publish(Bool(data=True))
        self.get_logger().info("Notificación de cierre enviada a los nodos secundarios.")
        self.destroy_node()

    def person_detected_callback(self, msg):
        """Callback para manejar detección de personas."""
        self.person_detected = msg.data
        if self.person_detected and self.current_state == 'IDLE':
            self.transition_to('TRACKING')
        elif not self.person_detected and self.current_state == 'TRACKING':
            self.transition_to('IDLE')

    def shutdown_confirmation_callback(self, msg):
        """Callback para manejar confirmación de cierre."""
        if msg.data:
            self.transition_to('SHUTDOWN')

    def toggle_tracking(self, enable):
        """Habilita o deshabilita el seguimiento según el estado actual."""
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
            self.get_logger().info(f"Seguimiento {'habilitado' if response.success else 'deshabilitado'}.")
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
        node.notify_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
