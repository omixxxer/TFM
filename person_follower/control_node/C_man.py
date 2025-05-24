import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Parámetros configurables
        self.declare_parameter('enabled', True)
        self.declare_parameter('slam_enabled', True)
        self.declare_parameter('camera_enabled', True)

        self.enabled = self.get_parameter('enabled').value
        self.slam_enabled = self.get_parameter('slam_enabled').value
        camera_enabled = self.get_parameter('camera_enabled').value
        self.ignore_gesture = not camera_enabled

        if not self.enabled:
            self.get_logger().info('Nodo de Control desactivado.')
            return
        if self.ignore_gesture:
            self.get_logger().info('Cámara deshabilitada: ignorando gestos.')

        # Estados de la FSM
        self.states = ['INIT', 'IDLE', 'TRACKING', 'MANUAL', 'SHUTDOWN']
        self.current_state = 'INIT'

        # Estado del sistema
        self.person_detected = False
        self.user_authorized = self.ignore_gesture
        self.tracking_service_ready = False

        # Subscripciones
        self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        self.create_subscription(Bool, '/shutdown_confirmation', self.shutdown_confirmation_callback, 10)
        self.create_subscription(Twist, '/tracking/velocity_cmd', self.velocity_callback, 10)
        self.create_subscription(String, '/gesture_command', self.gesture_command_callback, 10)
        self.create_subscription(String, '/ui_command', self.ui_command_callback, 10)

        # Publicadores
        self.shutdown_publisher = self.create_publisher(Bool, '/system_shutdown', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, '/control/state', 10)

        # Cliente de servicio
        self.tracking_client = self.create_client(SetBool, 'enable_tracking')
        self.wait_for_service(self.tracking_client, 'enable_tracking')

        # Iniciar
        self.get_logger().info('Nodo de Control iniciado.')
        self.transition_to('IDLE')

    def wait_for_service(self, client, service_name):
        while not client.service_is_ready():
            self.get_logger().info(f'Esperando el servicio {service_name}...')
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info(f'Servicio {service_name} listo.')
        self.tracking_service_ready = True

    def velocity_callback(self, msg):
        if self.current_state == 'TRACKING':
            self.cmd_vel_publisher.publish(msg)
            self.get_logger().info(f'Velocidad publicada: lin={msg.linear.x:.2f}, ang={msg.angular.z:.2f}')
        elif self.current_state == 'MANUAL':
            # En modo manual, UI publica en /commands/velocity, no actuamos
            pass
        else:
            self.stop_robot()

    def gesture_command_callback(self, msg):
        if self.ignore_gesture:
            return
        gesture = msg.data.lower().strip()
        self.get_logger().info(f'Gesto: {gesture}')
        if gesture == 'start_tracking':
            self.user_authorized = True
            if self.person_detected and self.current_state == 'IDLE':
                self.transition_to('TRACKING')
        elif gesture == 'stop_tracking':
            self.user_authorized = False
            if self.current_state == 'TRACKING':
                self.transition_to('IDLE')

    def ui_command_callback(self, msg):
        cmd = msg.data.lower().strip()
        self.get_logger().info(f'Comando UI: {cmd}')
        if cmd == 'emergency_stop':
            self.stop_robot()
            self.transition_to('IDLE')
        elif cmd == 'manual_mode':
            self.transition_to('MANUAL')
        elif cmd == 'automatic_mode':
            # Volver al modo automático; si persona detectada y autorizado, ir a TRACKING
            if self.person_detected and self.user_authorized:
                self.transition_to('TRACKING')
            else:
                self.transition_to('IDLE')

    def stop_robot(self):
        stop = Twist()
        self.cmd_vel_publisher.publish(stop)
        self.get_logger().info('Robot detenido.')

    def transition_to(self, new_state):
        if new_state not in self.states:
            self.get_logger().error(f'Estado inválido: {new_state}')
            return
        self.get_logger().info(f'{self.current_state} -> {new_state}')
        self.current_state = new_state
        # Publicar estado
        self.state_publisher.publish(String(data=new_state))
        # Acciones por estado
        if new_state == 'IDLE':
            self.handle_idle()
        elif new_state == 'TRACKING':
            self.start_tracking()
        elif new_state == 'MANUAL':
            self.handle_manual()
        elif new_state == 'SHUTDOWN':
            self.notify_shutdown()

    def handle_idle(self):
        self.get_logger().info('Modo IDLE: esperando detecciones...')

    def start_tracking(self):
        if self.tracking_service_ready:
            self.toggle_tracking(True)
            self.get_logger().info('Iniciando seguimiento...')
        else:
            self.get_logger().error('Servicio tracking no listo. Volviendo a IDLE.')
            self.transition_to('IDLE')

    def handle_manual(self):
        # En MANUAL simplemente esperamos comandos UI -> /commands/velocity
        self.get_logger().info('Modo MANUAL: esperando comandos de UI...')

    def notify_shutdown(self):
        self.shutdown_publisher.publish(Bool(data=True))
        self.get_logger().info('Notificación de shutdown enviada.')
        self.destroy_node()

    def person_detected_callback(self, msg):
        self.person_detected = msg.data
        if self.person_detected and self.user_authorized and self.current_state == 'IDLE':
            self.transition_to('TRACKING')
        elif not self.person_detected and self.current_state in ['TRACKING']:
            self.transition_to('IDLE')

    def shutdown_confirmation_callback(self, msg):
        if msg.data:
            self.transition_to('SHUTDOWN')

    def toggle_tracking(self, enable):
        if not self.tracking_service_ready:
            self.get_logger().warn('Servicio tracking no listo.')
            return
        req = SetBool.Request()
        req.data = enable
        fut = self.tracking_client.call_async(req)
        fut.add_done_callback(self.handle_tracking_response)

    def handle_tracking_response(self, future):
        try:
            res = future.result()
            status = 'habilitado' if res.success else 'deshabilitado'
            self.get_logger().info(f'Tracking {status}.')
        except Exception as e:
            self.get_logger().error(f'Error servicio tracking: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ControlNode detenido.')
    finally:
        node.notify_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
