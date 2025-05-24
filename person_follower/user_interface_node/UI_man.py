import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32MultiArray
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from diagnostic_msgs.msg import DiagnosticArray
import subprocess
import threading

class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('user_interface_node')

        # Parámetros de habilitación
        self.declare_parameter('enabled', True)
        self.declare_parameter('visualization_enabled', True)

        self.enabled = self.get_parameter('enabled').value
        self.visualization_enabled = self.get_parameter('visualization_enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de Interfaz de Usuario desactivado.")
            return

        # Inicializar escucha de cierre
        self.initialize_shutdown_listener()

        # Subscribers de estado de otros nodos
        self.create_subscription(String, '/camera/status', self.camera_status_callback, 10)
        self.create_subscription(String, '/detection/status', self.detection_status_callback, 10)
        self.create_subscription(String, '/tracking/status', self.tracking_status_callback, 10)
        self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        self.create_subscription(Float32MultiArray, '/clusters/general', self.general_clusters_callback, 10)
        self.create_subscription(Float32MultiArray, '/clusters/legs', self.leg_clusters_callback, 10)
        self.create_subscription(Point, '/expected_person_position', self.person_position_callback, 10)
        self.create_subscription(String, '/control/state', self.control_state_callback, 10)

        # Publishers RViz y diagnósticos
        self.marker_pub = self.create_publisher(Marker, '/visualization/person_marker', 10)
        self.leg_cluster_pub = self.create_publisher(Marker, '/visualization/leg_clusters', 10)
        self.general_cluster_pub = self.create_publisher(Marker, '/visualization/general_clusters', 10)
        self.robot_marker_pub = self.create_publisher(Marker, '/visualization/robot_marker', 10)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.status_text_pub = self.create_publisher(Marker, '/visualization/status_text', 10)

        # Control manual
        self.ui_command_pub = self.create_publisher(String, '/ui_command', 10)
        self.manual_cmd_pub = self.create_publisher(Twist, '/commands/velocity', 10)
        self.current_control_state = 'INIT'

        # Estado interno
        self.person_detected = False
        self.camera_status = 'Desconocido'
        self.detection_status = 'Desconocido'
        self.tracking_status = 'Desconocido'
        self.previous_status = {}

        # Lanzar RViz
        self.start_rviz()

        # Timer para mostrar estado
        self.timer = self.create_timer(5.0, self.display_status)

        # Hilo para captura de teclado
        self.input_thread = threading.Thread(target=self.listen_for_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info('Nodo de Interfaz de Usuario iniciado.')

    # ------------------- Control Manual -------------------
    def listen_for_keyboard_input(self):
        import sys, tty, termios
        def getch():
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return ch

        self.get_logger().info('Controles: [m] Manual  [a] Automático  [s] Emergencia  [W/S/A/D/ ] Mover')
        while rclpy.ok():
            key = getch().lower()
            if key == 'm':
                self.switch_to_manual()
                self.get_logger().info('Modo manual activado.')
            elif key == 'a':
                self.switch_to_automatic()
                self.get_logger().info('Modo automático activado.')
            elif key == 's':
                self.send_emergency_stop()
                self.get_logger().info('Parada de emergencia enviada.')
            elif self.current_control_state == 'MANUAL':
                twist = Twist()
                if key == 'w':
                    twist.linear.x = 0.2
                elif key == 's':
                    twist.linear.x = -0.2
                elif key == 'a':
                    twist.angular.z = 0.5
                elif key == 'd':
                    twist.angular.z = -0.5
                elif key == ' ':
                    # Espacio: frenar
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                else:
                    continue
                self.manual_cmd_pub.publish(twist)
                self.get_logger().info(f'MANUAL cmd_vel → lin: {twist.linear.x:.2f}, ang: {twist.angular.z:.2f}')

    def send_emergency_stop(self):
        self.ui_command_pub.publish(String(data='emergency_stop'))

    def switch_to_manual(self):
        self.ui_command_pub.publish(String(data='manual_mode'))

    def switch_to_automatic(self):
        self.ui_command_pub.publish(String(data='automatic_mode'))

    # ------------------- Callbacks existentes -------------------
    def control_state_callback(self, msg):
        self.current_control_state = msg.data

    def camera_status_callback(self, msg): self.camera_status = msg.data
    def detection_status_callback(self, msg): self.detection_status = msg.data
    def tracking_status_callback(self, msg): self.tracking_status = msg.data
    def person_detected_callback(self, msg): self.person_detected = msg.data

    def person_position_callback(self, msg: Point):
        if not self.visualization_enabled:
            return
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'person'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = -msg.x
        marker.pose.position.y = -msg.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.4
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        self.marker_pub.publish(marker)

    def leg_clusters_callback(self, msg: Float32MultiArray):
        if not self.visualization_enabled: return
        if (self.get_clock().now() - self.last_cluster_publish_time).nanoseconds * 1e-9 < 0.5:
            return
        self.last_cluster_publish_time = self.get_clock().now()
        if not msg.data or len(msg.data) % 2 != 0:
            self.get_logger().warn('Datos inválidos en clusters de piernas.')
            return
        self.publish_cluster_marker(msg.data, 'legs', 1.0, 0.0, 0.0)

    def general_clusters_callback(self, msg: Float32MultiArray):
        if not self.visualization_enabled: return
        if (self.get_clock().now() - self.last_cluster_publish_time).nanoseconds * 1e-9 < 0.5:
            return
        self.last_cluster_publish_time = self.get_clock().now()
        if not msg.data or len(msg.data) % 2 != 0:
            self.get_logger().warn('Datos inválidos en clusters generales.')
            return
        self.publish_cluster_marker(msg.data, 'general', 0.0, 0.0, 1.0)

    def publish_robot_marker(self):
        if self.robot_marker_published: return
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot'
        marker.id = 100
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        self.robot_marker_pub.publish(marker)
        self.robot_marker_published = True

    def publish_cluster_marker(self, data, ns, r, g, b):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = 0.05
        marker.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        for i in range(0, len(data), 2):
            p = Point()
            p.x = -data[i]
            p.y = -data[i + 1]
            p.z = 0.0
            marker.points.append(p)
        if ns == 'legs':
            self.leg_cluster_pub.publish(marker)
        else:
            self.general_cluster_pub.publish(marker)

    def publish_status_text_marker(self):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'status_text'
        marker.id = 200
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.1
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text_lines = [
            f"{self.camera_status}",
            f"{self.detection_status}",
            f"{self.tracking_status}",
            f"Person detected: {'Yes' if self.person_detected else 'No'}",
            f"Control state: {self.current_control_state}"
        ]
        marker.text = '\n'.join(text_lines)
        self.status_text_pub.publish(marker)

    def publish_legend_hud(self):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'legend_hud'
        marker.id = 400
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 2.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.3
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = (
            'Leyenda:\n'
            '🟡 Robot Base\n'
            '🟢 Persona Detectada\n'
            '🔵 Clusters Generales\n'
            '🔴 Clusters Piernas'
        )
        self.status_text_pub.publish(marker)

    def display_status(self):
        self.publish_robot_marker()
        self.publish_status_text_marker()
        if not hasattr(self, 'legend_published'):
            self.publish_legend_hud()
            self.legend_published = True
        current_status = {
            'Camera': self.camera_status,
            'Detection': self.detection_status,
            'Tracking': self.tracking_status,
            'Person detected': 'Yes' if self.person_detected else 'No',
            'Control state': self.current_control_state
        }
        if current_status != self.previous_status:
            print('\n=== Estado del Sistema ===')
            for k,v in current_status.items(): print(f"{k}: {v}")
            print('==========================')
            self.previous_status = current_status

    def initialize_shutdown_listener(self):
        from std_msgs.msg import Bool
        self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.shutdown_confirmation_publisher = self.create_publisher(Bool, '/shutdown_confirmation', 10)

    def shutdown_callback(self, msg):
        from std_msgs.msg import Bool
        if msg.data:
            self.get_logger().info('Cierre del sistema detectado. Enviando confirmación.')
            self.stop_rviz()
            try:
                self.shutdown_confirmation_publisher.publish(Bool(data=True))
            except Exception as e:
                self.get_logger().error(f"Error al publicar confirmación de apagado: {e}")
            finally:
                self.destroy_node()

    def destroy_node(self):
        self.stop_rviz()
        self.get_logger().info('Nodo de Interfaz de Usuario detenido.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UserInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo Interfaz de Usuario detenido manualmente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
