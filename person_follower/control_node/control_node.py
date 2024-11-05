# control_node/control_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('collision_enabled', True)
        self.declare_parameter('tracking_enabled', False)  # Mantener en False para pruebas
        self.declare_parameter('camera_enabled', False)
        self.declare_parameter('ui_enabled', False)

        self.collision_enabled = self.get_parameter('collision_enabled').value
        self.tracking_enabled = self.get_parameter('tracking_enabled').value
        self.camera_enabled = self.get_parameter('camera_enabled').value
        self.ui_enabled = self.get_parameter('ui_enabled').value

        self.person_detected_sub = self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)

        if self.collision_enabled:
            self.collision_sub = self.create_subscription(Bool, '/collision_detected', self.collision_callback, 10)

        self.control_cmd_pub = self.create_publisher(Twist, '/commands/velocity', 10)

        self.person_detected = False
        self.collision_detected = False
        self.current_state = "stopped"

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Nodo de Control iniciado")

    def person_detected_callback(self, msg):
        self.person_detected = msg.data
        self.evaluate_system_state()

    def collision_callback(self, msg):
        self.collision_detected = msg.data
        self.evaluate_system_state()

    def evaluate_system_state(self):
        if self.collision_detected:
            if self.current_state != "stopped":
                self.stop_robot()
                self.current_state = "stopped"
                self.get_logger().warn("Deteniendo robot por colisión detectada")
        elif self.person_detected and self.tracking_enabled:  # Solo activar seguimiento si el nodo está habilitado
            if self.current_state != "following":
                self.current_state = "following"
                self.get_logger().info("Persona detectada, activando seguimiento")
        else:
            if self.current_state != "stopped":
                self.stop_robot()
                self.current_state = "stopped"
                self.get_logger().info("No hay detecciones, robot en espera")

    def timer_callback(self):
        if self.current_state == "following" and not self.collision_detected:
            self.get_logger().info("Persona detectada, supervisando seguimiento...")

    def stop_robot(self):
        control_msg = Twist()
        control_msg.linear.x = 0.0
        control_msg.angular.z = 0.0
        self.control_cmd_pub.publish(control_msg)
        self.get_logger().info("Robot detenido.")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

