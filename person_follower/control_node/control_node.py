import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Declaración de parámetros para cada funcionalidad/nodo
        self.declare_parameter('collision_enabled', True)
        self.declare_parameter('tracking_enabled', True)
        self.declare_parameter('camera_enabled', True)
        self.declare_parameter('ui_enabled', True)
        self.declare_parameter('detection_enabled', True)
        self.declare_parameter('collision_handling_enabled', True)

        # Asignación de los valores de los parámetros
        self.collision_enabled = self.get_parameter('collision_enabled').value
        self.tracking_enabled = self.get_parameter('tracking_enabled').value
        self.camera_enabled = self.get_parameter('camera_enabled').value
        self.ui_enabled = self.get_parameter('ui_enabled').value
        self.detection_enabled = self.get_parameter('detection_enabled').value
        self.collision_handling_enabled = self.get_parameter('collision_handling_enabled').value
        
        # Suscripciones a estados de nodos
        self.create_subscription(String, '/camera/status', self.camera_status_callback, 10)
        self.create_subscription(String, '/collision_handling/status', self.collision_handling_status_callback, 10)
        self.create_subscription(String, '/detection/status', self.detection_status_callback, 10)
        self.create_subscription(String, '/tracking/status', self.tracking_status_callback, 10)

        # Suscripción a detección de persona
        self.person_detected_sub = self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        
        if self.collision_enabled:
            self.collision_sub = self.create_subscription(Bool, '/collision_detected', self.collision_callback, 10)
        
        # Publicador de comandos de control
        self.control_cmd_pub = self.create_publisher(Twist, '/commands/velocity', 10)
        
        # Estados y temporizador
        self.person_detected = False
        self.collision_detected = False
        self.current_state = "stopped"
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Nodo de Control iniciado con todas las funcionalidades posibles.")

    def camera_status_callback(self, msg):
        self.get_logger().info(msg.data)

    def collision_handling_status_callback(self, msg):
        self.get_logger().info(msg.data)

    def detection_status_callback(self, msg):
        self.get_logger().info(msg.data)

    def tracking_status_callback(self, msg):
        self.get_logger().info(msg.data)

    def person_detected_callback(self, msg):
        if self.detection_enabled and self.tracking_enabled:
            self.person_detected = msg.data
            self.evaluate_system_state()

    def collision_callback(self, msg):
        if self.collision_handling_enabled:
            self.collision_detected = msg.data
            self.evaluate_system_state()

    def evaluate_system_state(self):
        if self.collision_detected:
            self.stop_robot()
            self.current_state = "stopped"
            self.get_logger().warn("Deteniendo robot por posible colisión detectada")
        elif self.person_detected and self.tracking_enabled:
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

