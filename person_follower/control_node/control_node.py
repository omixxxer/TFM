# Nodo de Control: ControlNode
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Declaración de parámetros para cada funcionalidad/nodo
        self.declare_parameter('tracking_enabled', True)
        self.declare_parameter('camera_enabled', True)
        self.declare_parameter('ui_enabled', True)
        self.declare_parameter('detection_enabled', True)

        # Asignación de los valores de los parámetros
        self.tracking_enabled = self.get_parameter('tracking_enabled').value
        self.camera_enabled = self.get_parameter('camera_enabled').value
        self.ui_enabled = self.get_parameter('ui_enabled').value
        self.detection_enabled = self.get_parameter('detection_enabled').value
        
        # Suscripciones a estados de nodos
        self.create_subscription(String, '/camera/status', self.camera_status_callback, 10)
        self.create_subscription(String, '/detection/status', self.detection_status_callback, 10)
        self.create_subscription(String, '/tracking/status', self.tracking_status_callback, 10)

        # Suscripción a detección de persona
        self.person_detected_sub = self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        
        # Cliente del servicio para habilitar el nodo de seguimiento
        self.tracking_client = self.create_client(SetBool, 'enable_tracking')
        
        self.person_detected = False
        self.get_logger().info("Nodo de Control iniciado con las funcionalidades disponibles.")

    def camera_status_callback(self, msg):
        self.get_logger().info(msg.data)

    def detection_status_callback(self, msg):
        self.get_logger().info(msg.data)

    def tracking_status_callback(self, msg):
        self.get_logger().info(msg.data)

    def person_detected_callback(self, msg):
        if msg.data != self.person_detected:
            self.person_detected = msg.data
            self.toggle_tracking(self.person_detected)

    def toggle_tracking(self, enable):
        if not self.tracking_client.service_is_ready():
            self.get_logger().warn("Servicio de seguimiento no está listo.")
            return
        request = SetBool.Request()
        request.data = enable
        self.tracking_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
