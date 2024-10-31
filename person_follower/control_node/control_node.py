# control_node/control_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Suscripciones a los temas de estado de otros nodos
        self.person_detected_sub = self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        self.collision_sub = self.create_subscription(Twist, '/cmd_vel', self.collision_callback, 10)
        
        # Publicador de comandos de movimiento para el nodo de seguimiento
        self.control_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables de estado para tomar decisiones
        self.person_detected = False
        self.collision_detected = False

        self.get_logger().info("Nodo de Control iniciado")

    def person_detected_callback(self, msg):
        """Callback para manejar el estado de detección de personas"""
        self.person_detected = msg.data
        self.evaluate_system_state()

    def collision_callback(self, msg):
        """Callback para manejar el estado de detección de colisiones"""
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:  # Suponer que velocidad cero indica colisión
            self.collision_detected = True
        else:
            self.collision_detected = False
        self.evaluate_system_state()

    def evaluate_system_state(self):
        """Lógica de evaluación para iniciar o detener acciones según el estado del sistema"""
        control_msg = Twist()

        if self.collision_detected:
            # Si hay colisión, detener el robot
            control_msg.linear.x = 0.0
            control_msg.angular.z = 0.0
            self.get_logger().warn("Deteniendo robot por colisión detectada")

        elif self.person_detected:
            # Si se detecta una persona, activar seguimiento
            control_msg.linear.x = 0.2  # Ejemplo de velocidad hacia adelante
            control_msg.angular.z = 0.0
            self.get_logger().info("Persona detectada, activando seguimiento")

        else:
            # Si no hay personas ni colisiones, detener el robot
            control_msg.linear.x = 0.0
            control_msg.angular.z = 0.0
            self.get_logger().info("No hay detecciones, robot en espera")

        # Publicar el comando de control al nodo de seguimiento
        self.control_cmd_pub.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
