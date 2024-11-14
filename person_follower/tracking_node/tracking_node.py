# Nodo de Seguimiento: TrackingNode
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
import math

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        
        # Estado del nodo de seguimiento
        self.tracking_enabled = False
        
        # Servicio para habilitar o deshabilitar el seguimiento
        self.create_service(SetBool, 'enable_tracking', self.enable_tracking_callback)

        # Publicadores y suscripciones
        self.status_publisher = self.create_publisher(String, '/tracking/status', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/commands/velocity', 10)
        self.person_detected_subscription = self.create_subscription(Bool, '/person_detected', self.detection_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        
        self.person_detected = False
        self.get_logger().info("Nodo de Seguimiento iniciado")
        self.publish_status("Nodo de Seguimiento iniciado.")

    def enable_tracking_callback(self, request, response):
        self.tracking_enabled = request.data
        response.success = True
        response.message = f"Tracking {'enabled' if self.tracking_enabled else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def detection_callback(self, msg):
        self.person_detected = msg.data

    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))

    def listener_callback(self, input_msg):
        if not self.tracking_enabled or not self.person_detected:
            return

        # Verificar si hay obstáculos en la distancia mínima
        closest_distance = min(input_msg.ranges)
        if closest_distance < 0.4:
            # Detener el robot si el obstáculo está demasiado cerca
            self.stop_robot()
            self.get_logger().warn("Obstáculo detectado muy cerca. Robot detenido.")
            return  # No continuar con el seguimiento si hay un obstáculo

        # Calcular la dirección de la persona y ajustar la velocidad
        angle_min, angle_increment, ranges = input_msg.angle_min, input_msg.angle_increment, input_msg.ranges
        min_range_index = ranges.index(min(ranges))
        angle_to_person = angle_min + min_range_index * angle_increment
        distance_to_person = min(ranges)

        # Ajustar velocidad lineal según la distancia
        if distance_to_person < 0.4:
            vx = 0.0  # Detener si está demasiado cerca de la persona
        elif distance_to_person < 1.0:
            vx = 0.25  # Reducir velocidad
        else:
            vx = 0.45  # Velocidad normal

        angle_difference = -angle_to_person
        max_angular_velocity = 1.6
        wz = 2.0 * angle_difference
        if abs(wz) > max_angular_velocity:
            wz = max_angular_velocity if wz > 0 else -max_angular_velocity

        # Publicar mensaje de velocidad
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = vx
        cmd_vel_msg.angular.z = wz
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
        self.get_logger().info(f"Distancia a la persona: {distance_to_person:.2f} m | linear.x = {vx}, angular.z = {wz}")

    def stop_robot(self):
        control_msg = Twist()
        control_msg.linear.x = 0.0
        control_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(control_msg)
        self.get_logger().info("Robot detenido.")


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
