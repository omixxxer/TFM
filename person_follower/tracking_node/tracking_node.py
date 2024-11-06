import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Seguimiento desactivado.")
            self.publish_status("Nodo de Seguimiento desactivado.")
            return
        
        self.status_publisher = self.create_publisher(String, '/tracking/status', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/commands/velocity', 10)
        self.person_detected_subscription = self.create_subscription(Bool, '/person_detected', self.detection_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.get_logger().info("Nodo de Seguimiento iniciado")
        self.publish_status("Nodo de Seguimiento iniciado.")
        
        self.min_distance = 0.2
        self.intermediate_distance = 1.0
        self.person_detected = False
        self.collision_active = False

    def detection_callback(self, msg):
        self.person_detected = msg.data
        
    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))

    def listener_callback(self, input_msg):
        """Callback de LIDAR para calcular el ángulo hacia la persona detectada y ajustar velocidades"""
        if not self.person_detected:
            return  # No seguir si no hay persona

        angle_min, angle_increment, ranges = input_msg.angle_min, input_msg.angle_increment, input_msg.ranges
        min_range_index = ranges.index(min(ranges))
        angle_to_person = angle_min + min_range_index * angle_increment
        distance_to_person = min(ranges)

        # Ajustar la velocidad lineal en función de la distancia a la persona
        if distance_to_person < self.min_distance:
            vx = 0.0  # Detener si está demasiado cerca
        elif distance_to_person < self.intermediate_distance:
            vx = 0.25  # Reducir la velocidad
        else:
            vx = 0.45  # Velocidad normal

        angle_difference = -angle_to_person
        max_angular_velocity = 1.6
        wz = 2.0 * angle_difference
        if abs(wz) > max_angular_velocity:
            wz = max_angular_velocity if wz > 0 else -max_angular_velocity

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = vx
        cmd_vel_msg.angular.z = wz
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
        self.get_logger().info(f"Distancia a la persona: {distance_to_person:.2f} m | linear.x = {vx}, angular.z = {wz}")

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    if node.enabled:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

