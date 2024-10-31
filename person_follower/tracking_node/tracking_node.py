# tracking_node/tracking_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.person_detected_subscription = self.create_subscription(Bool, '/person_detected', self.detection_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        
        # Variables de estado
        self.min_distance = 0.4
        self.prev_angles = []
        self.angle_filter_window = 5
        self.person_detected = False
        self.latest_cmd_vel = Twist()  # Últimos comandos de movimiento recibidos del nodo de control
        self.get_logger().info("Nodo de Seguimiento iniciado")

    def detection_callback(self, msg):
        """Callback para actualizar el estado de detección de personas"""
        self.person_detected = msg.data

    def listener_callback(self, input_msg):
        """Callback de LIDAR para calcular el ángulo hacia la persona detectada"""
        if not self.person_detected:
            return

        angle_min, angle_increment, ranges = input_msg.angle_min, input_msg.angle_increment, input_msg.ranges
        min_range_index = ranges.index(min(ranges))
        angle_to_person = angle_min + min_range_index * angle_increment

        if self.prev_angles:
            angle_to_person = self.smooth_angle(angle_to_person - angle_min)

        self.latest_cmd_vel.angular.z = 2.0 * -angle_to_person if abs(angle_to_person) > 0.6 else -0.6

    def cmd_vel_callback(self, msg):
        """Callback que recibe comandos de velocidad desde el Nodo de Control y los publica al robot"""
        self.latest_cmd_vel.linear.x = msg.linear.x
        self.latest_cmd_vel.angular.z = msg.angular.z
        self.get_logger().info(f"Aplicando comando de control: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}")

    def smooth_angle(self, angle):
        angles_rad = [math.radians(a) for a in self.prev_angles]
        angle_rad = math.radians(angle)
        angles_rad.append(angle_rad)
        if len(angles_rad) > self.angle_filter_window:
            angles_rad.pop(0)
        avg_angle_rad = sum(math.atan2(math.sin(a), math.cos(a)) for a in angles_rad) / len(angles_rad)
        return math.degrees(avg_angle_rad)

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
