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
        
        self.declare_parameter('enabled', False)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Seguimiento desactivado.")
            return
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/commands/velocity', 10)
        self.person_detected_subscription = self.create_subscription(Bool, '/person_detected', self.detection_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        
        self.min_distance = 0.2
        self.intermediate_distance = 1.0
        self.prev_angles = []
        self.angle_filter_window = 5
        self.person_detected = False
        self.collision_active = False  # Inicializa el estado de colisión

        self.get_logger().info("Nodo de Seguimiento iniciado")

    def detection_callback(self, msg):
        self.person_detected = msg.data

    def listener_callback(self, input_msg):
        """Callback de LIDAR para calcular el ángulo hacia la persona detectada y ajustar velocidades"""
        if not self.person_detected or self.collision_active:
            return  # No seguir si no hay persona o si hay colisión

        angle_min, angle_increment, ranges = input_msg.angle_min, input_msg.angle_increment, input_msg.ranges
        min_range_index = ranges.index(min(ranges))
        angle_to_person = angle_min + min_range_index * angle_increment
        distance_to_person = min(ranges)

        if distance_to_person < self.min_distance:
            vx = 0.0  # Detener si está demasiado cerca
        elif distance_to_person < self.intermediate_distance:
            vx = 0.25
        else:
            vx = 0.45

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
    if node.enabled:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

