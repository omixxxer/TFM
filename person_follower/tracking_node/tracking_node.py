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
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.person_detected_subscription = self.create_subscription(Bool, '/person_detected', self.detection_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        
        self.min_distance = 0.4
        self.prev_angles = []
        self.angle_filter_window = 5
        self.person_detected = False
        self.get_logger().info("Nodo de Seguimiento iniciado")

    def detection_callback(self, msg):
        self.person_detected = msg.data

    def listener_callback(self, input_msg):
        if not self.person_detected:
            return

        angle_min, angle_increment, ranges = input_msg.angle_min, input_msg.angle_increment, input_msg.ranges
        min_range_index = ranges.index(min(ranges))
        angle_to_person = angle_min + min_range_index * angle_increment

        if self.prev_angles:
            angle_to_person = self.smooth_angle(angle_to_person - angle_min)

        vx = 0.05 if min(ranges) < self.min_distance else 0.35
        wz = 2.0 * -angle_to_person if abs(angle_to_person) > 0.6 else -0.6

        output_msg = Twist()
        output_msg.linear.x = vx
        output_msg.angular.z = wz
        self.cmd_vel_publisher.publish(output_msg)

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
