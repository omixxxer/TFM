import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')

        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Seguimiento desactivado.")
            return

        self.tracking_enabled = False
        self.person_detected = False
        self.person_position = None
        self.last_person_update_time = None
        self.timeout_duration = 2.0

        self.velocity_publisher = self.create_publisher(Twist, '/tracking/velocity_cmd', 10)
        self.position_publisher = self.create_publisher(Point, '/expected_person_position', 10)
        self.status_publisher = self.create_publisher(String, '/tracking/status', 10)
        self.trajectory_publisher = self.create_publisher(MarkerArray, '/dwa/trajectories', 10)

        self.create_subscription(Point, '/person_position', self.person_position_callback, 10)
        self.create_subscription(Bool, '/person_detected', self.detection_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)

        self.create_service(SetBool, 'enable_tracking', self.enable_tracking_callback)

        self.kalman_state = np.zeros(4)
        self.kalman_covariance = np.eye(4) * 0.1
        self.kalman_F = np.eye(4)
        self.kalman_H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.kalman_R = np.eye(2) * 0.05
        self.kalman_Q = np.eye(4) * 0.01

        self.max_speed = 0.6
        self.max_angular_speed = 2.0
        self.predict_time = 1.0

        self.get_logger().info("Nodo de Seguimiento con DWA y visualización iniciado.")

    def enable_tracking_callback(self, request, response):
        self.tracking_enabled = request.data
        response.success = True
        response.message = f"Tracking {'enabled' if self.tracking_enabled else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def detection_callback(self, msg):
        self.person_detected = msg.data

    def person_position_callback(self, msg):
        z = np.array([msg.x, msg.y])
        self.kalman_F[:2, 2:] = np.eye(2) * 0.1
        predicted_state = self.kalman_F @ self.kalman_state
        predicted_covariance = self.kalman_F @ self.kalman_covariance @ self.kalman_F.T + self.kalman_Q

        y = z - (self.kalman_H @ predicted_state)
        S = self.kalman_H @ predicted_covariance @ self.kalman_H.T + self.kalman_R
        K = predicted_covariance @ self.kalman_H.T @ np.linalg.inv(S)

        self.kalman_state = predicted_state + K @ y
        self.kalman_covariance = (np.eye(4) - K @ self.kalman_H) @ predicted_covariance

        self.person_position = Point(x=self.kalman_state[0], y=self.kalman_state[1])
        self.last_person_update_time = self.get_clock().now()

        self.position_publisher.publish(self.person_position)
        self.get_logger().info(f"Posición estimada: x={self.person_position.x:.2f}, y={self.person_position.y:.2f}")

    def listener_callback(self, scan_data):
        if not self.tracking_enabled or not self.person_detected or not self.person_position:
            self.stop_robot()
            return

        if self.last_person_update_time is None or (self.get_clock().now() - self.last_person_update_time).nanoseconds * 1e-9 > self.timeout_duration:
            self.get_logger().warn("Timeout posición persona. Deteniendo robot.")
            self.stop_robot()
            return

        vx, wz, markers = self.dynamic_window_approach(scan_data)
        self.trajectory_publisher.publish(markers)

        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.angular.z = wz
        self.velocity_publisher.publish(cmd_msg)

        self.get_logger().info(f"Comando: v={vx:.2f} m/s, w={wz:.2f} rad/s")

    def dynamic_window_approach(self, scan_data):
        best_score = -float('inf')
        best_v, best_w = 0.0, 0.0
        markers = MarkerArray()
        marker_id = 0

        v_samples = np.linspace(0, self.max_speed, num=7)
        w_samples = np.linspace(-self.max_angular_speed, self.max_angular_speed, num=5)

        for v in v_samples:
            for w in w_samples:
                heading_score = self.heading(v, w)
                clearance_score = self.clearance(v, w, scan_data)
                velocity_score = v / self.max_speed

                total_score = 0.5 * heading_score + 0.4 * clearance_score + 0.1 * velocity_score

                marker = Marker()
                marker.header.frame_id = "base_footprint"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "dwa"
                marker.id = marker_id
                marker.type = Marker.ARROW
                marker.scale.x = 0.05
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = 1.0 - total_score
                marker.color.g = total_score
                marker.color.b = 0.0
                marker.color.a = 0.8

                end_x = v * self.predict_time * math.cos(w * self.predict_time)
                end_y = v * self.predict_time * math.sin(w * self.predict_time)
                marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=end_x, y=end_y, z=0.0)]

                markers.markers.append(marker)
                marker_id += 1

                if total_score > best_score:
                    best_score = total_score
                    best_v = v
                    best_w = w

        # Mejor trayectoria destacada en azul
        best_marker = Marker()
        best_marker.header.frame_id = "base_footprint"
        best_marker.header.stamp = self.get_clock().now().to_msg()
        best_marker.ns = "dwa_best"
        best_marker.id = marker_id
        best_marker.type = Marker.ARROW
        best_marker.scale.x = 0.1
        best_marker.scale.y = 0.2
        best_marker.scale.z = 0.2
        best_marker.color.r = 0.0
        best_marker.color.g = 0.0
        best_marker.color.b = 1.0
        best_marker.color.a = 1.0

        end_x = best_v * self.predict_time * math.cos(best_w * self.predict_time)
        end_y = best_v * self.predict_time * math.sin(best_w * self.predict_time)
        best_marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=end_x, y=end_y, z=0.0)]

        markers.markers.append(best_marker)

        return best_v, best_w, markers

    def heading(self, v, w):
        px = v * math.cos(w * self.predict_time)
        py = v * math.sin(w * self.predict_time)
        dx = self.person_position.x - px
        dy = self.person_position.y - py
        angle_error = math.atan2(dy, dx)
        return math.cos(angle_error)

    def clearance(self, v, w, scan_data):
        min_dist = float('inf')
        for i, r in enumerate(scan_data.ranges):
            if scan_data.range_min < r < scan_data.range_max:
                angle = scan_data.angle_min + i * scan_data.angle_increment
                pred_x = v * math.cos(angle + w * self.predict_time)
                pred_y = v * math.sin(angle + w * self.predict_time)
                distance = math.hypot(pred_x, pred_y)
                if distance < min_dist:
                    min_dist = distance
        return min(1.0, min_dist)

    def stop_robot(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.velocity_publisher.publish(cmd_msg)
        self.get_logger().info("Robot detenido.")

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Apagando nodo.")
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido manualmente.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
