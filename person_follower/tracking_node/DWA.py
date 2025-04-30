import rclpy
import numpy as np
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from nav_msgs.msg import OccupancyGrid


def simulate_trajectory(x, y, theta, v, w, dt=0.1, steps=10):
    trajectory = []
    for _ in range(steps):
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        trajectory.append((x, y))
    return trajectory


def evaluate_trajectory(trajectory, goal):
    last_x, last_y = trajectory[-1]
    goal_dist = math.hypot(goal[0] - last_x, goal[1] - last_y)
    return -goal_dist


def check_collision(trajectory, laser_ranges, angle_min, angle_increment, max_range=5.0, safety_radius=0.3):
    obstacle_points = []
    for i, r in enumerate(laser_ranges):
        if 0.1 < r < max_range:
            angle = angle_min + i * angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            obstacle_points.append((x, y))

    for px, py in trajectory:
        for ox, oy in obstacle_points:
            if math.hypot(px - ox, py - oy) < safety_radius:
                return True
    return False


def compute_dwa_velocity(current_pose, goal, laser_scan):
    x, y, theta = current_pose
    best_score = -float('inf')
    best_v, best_w = 0.0, 0.0

    for v in np.arange(0.0, 0.8, 0.1):
        for w in np.arange(-1.6, 1.6, 0.2):
            traj = simulate_trajectory(x, y, theta, v, w)
            if not check_collision(traj, laser_scan['ranges'], laser_scan['angle_min'], laser_scan['angle_increment']):
                score = evaluate_trajectory(traj, goal)
                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w
    return best_v, best_w


class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Seguimiento desactivado.")
            return

        self.tracking_enabled = False
        self.kalman_state = np.zeros(4)
        self.kalman_covariance = np.eye(4) * 0.1
        self.kalman_F = np.eye(4)
        self.kalman_H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.kalman_R = np.eye(2) * 0.05
        self.kalman_Q = np.eye(4) * 0.01

        self.create_service(SetBool, 'enable_tracking', self.enable_tracking_callback)
        self.declare_parameter('obstacle_avoidance_enabled', True)
        self.obstacle_avoidance_enabled = self.get_parameter('obstacle_avoidance_enabled').value

        self.person_position_subscription = self.create_subscription(Point, '/person_position', self.person_position_callback, 10)
        self.person_detected_subscription = self.create_subscription(Bool, '/person_detected', self.detection_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.shutdown_subscription = self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.status_publisher = self.create_publisher(String, '/tracking/status', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/tracking/velocity_cmd', 10)
        self.position_publisher = self.create_publisher(Point, '/expected_person_position', 10)

        self.person_detected = False
        self.person_position = None
        self.last_person_update_time = None
        self.timeout_duration = 2.0
        self.map_data = None
        self.last_map_data = None

        self.get_logger().info("Nodo de Seguimiento iniciado")

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
        self.get_logger().info(f"Posición estimada (Kalman): x={self.person_position.x:.2f}, y={self.person_position.y:.2f}")

    def map_callback(self, msg):
        current_map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        if self.last_map_data is None or not np.array_equal(current_map_data, self.last_map_data):
            self.map_data = current_map_data
            self.get_logger().info("Mapa de ocupación recibido y procesado.")
            self.last_map_data = current_map_data

    def listener_callback(self, input_msg):
        if not self.tracking_enabled or not self.person_detected or not self.person_position:
            self.stop_robot()
            return

        if self.last_person_update_time is None or (self.get_clock().now() - self.last_person_update_time).nanoseconds * 1e-9 > self.timeout_duration:
            self.get_logger().warn("Tiempo de espera agotado. Deteniendo robot.")
            self.stop_robot()
            return

        current_pose = (0.0, 0.0, 0.0)
        goal = (self.person_position.x, self.person_position.y)
        laser_scan = {
            "ranges": input_msg.ranges,
            "angle_min": input_msg.angle_min,
            "angle_increment": input_msg.angle_increment
        }

        vx, wz = compute_dwa_velocity(current_pose, goal, laser_scan)

        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.angular.z = wz
        self.velocity_publisher.publish(cmd_msg)

    def stop_robot(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.velocity_publisher.publish(cmd_msg)

    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Cierre del sistema detectado. Enviando confirmación.")
            try:
                self.shutdown_confirmation_publisher.publish(Bool(data=True))
            except Exception as e:
                self.get_logger().error(f"Error al publicar confirmación de apagado: {e}")
            finally:
                self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo de Seguimiento detenido manualmente.")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
