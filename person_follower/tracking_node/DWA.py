import rclpy
import numpy as np
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

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

        self.status_publisher = self.create_publisher(String, '/tracking/status', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/tracking/velocity_cmd', 10)
        self.position_publisher = self.create_publisher(Point, '/expected_person_position', 10)

        self.person_detected = False
        self.person_position = None
        self.last_person_update_time = None
        self.timeout_duration = 2.0
        self.previous_vx = 0.0

        self.max_speed = 0.8
        self.min_speed = 0.0
        self.max_yaw_rate = 1.6
        self.max_accel = 0.2
        self.dt = 0.1
        self.predict_time = 1.0

        self.get_logger().info("Nodo de Seguimiento con DWA iniciado")

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

    def calc_dynamic_window(self, vx):
        return [
            max(self.min_speed, vx - self.max_accel * self.dt),
            min(self.max_speed, vx + self.max_accel * self.dt),
            -self.max_yaw_rate,
            self.max_yaw_rate
        ]

    def simulate_trajectory(self, vx, wz):
        x, y, theta = 0.0, 0.0, 0.0
        traj = []
        for _ in range(int(self.predict_time / self.dt)):
            x += vx * math.cos(theta) * self.dt
            y += vx * math.sin(theta) * self.dt
            theta += wz * self.dt
            traj.append((x, y))
        return traj

    def get_obstacles(self, scan):
        angle = scan.angle_min
        obstacles = []
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                obstacles.append((x, y))
            angle += scan.angle_increment
        return obstacles

    def evaluate_trajectory(self, traj, goal, scan):
        goal_dist = math.hypot(goal[0] - traj[-1][0], goal[1] - traj[-1][1])
        obst_dists = [math.hypot(px - x, py - y) for (px, py) in traj for (x, y) in self.get_obstacles(scan)]
        min_obst_dist = min(obst_dists) if obst_dists else 1.0
        return -goal_dist + 0.8 * min_obst_dist

    def listener_callback(self, input_msg):
        if not self.tracking_enabled or not self.person_detected or not self.person_position:
            self.stop_robot()
            return

        if self.last_person_update_time is None or (self.get_clock().now() - self.last_person_update_time).nanoseconds * 1e-9 > self.timeout_duration:
            self.get_logger().warn("Tiempo de espera agotado. Deteniendo robot.")
            self.stop_robot()
            return

        dw = self.calc_dynamic_window(self.previous_vx)
        best_score = -float('inf')
        best_vx, best_wz = 0.0, 0.0
        goal = (self.person_position.x, self.person_position.y)

        for vx in np.linspace(dw[0], dw[1], num=5):
            for wz in np.linspace(dw[2], dw[3], num=5):
                traj = self.simulate_trajectory(vx, wz)
                score = self.evaluate_trajectory(traj, goal, input_msg)
                if score > best_score:
                    best_score = score
                    best_vx, best_wz = vx, wz

        self.previous_vx = best_vx
        cmd_msg = Twist()
        cmd_msg.linear.x = best_vx
        cmd_msg.angular.z = best_wz
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
