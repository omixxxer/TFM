import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String
from geometry_msgs.msg import Point
import numpy as np
from sklearn.cluster import DBSCAN
import time


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        
        # Comprobación de nodo habilitado
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de Detección desactivado.")
            return

        # Declaración de parámetros ajustables
        self.declare_parameter('max_detection_distance', 5.0)
        self.declare_parameter('min_detection_distance', 0.1)
        self.declare_parameter('dbscan_eps', 0.1)
        self.declare_parameter('dbscan_min_samples', 15)
        self.declare_parameter('min_leg_cluster_size', 30)
        self.declare_parameter('max_leg_cluster_size', 80)
        self.declare_parameter('min_leg_radius', 0.01)
        self.declare_parameter('max_leg_radius', 0.05)
        self.declare_parameter('median_filter_window', 7)

        # Obtener valores de parámetros
        self.max_detection_distance = self.get_parameter('max_detection_distance').value
        self.min_detection_distance = self.get_parameter('min_detection_distance').value
        self.dbscan_eps = self.get_parameter('dbscan_eps').value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        self.min_leg_cluster_size = self.get_parameter('min_leg_cluster_size').value
        self.max_leg_cluster_size = self.get_parameter('max_leg_cluster_size').value
        self.min_leg_radius = self.get_parameter('min_leg_radius').value
        self.max_leg_radius = self.get_parameter('max_leg_radius').value
        self.median_filter_window = self.get_parameter('median_filter_window').value

        # Inicialización de suscriptores y publicadores
        self.status_publisher = self.create_publisher(String, '/detection/status', 10)
        self.detection_publisher = self.create_publisher(Bool, '/person_detected', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.person_position_publisher = self.create_publisher(Point, '/person_position', 10)
        self.leg_cluster_publisher = self.create_publisher(Float32MultiArray, '/clusters/legs', 10)

        # Variables para identificación de personas
        self.person_id_counter = 1
        self.detected_people = {}  # Clave: ID de persona, Valor: {posición, timestamp, Kalman}

        self.publish_status("Nodo de Detección iniciado.")

    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))

    def lidar_callback(self, msg):
        ranges_filtered = self.apply_median_filter(msg.ranges, self.median_filter_window)

        interpolated_ranges, interpolated_angles = self.interpolate_lidar_points(
            ranges_filtered, msg.angle_min, msg.angle_max, msg.angle_increment, factor=2
        )

        person_detected = self.detect_person(interpolated_ranges, interpolated_angles[0], interpolated_angles[1] - interpolated_angles[0])

        self.detection_publisher.publish(Bool(data=person_detected))
        if person_detected:
            self.publish_status("Persona detectada.")

        self.cleanup_lost_people(max_inactive_time=10)

    def apply_median_filter(self, data, window_size):
        filtered_data = np.copy(data)
        for i in range(len(data)):
            start = max(0, i - window_size // 2)
            end = min(len(data), i + window_size // 2 + 1)
            filtered_data[i] = np.median(data[start:end])
        return filtered_data

    def interpolate_lidar_points(self, ranges, angle_min, angle_max, angle_increment, factor=2):
        original_angles = np.arange(angle_min, angle_max, angle_increment)
        interpolated_angles = np.linspace(angle_min, angle_max, len(ranges) * factor)
        interpolated_ranges = np.interp(interpolated_angles, original_angles, ranges)
        return interpolated_ranges, interpolated_angles

    def initialize_kalman(self):
        return {
            "state": np.zeros(4),  # [x, y, vx, vy]
            "covariance": np.eye(4) * 0.1,
            "F": np.eye(4),  # Matriz de transición
            "H": np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]]),  # Matriz de observación
            "R": np.eye(2) * 0.05,  # Ruido de observación
            "Q": np.eye(4) * 0.01,  # Ruido del proceso
            "timestamp": time.time()
        }

    def detect_person(self, ranges, angle_min, angle_increment):
        points = [
            (r * np.cos(angle_min + i * angle_increment), r * np.sin(angle_min + i * angle_increment))
            for i, r in enumerate(ranges)
            if self.min_detection_distance < r < self.max_detection_distance
        ]

        if not points:
            return False

        points = np.array(points)
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        labels = clustering.labels_
        clusters = [points[labels == label] for label in set(labels) if label != -1]

        leg_clusters = self.detect_leg_clusters(clusters)

        if leg_clusters:
            position = np.mean(np.concatenate(leg_clusters), axis=0)
            person_id = self.get_person_id(position)

            person_position = Point(x=position[0], y=position[1], z=float(person_id))
            self.person_position_publisher.publish(person_position)

        return bool(leg_clusters)

    def detect_leg_clusters(self, clusters):
        leg_clusters = []
        for cluster in clusters:
            cluster_size = len(cluster)
            if self.min_leg_cluster_size < cluster_size < self.max_leg_cluster_size:
                mean_radius = np.mean(np.linalg.norm(cluster - np.mean(cluster, axis=0), axis=1))
                if self.min_leg_radius < mean_radius < self.max_leg_radius:
                    leg_clusters.append(cluster)
        return leg_clusters

    def get_person_id(self, position):
        current_time = time.time()
        for person_id, data in self.detected_people.items():
            if np.linalg.norm(position - data["position"]) < 0.5 and (current_time - data["timestamp"]) < 5:
                self.detected_people[person_id]["position"] = position
                self.detected_people[person_id]["timestamp"] = current_time
                return person_id

        new_id = self.person_id_counter
        self.person_id_counter += 1
        self.detected_people[new_id] = {"position": position, "timestamp": current_time}
        return new_id

    def cleanup_lost_people(self, max_inactive_time=10):
        current_time = time.time()
        self.detected_people = {id: data for id, data in self.detected_people.items() if current_time - data["timestamp"] < max_inactive_time}


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
