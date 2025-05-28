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
        
        # Parámetros configurables
        self.declare_parameter('enabled', True)
        self.declare_parameter('camera_timeout', 1.0)               # segundos antes de invalidar detección visual
        self.declare_parameter('camera_debounce_count', 2)         # detecciones visuales consecutivas mínimas
        # Parámetros LIDAR y clustering
        self.declare_parameter('max_detection_distance', 6.0)
        self.declare_parameter('min_detection_distance', 0.1)
        self.declare_parameter('dbscan_eps', 0.1)
        self.declare_parameter('dbscan_min_samples', 15)
        self.declare_parameter('min_leg_cluster_size', 30)
        self.declare_parameter('max_leg_cluster_size', 80)
        self.declare_parameter('min_leg_radius', 0.01)
        self.declare_parameter('max_leg_radius', 0.05)
        self.declare_parameter('min_leg_distance', 0.04)
        self.declare_parameter('max_leg_distance', 0.3)
        self.declare_parameter('median_filter_window', 7)

        # Carga de parámetros
        self.enabled = self.get_parameter('enabled').value
        self.camera_timeout = self.get_parameter('camera_timeout').value
        self.camera_debounce_count = int(self.get_parameter('camera_debounce_count').value)
        self.max_detection_distance = self.get_parameter('max_detection_distance').value
        self.min_detection_distance = self.get_parameter('min_detection_distance').value
        self.dbscan_eps = self.get_parameter('dbscan_eps').value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        self.min_leg_cluster_size = self.get_parameter('min_leg_cluster_size').value
        self.max_leg_cluster_size = self.get_parameter('max_leg_cluster_size').value
        self.min_leg_radius = self.get_parameter('min_leg_radius').value
        self.max_leg_radius = self.get_parameter('max_leg_radius').value
        self.min_leg_distance = self.get_parameter('min_leg_distance').value
        self.max_leg_distance = self.get_parameter('max_leg_distance').value
        self.median_filter_window = self.get_parameter('median_filter_window').value

        if not self.enabled:
            self.get_logger().info("Nodo de Detección desactivado.")
            self.publish_status("Nodo desactivado.")
            return

        # Publicadores y suscriptores
        self.status_publisher = self.create_publisher(String, '/detection/status', 10)
        self.detection_publisher = self.create_publisher(Bool, '/person_detected', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Bool, '/person_detected_visual', self.visual_detected_callback, 10)
        self.cluster_publisher = self.create_publisher(Float32MultiArray, '/detection/clusters', 10)
        self.general_cluster_publisher = self.create_publisher(Float32MultiArray, '/clusters/general', 10)
        self.leg_cluster_publisher = self.create_publisher(Float32MultiArray, '/clusters/legs', 10)
        self.person_position_publisher = self.create_publisher(Point, '/person_position', 10)

        # Estado de detección visual
        self.visual_detected = False
        self.visual_count = 0
        self.last_visual_time = self.get_clock().now()

        self.publish_status("Nodo OK.")
        self.initialize_shutdown_listener()

    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))

    def log_info(self, message, data=None):
        if data is not None:
            self.get_logger().info(f"{message} | {data}")
        else:
            self.get_logger().debug(message)

    def lidar_callback(self, msg):
        # Filtro de mediana
        ranges_filtered = self.apply_median_filter(msg.ranges, self.median_filter_window)

        # Interpolación para mayor resolución
        interpolated_ranges, interpolated_angles = self.interpolate_lidar_points(
            ranges_filtered, msg.angle_min, msg.angle_max, msg.angle_increment, factor=2
        )

        # Detección con LIDAR
        person_detected = self.detect_person(
            interpolated_ranges,
            interpolated_angles[0],
            interpolated_angles[1] - interpolated_angles[0]
        )

        # Fusión LIDAR + cámara simplificada
        elapsed = (self.get_clock().now() - self.last_visual_time).nanoseconds * 1e-9
        visual_valid = (elapsed < self.camera_timeout) and (self.visual_count >= self.camera_debounce_count)
        final_detection = person_detected or visual_valid

        self.detection_publisher.publish(Bool(data=final_detection))
        if final_detection:
            self.log_info("Persona detectada (fusion LIDAR+cam)", {'lidar': person_detected, 'cam': visual_valid})

    def initialize_shutdown_listener(self):
        self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.shutdown_confirmation_publisher = self.create_publisher(Bool, '/shutdown_confirmation', 10)

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Cierre del sistema detectado. Enviando confirmación.")
            try:
                self.shutdown_confirmation_publisher.publish(Bool(data=True))
            except Exception as e:
                self.get_logger().error(f"Error al publicar confirmación de apagado: {e}")
            self.destroy_node()

    def visual_detected_callback(self, msg):
        # Debounce: contar detecciones consecutivas
        if msg.data:
            self.visual_count += 1
        else:
            self.visual_count = 0
        if msg.data:
            self.last_visual_time = self.get_clock().now()

    def apply_median_filter(self, data, window_size):
        filtered = np.copy(data)
        for i in range(len(data)):
            start = max(0, i - window_size // 2)
            end = min(len(data), i + window_size // 2 + 1)
            filtered[i] = np.median(data[start:end])
        return filtered

    def interpolate_lidar_points(self, ranges, angle_min, angle_max, angle_increment, factor=2):
        original_angles = np.arange(angle_min, angle_max, angle_increment)
        interpolated_angles = np.linspace(angle_min, angle_max, len(ranges) * factor)
        interpolated_ranges = np.interp(interpolated_angles, original_angles, ranges)
        return interpolated_ranges, interpolated_angles

    def detect_person(self, ranges, angle_min, angle_increment):
        points = [
            (r * np.cos(angle_min + i * angle_increment),
             r * np.sin(angle_min + i * angle_increment))
            for i, r in enumerate(ranges)
            if self.min_detection_distance < r < self.max_detection_distance
        ]
        if not points:
            self.log_info("No se detectaron puntos")
            return False

        points = np.array(points)
        labels = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit_predict(points)
        clusters = [points[labels == lbl] for lbl in set(labels) if lbl != -1]

        _, leg_clusters = self.detect_leg_clusters(clusters)
        self.publish_general_clusters(points, labels)
        if leg_clusters:
            all_leg_points = np.concatenate(leg_clusters)
            self.publish_leg_clusters(all_leg_points)

        candidate_positions = []
        used = set()
        for i, ci in enumerate(leg_clusters):
            if i in used: continue
            ci_center = np.mean(ci, axis=0)
            closest, min_dist = None, float('inf')
            for j, cj in enumerate(leg_clusters):
                if i == j or j in used: continue
                cj_center = np.mean(cj, axis=0)
                d = np.linalg.norm(ci_center - cj_center)
                if self.min_leg_distance < d < self.max_leg_distance and d < min_dist:
                    closest, min_dist = j, d
            if closest is not None:
                used.update({i, closest})
                combo = np.vstack((ci, leg_clusters[closest]))
                candidate_positions.append(np.mean(combo, axis=0))

        if candidate_positions:
            selected = min(candidate_positions, key=lambda p: np.linalg.norm(p))
            pt = Point(x=selected[0], y=selected[1], z=0.0)
            self.person_position_publisher.publish(pt)
            self.log_info("Posición de persona publicada", {"x": selected[0], "y": selected[1]})
            return True

        self.log_info("No se encontraron pares de piernas válidos")
        return False

    def detect_leg_clusters(self, clusters):
        leg_clusters, all_clusters = [], []
        for cl in clusters:
            all_clusters.append(cl)
            size = len(cl)
            if self.min_leg_cluster_size < size < self.max_leg_cluster_size:
                xmin, ymin = cl.min(axis=0)
                xmax, ymax = cl.max(axis=0)
                w, h = xmax - xmin, ymax - ymin
                if min(w, h) > 0 and max(w, h) / min(w, h) < 5.0:
                    radii = np.linalg.norm(cl - np.mean(cl, axis=0), axis=1)
                    if self.min_leg_radius < radii.mean() < self.max_leg_radius:
                        leg_clusters.append(cl)
        return all_clusters, leg_clusters

    def publish_general_clusters(self, points, labels):
        msg = Float32MultiArray()
        for lbl in set(labels):
            if lbl == -1: continue
            for x, y in points[labels == lbl]: msg.data.extend([x, y])
        self.general_cluster_publisher.publish(msg)

    def publish_leg_clusters(self, points):
        msg = Float32MultiArray()
        for x, y in points: msg.data.extend([x, y])
        self.leg_cluster_publisher.publish(msg)


def main(args=None):
    rclpy.init()
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("DetectionNode detenido con Ctrl-C.")
    finally:
        node.destroy_node()

