import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
import numpy as np
from sklearn.cluster import DBSCAN


class DetectionNode(Node):
    def __init__(self, enable_visualization=False):
        super().__init__('detection_node')

        # Declaración de parámetros ajustables
        self.declare_parameter('enabled', True)
        self.declare_parameter('max_detection_distance', 5.0)
        self.declare_parameter('min_detection_distance', 0.3)
        self.declare_parameter('dbscan_eps', 0.08)
        self.declare_parameter('dbscan_min_samples', 15)
        self.declare_parameter('min_leg_cluster_size', 40)
        self.declare_parameter('max_leg_cluster_size', 80)
        self.declare_parameter('min_leg_radius', 0.01)
        self.declare_parameter('max_leg_radius', 0.05)

        # Obtener valores de parámetros
        self.enabled = self.get_parameter('enabled').value
        self.max_detection_distance = self.get_parameter('max_detection_distance').value
        self.min_detection_distance = self.get_parameter('min_detection_distance').value
        self.dbscan_eps = self.get_parameter('dbscan_eps').value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        self.min_leg_cluster_size = self.get_parameter('min_leg_cluster_size').value
        self.max_leg_cluster_size = self.get_parameter('max_leg_cluster_size').value
        self.min_leg_radius = self.get_parameter('min_leg_radius').value
        self.max_leg_radius = self.get_parameter('max_leg_radius').value

        if not self.enabled:
            self.get_logger().info("Nodo desactivado.")
            return

        # Inicialización de suscriptores y publicadores
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.detection_publisher = self.create_publisher(Bool, '/person_detected', 10)
        self.cluster_publisher = self.create_publisher(Float32MultiArray, '/detection/clusters', 10)

        self.get_logger().info("Nodo de Detección iniciado.")
        self.enable_visualization = enable_visualization

    def lidar_callback(self, msg):
        """Procesar datos LIDAR y publicar clusters detectados."""
        self.get_logger().debug("Procesando datos del LIDAR.")

        # Filtrar y preprocesar datos
        points = [
            (r * np.cos(msg.angle_min + i * msg.angle_increment), r * np.sin(msg.angle_min + i * msg.angle_increment))
            for i, r in enumerate(msg.ranges)
            if self.min_detection_distance < r < self.max_detection_distance
        ]

        if not points:
            self.get_logger().info("No se detectaron puntos válidos.")
            self.detection_publisher.publish(Bool(data=False))
            return

        points = np.array(points)

        # Agrupar puntos con DBSCAN
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        labels = clustering.labels_

        # Publicar clusters
        self.publish_clusters(points, labels)

        # Detectar si hay personas
        person_detected = self.detect_person(points, labels)
        self.detection_publisher.publish(Bool(data=person_detected))

        if person_detected:
            self.get_logger().info("Persona detectada.")
        else:
            self.get_logger().info("No se detectaron personas.")

    def detect_person(self, points, labels):
        """Detecta si hay una persona basada en los clusters obtenidos."""
        clusters = [points[labels == label] for label in set(labels) if label != -1]
        legs_detected = self.detect_leg_clusters(clusters)
        return len(legs_detected) >= 2

    def detect_leg_clusters(self, clusters):
        """Detecta clusters que podrían ser piernas."""
        leg_clusters = []
        for cluster in clusters:
            cluster_size = len(cluster)
            if self.min_leg_cluster_size < cluster_size < self.max_leg_cluster_size:
                distances = np.linalg.norm(cluster - np.mean(cluster, axis=0), axis=1)
                mean_radius = np.mean(distances)
                if self.min_leg_radius < mean_radius < self.max_leg_radius:
                    leg_clusters.append(cluster)
        return leg_clusters

    def publish_clusters(self, points, labels):
        """Publica los clusters detectados como Float32MultiArray."""
        cluster_msg = Float32MultiArray()

        for label in set(labels):
            if label == -1:  # Ignorar ruido
                continue
            cluster_points = points[labels == label]
            for point in cluster_points:
                cluster_msg.data.extend([point[0], point[1]])  # x, y de cada punto

        self.cluster_publisher.publish(cluster_msg)
        self.get_logger().info(f"Clusters publicados: {len(set(labels)) - (1 if -1 in labels else 0)}")


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode(enable_visualization=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo de Detección detenido manualmente.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

