import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
from sklearn.cluster import DBSCAN
from std_msgs.msg import String
import matplotlib.pyplot as plt

class DetectionNode(Node):
    def __init__(self, enable_visualization=True):
        super().__init__('detection_node')
        
        # Declaración de parámetros ajustables para el nodo de detección
        self.declare_parameter('enabled', True)
        self.declare_parameter('max_detection_distance', 5.0)
        self.declare_parameter('min_detection_distance', 0.3)
        self.declare_parameter('dbscan_eps', 0.08)
        self.declare_parameter('dbscan_min_samples', 15)
        self.declare_parameter('min_leg_cluster_size', 40)
        self.declare_parameter('max_leg_cluster_size', 80)
        self.declare_parameter('min_leg_radius', 0.01)
        self.declare_parameter('max_leg_radius', 0.05)
        self.declare_parameter('min_leg_distance', 0.01)
        self.declare_parameter('max_leg_distance', 0.5)
        self.declare_parameter('median_filter_window', 7)  # Nuevo parámetro para el filtro de mediana

        # Obtener valores de parámetros desde la configuración
        self.enabled = self.get_parameter('enabled').value
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
            self.log_info("Nodo desactivado", {"status": "disabled"})
            self.publish_status("Nodo de Detección desactivado.")
            return

        # Inicialización de suscriptores y publicadores
        self.status_publisher = self.create_publisher(String, '/detection/status', 10)
        self.detection_publisher = self.create_publisher(Bool, '/person_detected', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        self.log_info("Nodo iniciado", {"status": "enabled"})
        self.publish_status("Nodo de Detección iniciado.")
        
        # Inicializar lógica de cierre
        self.initialize_shutdown_listener()
        
        self.enable_visualization = enable_visualization
    
    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))

    def log_info(self, message, data):
        """Método auxiliar para logging estructurado."""
        self.get_logger().info(f"{message} | {data}")

    def lidar_callback(self, msg):
        self.log_info("Procesando datos LIDAR", {"ranges": len(msg.ranges)})
        
        # Preprocesamiento de datos: aplicar filtro de mediana
        ranges_filtered = self.apply_median_filter(msg.ranges, self.median_filter_window)

        # Interpolación de puntos para mejorar la resolución
        interpolated_ranges, interpolated_angles = self.interpolate_lidar_points(
            ranges_filtered, msg.angle_min, msg.angle_max, msg.angle_increment, factor=2
        )

        # Detección de persona usando los datos interpolados
        person_detected = self.detect_person(interpolated_ranges, interpolated_angles[0], interpolated_angles[1] - interpolated_angles[0])
        self.detection_publisher.publish(Bool(data=person_detected))
        if person_detected:
            self.log_info("Persona detectada", {"detection": "successful"})
    
    def initialize_shutdown_listener(self):
        """Inicializa el suscriptor para manejar el cierre del sistema."""
        self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.shutdown_confirmation_publisher = self.create_publisher(Bool, '/shutdown_confirmation', 10)

    def shutdown_callback(self, msg):
        """Callback para manejar la notificación de cierre del sistema."""
        if msg.data:
            self.get_logger().info("Cierre del sistema detectado. Enviando confirmación.")
            try:
                self.shutdown_confirmation_publisher.publish(Bool(data=True))
            except Exception as e:
                self.get_logger().error(f"Error al publicar confirmación de apagado: {e}")
            finally:
                self.destroy_node()   
    

    def apply_median_filter(self, data, window_size):
        """Aplica un filtro de mediana a los datos LIDAR para reducir el ruido."""
        filtered_data = np.copy(data)
        for i in range(len(data)):
            start = max(0, i - window_size // 2)
            end = min(len(data), i + window_size // 2 + 1)
            filtered_data[i] = np.median(data[start:end])
        self.log_info("Filtro de mediana aplicado", {"window_size": window_size})
        return filtered_data

    def interpolate_lidar_points(self, ranges, angle_min, angle_max, angle_increment, factor=2):
        original_angles = np.arange(angle_min, angle_max, angle_increment)
        interpolated_angles = np.linspace(angle_min, angle_max, len(ranges) * factor)
        interpolated_ranges = np.interp(interpolated_angles, original_angles, ranges)
        self.log_info("Interpolación realizada", {"factor": factor})
        return interpolated_ranges, interpolated_angles

    def detect_person(self, ranges, angle_min, angle_increment):
        points = [
            (r * np.cos(angle_min + i * angle_increment), r * np.sin(angle_min + i * angle_increment))
            for i, r in enumerate(ranges)
            if self.min_detection_distance < r < self.max_detection_distance
        ]

        if not points:
            self.log_info("No se detectaron puntos", {"status": "no_points"})
            return False

        points = np.array(points)
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        labels = clustering.labels_
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        self.log_info("Clusters detectados", {"num_clusters": num_clusters})

        if self.enable_visualization:
            self.plot_clusters(points, labels)

        clusters = [points[labels == label] for label in set(labels) if label != -1]
        detected = self.detect_leg_clusters(clusters)

        return detected

    def detect_leg_clusters(self, clusters):
        leg_clusters = []
        for cluster in clusters:
            cluster_size = len(cluster)
            if self.min_leg_cluster_size < cluster_size < self.max_leg_cluster_size:
                x_min, y_min = np.min(cluster, axis=0)
                x_max, y_max = np.max(cluster, axis=0)
                width, height = x_max - x_min, y_max - y_min
                aspect_ratio = max(width, height) / min(width, height) if min(width, height) > 0 else 0

                if aspect_ratio < 5.0:
                    distances = np.linalg.norm(cluster - np.mean(cluster, axis=0), axis=1)
                    mean_radius = np.mean(distances)
                    if self.min_leg_radius < mean_radius < self.max_leg_radius:
                        leg_clusters.append(cluster)
                        self.log_info(
                            "Cluster de pierna detectado",
                            {"cluster_size": cluster_size, "radius": mean_radius, "aspect_ratio": aspect_ratio},
                        )
        if len(leg_clusters) >= 2:
            self.log_info("Piernas detectadas", {"legs_detected": len(leg_clusters)})
            return True
        self.log_info("Piernas no detectadas", {"legs_detected": len(leg_clusters)})
        return False

    def plot_clusters(self, points, labels):
        """Visualiza los clusters detectados en los datos LIDAR."""
        unique_labels = set(labels)
        plt.figure(figsize=(8, 6))

        for label in unique_labels:
            if label == -1:  # Puntos marcados como ruido
                color = 'k'
                cluster_points = points[labels == label]
                plt.plot(cluster_points[:, 0], cluster_points[:, 1], 'o', markerfacecolor=color, markeredgecolor='k', markersize=6)
            else:
                color = plt.cm.jet(float(label) / len(unique_labels))
                cluster_points = points[labels == label]
                plt.plot(cluster_points[:, 0], cluster_points[:, 1], 'o', markerfacecolor=color, markeredgecolor='k', markersize=6)

                # Calcular el tamaño del cluster y el radio medio
                cluster_size = len(cluster_points)
                distances = np.linalg.norm(cluster_points - np.mean(cluster_points, axis=0), axis=1)
                mean_radius = np.mean(distances)

                # Anotar información del cluster en el gráfico
                centroid = np.mean(cluster_points, axis=0)
                plt.text(centroid[0], centroid[1],
                         f'Size: {cluster_size}\nRadius: {mean_radius:.2f}m',
                         fontsize=8, ha='center', color=color)

        plt.title('DBSCAN Clustering Result')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        plt.grid()
        plt.show()


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
