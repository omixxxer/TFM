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
        self.declare_parameter('max_detection_distance', 2.5)
        self.declare_parameter('min_detection_distance', 0.01)
        self.declare_parameter('dbscan_eps', 0.1)
        self.declare_parameter('dbscan_min_samples', 10)
        self.declare_parameter('min_leg_cluster_size', 15)
        self.declare_parameter('max_leg_cluster_size', 45)
        self.declare_parameter('min_leg_radius', 0.01)
        self.declare_parameter('max_leg_radius', 0.15)
        self.declare_parameter('min_leg_distance', 0.1)
        self.declare_parameter('max_leg_distance', 0.4)
        self.declare_parameter('median_filter_window', 5)  # Nuevo parámetro para el filtro de mediana

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
            self.get_logger().info("Nodo de Detección desactivado.")
            self.publish_status("Nodo de Detección desactivado.")
            return

        # Inicialización de suscriptores y publicadores
        self.status_publisher = self.create_publisher(String, '/detection/status', 10)
        self.detection_publisher = self.create_publisher(Bool, '/person_detected', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.get_logger().info("Nodo de Detección iniciado")
        self.publish_status("Nodo de Detección iniciado.")
        
        self.enable_visualization = enable_visualization

    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))

    def lidar_callback(self, msg):
        # Preprocesamiento de datos: aplicar filtro de mediana
        ranges_filtered = self.apply_median_filter(msg.ranges, self.median_filter_window)

        # Detección de persona usando los datos filtrados
        person_detected = self.detect_person(ranges_filtered, msg.angle_min, msg.angle_increment)
        self.detection_publisher.publish(Bool(data=person_detected))
        if person_detected:
            self.get_logger().info("¡Persona detectada!")

    def apply_median_filter(self, data, window_size):
        """Aplica un filtro de mediana a los datos LIDAR para reducir el ruido."""
        data = np.array(data)
        filtered_data = np.copy(data)
        
        for i in range(len(data)):
            # Determina la ventana alrededor del punto actual
            start = max(0, i - window_size // 2)
            end = min(len(data), i + window_size // 2 + 1)
            filtered_data[i] = np.median(data[start:end])
        
        return filtered_data

    def detect_person(self, ranges, angle_min, angle_increment):
        points = []
        
        # Convertir los datos del LIDAR a coordenadas cartesianas y filtrar por distancia
        for i, r in enumerate(ranges):
            if self.min_detection_distance < r < self.max_detection_distance:
                angle = angle_min + i * angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y))

        if not points:
            self.get_logger().info("No se detectaron puntos en el rango especificado.")
            return False

        points = np.array(points)

        # Agrupamiento de puntos con DBSCAN
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        labels = clustering.labels_

        # Log del número de clusters detectados
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        self.get_logger().info(f"Clusters detectados: {num_clusters}")

        # Visualización opcional de clusters
        if self.enable_visualization:
            self.plot_clusters(points, labels)

        # Filtrar clusters para detectar pares de semicírculos que simulen piernas
        clusters = [points[labels == label] for label in set(labels) if label != -1]
        detected = self.detect_leg_clusters(clusters)

        return detected

    def detect_leg_clusters(self, clusters):
        # Filtrar clusters basados en tamaño y forma para identificar "piernas"
        leg_clusters = []
        for cluster in clusters:
            cluster_size = len(cluster)
            if self.min_leg_cluster_size < cluster_size < self.max_leg_cluster_size:
                distances = np.linalg.norm(cluster - np.mean(cluster, axis=0), axis=1)
                mean_radius = np.mean(distances)
                if self.min_leg_radius < mean_radius < self.max_leg_radius:
                    leg_clusters.append(cluster)
                    self.get_logger().info(f"Cluster de pierna detectado con tamaño: {cluster_size} y radio promedio: {mean_radius:.2f} m")
        
        # Verificar si hay al menos dos clusters en posiciones razonables para representar piernas
        if len(leg_clusters) >= 2:
            for i, cluster_a in enumerate(leg_clusters):
                for j, cluster_b in enumerate(leg_clusters):
                    if i < j:
                        distance_between_clusters = np.linalg.norm(np.mean(cluster_a, axis=0) - np.mean(cluster_b, axis=0))
                        self.get_logger().info(f"Distancia entre clusters de pierna: {distance_between_clusters:.2f} m")
                        if self.min_leg_distance < distance_between_clusters < self.max_leg_distance:
                            self.get_logger().info("¡Pareja de clusters de pierna detectada!")
                            return True  # Persona detectada
        self.get_logger().info("No se detectaron parejas de clusters de pierna.")
        return False

    def plot_clusters(self, points, labels):
        unique_labels = set(labels)
        plt.figure(figsize=(8, 6))

        for label in unique_labels:
            if label == -1:
                color = 'k'  # Color para el ruido
                xy = points[labels == label]
                plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=color, markeredgecolor='k', markersize=6)
            else:
                color = plt.cm.jet(float(label) / len(unique_labels))
                cluster_points = points[labels == label]
                plt.plot(cluster_points[:, 0], cluster_points[:, 1], 'o', markerfacecolor=color, markeredgecolor='k', markersize=6)

                # Cálculo de tamaño y radio del cluster
                cluster_size = len(cluster_points)
                distances = np.linalg.norm(cluster_points - np.mean(cluster_points, axis=0), axis=1)
                mean_radius = np.mean(distances)

                # Anotación del tamaño y radio en el gráfico
                centroid = np.mean(cluster_points, axis=0)
                plt.text(centroid[0], centroid[1], f'Size: {cluster_size}\nRadius: {mean_radius:.2f}m',
                         fontsize=8, ha='center', color=color)

        plt.title('DBSCAN Clustering Result with Size and Radius Annotations')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        plt.grid()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode(enable_visualization=True)
    if node.enabled:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
