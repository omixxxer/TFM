import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
import numpy as np
from sklearn.cluster import DBSCAN
from std_msgs.msg import String
from geometry_msgs.msg import Point
import time


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        
        #Comprobación de nodo habilitado
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de Detección desactivado.")
            return

        # Declaración de parámetros ajustables para el nodo de detección
        self.declare_parameter('max_detection_distance', 5.0)
        self.declare_parameter('min_detection_distance', 0.1)
        self.declare_parameter('dbscan_eps', 0.1)
        self.declare_parameter('dbscan_min_samples', 15)
        self.declare_parameter('min_leg_cluster_size', 30)
        self.declare_parameter('max_leg_cluster_size', 80)
        self.declare_parameter('min_leg_radius', 0.01)
        self.declare_parameter('max_leg_radius', 0.05)
        self.declare_parameter('min_leg_distance', 0.02)
        self.declare_parameter('max_leg_distance', 0.4)
        self.declare_parameter('median_filter_window', 7)  # Nuevo parámetro para el filtro de mediana

        # Obtener valores de parámetros desde la configuración
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


        # Inicialización de suscriptores y publicadores
        self.status_publisher = self.create_publisher(String, '/detection/status', 10)
        self.detection_publisher = self.create_publisher(Bool, '/person_detected', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Bool, '/person_detected_visual', self.visual_detected_callback, 10)
        self.cluster_publisher = self.create_publisher(Float32MultiArray, '/detection/clusters', 10)
        self.general_cluster_publisher = self.create_publisher(Float32MultiArray, '/clusters/general', 10)
        self.leg_cluster_publisher = self.create_publisher(Float32MultiArray, '/clusters/legs', 10)
        self.person_position_publisher = self.create_publisher(Point, '/person_position', 10)

        # Estado de la verificación visual
        self.visual_detected = False
        self.visual_detection_confirmed = False
        self.visual_detection_timeout = 3.0  # segundos
        self.last_visual_detected_time = self.get_clock().now()

        # Variable para asignar IDs a las personas detectadas
        self.person_id_counter = 0
        self.last_seen_positions = {}
        self.cleanup_lost_people(max_inactive_time=10)

        self.publish_status("Nodo de Detección iniciado.")
        
        # Inicializar lógica de cierre
        self.initialize_shutdown_listener()
        
    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))    


    def log_info(self, message, data):
        """Método auxiliar para logging estructurado."""
        self.get_logger().info(f"{message} | {data}")

    def lidar_callback(self, msg):
        # Preprocesamiento de datos: aplicar filtro de mediana
        ranges_filtered = self.apply_median_filter(msg.ranges, self.median_filter_window)

        # Interpolación de puntos para mejorar la resolución
        interpolated_ranges, interpolated_angles = self.interpolate_lidar_points(
            ranges_filtered, msg.angle_min, msg.angle_max, msg.angle_increment, factor=2
        )

        # Detección de persona con LIDAR
        person_detected = self.detect_person(
            interpolated_ranges,
            interpolated_angles[0],
            interpolated_angles[1] - interpolated_angles[0]
        )

        # Comprobación combinada con detección visual
        if not self.visual_detection_confirmed:
            # No se ha confirmado detección visual aún: bloquear salida
            final_detection = False
        else:
            # Visual ya fue confirmada alguna vez
            time_since_visual = (self.get_clock().now() - self.last_visual_detected_time).nanoseconds * 1e-9
            visual_valid = self.visual_detected or (time_since_visual < self.visual_detection_timeout)
            final_detection = person_detected or visual_valid

        # Publicar resultado combinado
        self.detection_publisher.publish(Bool(data=final_detection))

        if final_detection:
            self.log_info("Persona detectada (combinada)", {
                "lidar_ok": person_detected,
                "visual_ok": self.visual_detected
            })

        # Limpieza de personas no activas
        self.cleanup_lost_people(max_inactive_time=10)
    
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
    
    def visual_detected_callback(self, msg):
        self.visual_detected = msg.data
        if msg.data:
            self.visual_detection_confirmed = True
            self.last_visual_detected_time = self.get_clock().now()

    def apply_median_filter(self, data, window_size):
        """Aplica un filtro de mediana a los datos LIDAR para reducir el ruido."""
        filtered_data = np.copy(data)
        for i in range(len(data)):
            start = max(0, i - window_size // 2)
            end = min(len(data), i + window_size // 2 + 1)
            filtered_data[i] = np.median(data[start:end])
        #self.log_info("Filtro de mediana aplicado", {"window_size": window_size})
        return filtered_data

    def interpolate_lidar_points(self, ranges, angle_min, angle_max, angle_increment, factor=2):
        original_angles = np.arange(angle_min, angle_max, angle_increment)
        interpolated_angles = np.linspace(angle_min, angle_max, len(ranges) * factor)
        interpolated_ranges = np.interp(interpolated_angles, original_angles, ranges)
        #self.log_info("Interpolación realizada", {"factor": factor})
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
    
        # Agrupa los puntos según las etiquetas de DBSCAN
        clusters = [points[labels == label] for label in set(labels) if label != -1]
    
        # Detectar los clusters de piernas y obtener todos los clusters
        all_clusters, leg_clusters = self.detect_leg_clusters(clusters)
    
        # Publicar clusters generales
        self.publish_general_clusters(points, labels)

        if len(leg_clusters) >=2:
            position = np.mean(np.concatenate(leg_clusters), axis=0)
            person_id = self.get_person_id(position)
            person_position = Point(x=position[0], y=position[1], z=float(person_id))
            self.person_position_publisher.publish(person_position)

            # Publicar clusters de piernas
            leg_points = np.concatenate(leg_clusters)
            self.publish_leg_clusters(leg_points)

            self.log_info(
                f"Posición de la persona {person_id} publicada",
                {"x": position[0], "y": position[1]}
            )
    
        return bool(leg_clusters)
    
    
    def detect_leg_clusters(self, clusters):
        """Detecta los clusters de piernas y devuelve ambos: los clusters generales y los de piernas."""
        leg_clusters = []
        all_clusters = []
    
        for cluster in clusters:
            all_clusters.append(cluster)
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
                        # self.log_info(
                            # "Cluster de pierna detectado",
                            # {"cluster_size": cluster_size, "radius": mean_radius, "aspect_ratio": aspect_ratio},
                        # )
                    
        return all_clusters, leg_clusters

    
    def get_person_id(self, position):
        """Obtiene el ID de la persona basándose en la posición y el tiempo"""
        current_time = time.time()
        for person_id, data in self.last_seen_positions.items():
            last_position = data['position']
            last_timestamp = data['timestamp']

            # Compara la posición y también si ha pasado suficiente tiempo
            if np.linalg.norm(position - last_position) < 0.4 and (current_time - last_timestamp) < 5:  # 5 segundos
                return person_id

        # Si no se encuentra una coincidencia, asigna un nuevo ID
        person_id = self.person_id_counter
        self.person_id_counter += 1
        self.last_seen_positions[person_id] = {'position': position, 'timestamp': current_time}
        return person_id
    
    def publish_general_clusters(self, points, labels):
        """Publica los clusters generales detectados como Float32MultiArray."""
        cluster_msg = Float32MultiArray()
    
        for label in set(labels):
            if label == -1:  # Ignorar ruido
                continue
            cluster_points = points[labels == label]
            for point in cluster_points:
                cluster_msg.data.extend([point[0], point[1]])  # x, y de cada punto
    
        self.general_cluster_publisher.publish(cluster_msg)
        #self.get_logger().debug(f"Clusters generales publicados: {len(set(labels)) - (1 if -1 in labels else 0)}")
    

    def publish_leg_clusters(self, points):
        """Publica los clusters de piernas detectados como Float32MultiArray."""
        cluster_msg = Float32MultiArray()
    
        for point in points:
            cluster_msg.data.extend([point[0], point[1]])  # x, y de cada punto
    
        self.leg_cluster_publisher.publish(cluster_msg)
        #self.get_logger().debug(f"Clusters de piernas publicados: {len(points)} puntos")

    def cleanup_lost_people(self, max_inactive_time=10):
        """Elimina las personas que no han sido detectadas en el último 'max_inactive_time' segundos."""
        current_time = time.time()
        self.last_seen_positions = {
            person_id: data
            for person_id, data in self.last_seen_positions.items()
            if current_time - data['timestamp'] < max_inactive_time
        }


    def publish_person_id(self, person_id, position):
        """Publica el ID de la persona y su posición"""
        person_position = Point(x=position[0], y=position[1], z=float(person_id))  # Convertir person_id a float
        self.person_position_publisher.publish(person_position)
        self.get_logger().info(f"Persona {person_id} en posición ({position[0]}, {position[1]})")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo de Detección detenido manualmente.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
