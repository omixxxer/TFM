import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('user_interface_node')

        # Verificar si el nodo está habilitado
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Interfaz de Usuario desactivado.")
            return

        # Subscripciones para obtener estados
        self.create_subscription(String, '/camera/status', self.camera_status_callback, 10)
        self.create_subscription(String, '/detection/status', self.detection_status_callback, 10)
        self.create_subscription(String, '/tracking/status', self.tracking_status_callback, 10)
        self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        self.create_subscription(Float32MultiArray, '/detection/clusters', self.visualize_clusters_callback, 10)

        # Publicador para confirmar el apagado
        self.shutdown_confirmation_publisher = self.create_publisher(Bool, '/shutdown_confirmation', 10)
        self.shutdown_subscription = self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)

        # Estado del sistema
        self.person_detected = False
        self.camera_status = "Desconocido"
        self.detection_status = "Desconocido"
        self.tracking_status = "Desconocido"

        # Estado previo para evitar logs repetitivos
        self.previous_status = {}

        # Configuración de Matplotlib para visualización en tiempo real
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.scatter = None

        # Timer para mostrar el estado en la terminal cada 5 segundos
        self.timer = self.create_timer(5.0, self.display_status)

        self.get_logger().info("Nodo de Interfaz de Usuario iniciado.")

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

    def camera_status_callback(self, msg):
        self.camera_status = msg.data

    def detection_status_callback(self, msg):
        self.detection_status = msg.data

    def tracking_status_callback(self, msg):
        self.tracking_status = msg.data

    def person_detected_callback(self, msg):
        self.person_detected = msg.data

    def display_status(self):
        """Muestra el estado actual del sistema en la terminal."""
        current_status = {
            "Cámara": self.camera_status,
            "Detección": self.detection_status,
            "Seguimiento": self.tracking_status,
            "Persona detectada": "Sí" if self.person_detected else "No"
        }

        # Mostrar el estado solo si hay cambios
        if current_status != self.previous_status:
            print("\n=== Estado del Sistema ===")
            for key, value in current_status.items():
                print(f"{key}: {value}")
            print("==========================")
            self.previous_status = current_status

    def visualize_clusters_callback(self, msg):
        """Visualiza los clusters publicados como Float32MultiArray."""
        data = np.array(msg.data)
        if data.size == 0:
            self.get_logger().info("No hay clusters para visualizar.")
            return

        # Reorganizar los datos en pares (x, y)
        points = data.reshape(-1, 2)

        # Suponiendo que los clusters se reciben como etiquetas
        labels = self.get_cluster_labels(points)  # O ajusta según tu método de generación de etiquetas

        # Limpiar completamente la gráfica antes de dibujar nuevos datos
        self.ax.clear()

        # Graficar los puntos de los clusters, diferenciados por colores
        unique_labels = np.unique(labels)
        for label in unique_labels:
            if label == -1:  # Ignorar los puntos de ruido (DBSCAN marca como -1)
                continue

            cluster_points = points[labels == label]  # Filtrar puntos por etiqueta
            self.ax.scatter(cluster_points[:, 0], cluster_points[:, 1], label=f'Cluster {label}')

        # Etiquetas y configuración de la gráfica
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Visualización de Clusters")
        self.ax.legend(loc='upper right')

        # Actualizar la visualización
        plt.draw()
        plt.pause(0.001)

    def get_cluster_labels(self, points):
        """Simulación de un método para obtener las etiquetas de los clusters.
        En el código real, deberías utilizar algo como DBSCAN u otro algoritmo de agrupamiento."""
        # Este es un ejemplo, debes reemplazarlo con tu lógica de agrupación real.
        from sklearn.cluster import DBSCAN

        clustering = DBSCAN(eps=0.5, min_samples=5).fit(points)
        return clustering.labels_



def main(args=None):
    rclpy.init(args=args)
    node = UserInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo Interfaz de Usuario detenido manualmente.")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

