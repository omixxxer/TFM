import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
import cv2


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

        # Configuración de la cámara
        self.camera_active = True
        self.cap = cv2.VideoCapture(0)  # Usar la cámara por defecto
        if not self.cap.isOpened():
            self.camera_active = False
            self.get_logger().warning("No se pudo abrir la cámara.")

        # Timer para actualizar la visualización cada 0.1 segundos
        self.timer = self.create_timer(0.1, self.update_visualization)

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

    def visualize_clusters_callback(self, msg):
        """Visualiza los clusters publicados como Float32MultiArray."""
        data = np.array(msg.data)
        if data.size == 0:
            self.get_logger().info("No hay clusters para visualizar.")
            return

        # Reorganizar los datos en pares (x, y)
        self.points = data.reshape(-1, 2)
        self.labels = self.get_cluster_labels(self.points)  # Ajustar según tu lógica de agrupación

    def get_cluster_labels(self, points):
        """Genera etiquetas de clusters con DBSCAN."""
        from sklearn.cluster import DBSCAN
        clustering = DBSCAN(eps=0.5, min_samples=5).fit(points)
        return clustering.labels_

    def update_visualization(self):
        """Actualiza la visualización del sistema (clusters, estado, cámara)."""
        # Limpia la gráfica
        self.ax.clear()

        # Mostrar el estado del sistema
        status_text = f"Cámara: {self.camera_status}\nDetección: {self.detection_status}\n" \
                      f"Seguimiento: {self.tracking_status}\nPersona detectada: {'Sí' if self.person_detected else 'No'}"
        self.ax.text(0.05, 0.95, status_text, transform=self.ax.transAxes, fontsize=10,
                     verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # Visualizar clusters
        if hasattr(self, 'points') and hasattr(self, 'labels'):
            unique_labels = np.unique(self.labels)
            for label in unique_labels:
                if label == -1:
                    continue
                cluster_points = self.points[self.labels == label]
                self.ax.scatter(cluster_points[:, 0], cluster_points[:, 1], label=f'Cluster {label}')
            self.ax.legend(loc='upper right')

        # Configuración de la gráfica
        self.ax.set_title("Visualización de Clusters y Estado del Sistema")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")

        # Actualizar la visualización
        plt.draw()
        plt.pause(0.001)

        # Mostrar la cámara si está activa
        if self.camera_active:
            ret, frame = self.cap.read()
            if ret:
                cv2.imshow("Cámara", frame)
            else:
                self.get_logger().warning("No se pudo capturar la imagen de la cámara.")

        # Manejar cierre de ventanas
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Cerrando visualización.")
            self.cap.release()
            cv2.destroyAllWindows()
            self.destroy_node()


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
