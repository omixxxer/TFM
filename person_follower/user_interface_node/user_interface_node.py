import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32MultiArray
from geometry_msgs.msg import Point
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
        self.create_subscription(Float32MultiArray, '/clusters/general', self.visualize_general_clusters_callback, 10)
        self.create_subscription(Float32MultiArray, '/clusters/legs', self.visualize_leg_clusters_callback, 10)
        self.create_subscription(Point, '/expected_person_position', self.visualize_person_position_callback, 10)

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

        # Inicialización de objetos scatter
        self.general_clusters_scatter = None
        self.leg_clusters_scatter = None
        self.person_position_scatter = None

        # Configuración de Matplotlib para visualización en tiempo real
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))

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

    def visualize_general_clusters_callback(self, msg):
        """Visualiza los clusters generales excluyendo los clusters de piernas."""
        data = np.array(msg.data)
        if data.size == 0:
            self.get_logger().info("No hay clusters generales para visualizar.")
            if self.general_clusters_scatter is not None:
                self.general_clusters_scatter.remove()
                self.general_clusters_scatter = None
            self.update_plot()
            return

        # Reorganizar los datos en pares (x, y)
        general_points = data.reshape(-1, 2)

        # Si hay clusters de piernas, filtrar los puntos solapados
        if self.leg_clusters_scatter is not None:
            leg_points = self.leg_clusters_scatter.get_offsets()

            # Calcular distancias y filtrar puntos
            filtered_points = []
            for point in general_points:
                distances = np.linalg.norm(leg_points - point, axis=1)
                if np.all(distances > 0.001):  # Umbral de 0.2 para considerar solapamiento
                    filtered_points.append(point)

            # Convertir a array para graficar
            general_points = np.array(filtered_points)

        # Actualizar los puntos generales
        if self.general_clusters_scatter is None:
            self.general_clusters_scatter = self.ax.scatter(
                general_points[:, 0], general_points[:, 1],
                c='blue', s=20, alpha=0.7, marker='o', label='Clusters Generales'
            )  # s: tamaño de los puntos
        else:
            self.general_clusters_scatter.set_offsets(general_points)

        # Actualizar la visualización
        self.update_plot()

    def visualize_leg_clusters_callback(self, msg):
        """Visualiza los clusters de piernas publicados como Float32MultiArray."""
        data = np.array(msg.data)
        if data.size == 0:
            self.get_logger().info("No hay clusters de piernas para visualizar.")
            if self.leg_clusters_scatter is not None:
                self.leg_clusters_scatter.remove()
                self.leg_clusters_scatter = None
            self.update_plot()
            return

        # Reorganizar los datos en pares (x, y)
        points = data.reshape(-1, 2)

        # Actualizar los puntos de piernas
        if self.leg_clusters_scatter is None:
            self.leg_clusters_scatter = self.ax.scatter(
                points[:, 0], points[:, 1],
                c='red', s=20, alpha=0.9, marker='d', label='Clusters de Piernas'
            )  # s: tamaño de los puntos
        else:
            self.leg_clusters_scatter.set_offsets(points)

        # Actualizar la visualización
        self.update_plot()

    def visualize_person_position_callback(self, msg):
        """Visualiza la posición esperada de la persona."""
        person_position = np.array([msg.x, msg.y])

        # Actualizar la posición esperada de la persona
        if self.person_position_scatter is None:
            self.person_position_scatter = self.ax.scatter(
                person_position[0], person_position[1],
                c='green', s=50, alpha=0.9, marker='x', label='Posición Estimada de la Persona'
            )  # s: tamaño de los puntos
        else:
            self.person_position_scatter.set_offsets(person_position)

        # Actualizar la visualización
        self.update_plot()

    def update_plot(self):
        """Actualiza la visualización en tiempo real."""
        self.ax.set_xlabel("Coordenada X", fontsize=12)
        self.ax.set_ylabel("Coordenada Y", fontsize=12)
        self.ax.set_title("Visualización de Clusters", fontsize=16)
        self.ax.legend(loc='upper right', fontsize=10)
        
        # Activar la cuadrícula
        self.ax.grid(True, linestyle='--', alpha=0.7)

        # Redibujar el gráfico
        plt.draw()
        plt.pause(0.001)

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
