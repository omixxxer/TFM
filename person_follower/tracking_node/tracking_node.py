import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import csv
from heapq import heappush, heappop

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')

        # Estado del nodo de seguimiento
        self.tracking_enabled = False
        self.last_obstacle_state = None  # Estado previo del obstáculo

        # Inicialización del filtro de Kalman
        self.kalman_state = np.zeros(4)  # [x, y, vx, vy]
        self.kalman_covariance = np.eye(4) * 0.1
        self.kalman_F = np.eye(4)  # Matriz de transición de estado
        self.kalman_H = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0]])  # Matriz de observación
        self.kalman_R = np.eye(2) * 0.05  # Covarianza del ruido de observación
        self.kalman_Q = np.eye(4) * 0.01  # Covarianza del ruido del proceso

        # Almacenamiento del historial de trayectorias
        self.trajectory_history = []  # Lista para almacenar puntos visitados
        self.max_history_length = 1000  # Máximo número de puntos en el historial

        # Servicio para habilitar o deshabilitar el seguimiento
        self.create_service(SetBool, 'enable_tracking', self.enable_tracking_callback)
        self.declare_parameter('obstacle_avoidance_enabled', True)

        self.obstacle_avoidance_enabled = self.get_parameter('obstacle_avoidance_enabled').value

        # Publicadores y suscripciones
        self.person_position_subscription = self.create_subscription(Point, '/person_position', self.person_position_callback, 10)
        self.person_detected_subscription = self.create_subscription(Bool, '/person_detected', self.detection_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.shutdown_subscription = self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.status_publisher = self.create_publisher(String, '/tracking/status', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/tracking/velocity_cmd', 10)
        self.position_publisher = self.create_publisher(Point, '/expected_person_position', 10)  # Publicador de la posición esperada

        self.person_detected = False
        self.person_position = None  # Posición actual de la persona
        self.last_person_update_time = None  # Última vez que se actualizó la posición de la persona
        self.timeout_duration = 2.0  # Segundos antes de detener el robot si no hay actualizaciones
        self.previous_vx = 0.0

        self.map_data = None  # Mapa recibido para planificación

        self.get_logger().info("Nodo de Seguimiento con Filtro de Kalman iniciado")
        self.publish_status("Nodo de Seguimiento con Filtro de Kalman iniciado.")

    def enable_tracking_callback(self, request, response):
        self.tracking_enabled = request.data
        response.success = True
        response.message = f"Tracking {'enabled' if self.tracking_enabled else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def detection_callback(self, msg):
        self.person_detected = msg.data

    def person_position_callback(self, msg):
        """Callback para procesar la posición de la persona."""
        self.person_position = msg  # Actualizar posición detectada
        self.last_person_update_time = self.get_clock().now()

        # Actualizar el historial de trayectorias
        self.update_trajectory_history(self.person_position)

    def update_trajectory_history(self, position):
        if len(self.trajectory_history) >= self.max_history_length:
            self.trajectory_history.pop(0)  # Eliminar el punto más antiguo si se supera el límite
        self.trajectory_history.append((position.x, position.y))
        self.get_logger().info(f"Historial actualizado: {len(self.trajectory_history)} puntos")

    def export_trajectory_history(self, filename='trajectory_history.csv'):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y'])
            writer.writerows(self.trajectory_history)
        self.get_logger().info(f"Historial de trayectorias exportado a {filename}")

    def map_callback(self, msg):
        """Callback para manejar el mapa recibido."""
        self.map_data = msg
        self.get_logger().info("Mapa recibido para planificación de trayectorias.")

    def plan_path_to_person(self):
        """Calcula una trayectoria óptima hacia la persona detectada."""
        if not self.map_data or not self.person_position:
            self.get_logger().warn("Mapa o posición de la persona no disponibles para planificación.")
            return

        # Obtener datos del mapa
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        grid = np.array(self.map_data.data).reshape((height, width))

        # Convertir posiciones a índices de cuadrícula
        def world_to_grid(x, y):
            gx = int((x - origin_x) / resolution)
            gy = int((y - origin_y) / resolution)
            return gx, gy

        def grid_to_world(gx, gy):
            x = gx * resolution + origin_x
            y = gy * resolution + origin_y
            return x, y

        robot_pos = world_to_grid(0, 0)  # Suponiendo posición inicial del robot en (0,0)
        person_pos = world_to_grid(self.person_position.x, self.person_position.y)

        # Implementar algoritmo A*
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        open_set = []
        heappush(open_set, (0, robot_pos))
        came_from = {}
        cost_so_far = {robot_pos: 0}

        while open_set:
            _, current = heappop(open_set)

            if current == person_pos:
                break

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)

                if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height and grid[neighbor[1], neighbor[0]] == 0:
                    new_cost = cost_so_far[current] + 1
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic(person_pos, neighbor)
                        heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

        # Reconstruir el camino
        path = []
        current = person_pos
        while current != robot_pos:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                self.get_logger().warn("No se pudo encontrar un camino válido.")
                return

        path.reverse()
        world_path = [grid_to_world(*p) for p in path]
        self.get_logger().info(f"Camino planificado: {world_path}")
        return world_path

    def listener_callback(self, input_msg):
        if not self.tracking_enabled or not self.person_detected or not self.person_position:
            self.stop_robot()
            return

        # Verificar si la posición de la persona ha expirado
        if self.last_person_update_time is None or (self.get_clock().now() - self.last_person_update_time).nanoseconds * 1e-9 > self.timeout_duration:
            self.get_logger().warn("Tiempo de espera agotado. Deteniendo robot.")
            self.stop_robot()
            return

        # Planificar camino hacia la persona
        path = self.plan_path_to_person()
        if path:
            next_goal = path[0]  # Obtener el siguiente punto en la trayectoria
            distance_to_goal = math.sqrt(next_goal[0]**2 + next_goal[1]**2)
            angle_to_goal = math.atan2(next_goal[1], next_goal[0])

            # Publicar velocidades hacia el objetivo
            cmd_msg = Twist()
            cmd_msg.linear.x = min(0.5, distance_to_goal)  # Limitar velocidad lineal
            cmd_msg.angular.z = angle_to_goal
            self.velocity_publisher.publish(cmd_msg)

    def stop_robot(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.velocity_publisher.publish(cmd_msg)
        self.get_logger().info("Robot detenido.")

    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))

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


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo de Seguimiento detenido manualmente.")
    finally:
        node.export_trajectory_history()
        node.destroy_node()

if __name__ == '__main__':
    main()
