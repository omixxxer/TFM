import rclpy
import numpy as np
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from nav_msgs.msg import OccupancyGrid  # Para suscripción al mapa de ocupación


class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        
        # Verificar si el nodo está habilitado
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de Seguimiento desactivado.")
            return

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

        # Servicio para habilitar o deshabilitar el seguimiento
        self.create_service(SetBool, 'enable_tracking', self.enable_tracking_callback)
        self.declare_parameter('obstacle_avoidance_enabled', True)

        self.obstacle_avoidance_enabled = self.get_parameter('obstacle_avoidance_enabled').value
        
        # Publicadores y suscripciones
        self.person_position_subscription = self.create_subscription(Point, '/person_position', self.person_position_callback, 10)
        self.person_detected_subscription = self.create_subscription(Bool, '/person_detected', self.detection_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.shutdown_subscription = self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)  # Suscripción al mapa

        self.status_publisher = self.create_publisher(String, '/tracking/status', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/tracking/velocity_cmd', 10)
        self.position_publisher = self.create_publisher(Point, '/expected_person_position', 10)  # Publicador de la posición esperada

        self.person_detected = False
        self.person_position = None  # Posición actual de la persona
        self.last_person_update_time = None  # Última vez que se actualizó la posición de la persona
        self.timeout_duration = 2.0  # Segundos antes de detener el robot si no hay actualizaciones
        self.previous_vx = 0.0 
        self.map_data = None  # Mapa de ocupación (de SLAM)
        self.last_map_data = None  # Inicializar el atributo last_map_data

        self.get_logger().info("Nodo de Seguimiento iniciado")
        
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
        z = np.array([msg.x, msg.y])  # Observación actual

        # Predicción del estado
        self.kalman_F[:2, 2:] = np.eye(2) * 0.1  # Suponiendo un delta_t = 0.1 s
        predicted_state = self.kalman_F @ self.kalman_state
        predicted_covariance = self.kalman_F @ self.kalman_covariance @ self.kalman_F.T + self.kalman_Q

        # Actualización con la observación
        y = z - (self.kalman_H @ predicted_state)  # Innovación
        S = self.kalman_H @ predicted_covariance @ self.kalman_H.T + self.kalman_R
        K = predicted_covariance @ self.kalman_H.T @ np.linalg.inv(S)  # Ganancia de Kalman
        self.kalman_state = predicted_state + K @ y
        self.kalman_covariance = (np.eye(4) - K @ self.kalman_H) @ predicted_covariance

        # Actualizar la posición estimada
        self.person_position = Point(x=self.kalman_state[0], y=self.kalman_state[1])
        self.last_person_update_time = self.get_clock().now()

        # Publicar la posición estimada
        self.position_publisher.publish(self.person_position)

        self.get_logger().info(f"Posición estimada (Kalman): x={self.person_position.x:.2f}, y={self.person_position.y:.2f}")


    def map_callback(self, msg):
        """Callback para recibir el mapa de ocupación generado por SLAM."""
        # Convertir el mapa en un formato numpy
        current_map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)

        # Solo procesar si el mapa ha cambiado
        if self.last_map_data is None or not np.array_equal(current_map_data, self.last_map_data):
            self.map_data = current_map_data
            self.get_logger().info("Mapa de ocupación recibido y procesado.")
            self.last_map_data = current_map_data
        else:
            self.get_logger().info("Mapa recibido, pero no ha cambiado, por lo que no se procesará.")


    def avoid_obstacles(self, input_msg):
        if not self.obstacle_avoidance_enabled:
            return 0.0  # No ajustar si la evasión está deshabilitada

        closest_distance = min(input_msg.ranges)
        obstacle_angle_index = input_msg.ranges.index(closest_distance)
        angle_to_obstacle = input_msg.angle_min + obstacle_angle_index * input_msg.angle_increment

        if closest_distance < 0.4:  # Distancia mínima de seguridad
            self.get_logger().warn(f"Obstáculo detectado a {closest_distance:.2f} m en ángulo {math.degrees(angle_to_obstacle):.2f}°")
            return -0.3 * angle_to_obstacle  # Ajuste angular para esquivar
        return 0.0  # No se requiere ajuste si no hay obstáculo

    def listener_callback(self, input_msg):
        if not self.tracking_enabled or not self.person_detected or not self.person_position:
            self.stop_robot()
            return

        # Verificar si la posición de la persona ha expirado
        if self.last_person_update_time is None or (self.get_clock().now() - self.last_person_update_time).nanoseconds * 1e-9 > self.timeout_duration:
            self.get_logger().warn("Tiempo de espera agotado. Deteniendo robot.")
            self.stop_robot()
            return

        # Usa la posición filtrada de la persona
        distance_to_person = math.sqrt(self.person_position.x**2 + self.person_position.y**2)
        angle_to_person = math.atan2(self.person_position.y, self.person_position.x)

        # Esquivar obstáculos
        adjustment = self.avoid_obstacles(input_msg)

        # Velocidad lineal
        max_speed = 0.8
        acceleration_limit = 0.01
        smoothing_factor = 0.4
        target_vx = min(max_speed, max(0.0, max_speed * (distance_to_person - 0.1) / 0.9)) if distance_to_person > 0.1 else 0.0
        filtered_vx = self.previous_vx * smoothing_factor + target_vx * (1 - smoothing_factor)
        vx = self.previous_vx + min(acceleration_limit, max(-acceleration_limit, filtered_vx - self.previous_vx))
        self.previous_vx = vx

        # Velocidad angular
        angle_difference = -angle_to_person
        max_angular_velocity = 1.6
        wz = 2.0 * angle_difference + adjustment
        wz = max(-max_angular_velocity, min(max_angular_velocity, wz))

        # Reducir velocidad lineal al esquivar
        if adjustment != 0.0:
            vx *= 0.8

        # Publicar mensaje de velocidad
        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.angular.z = wz
        self.velocity_publisher.publish(cmd_msg)

        # # Publicar estado
        # if distance_to_person < 0.1:
        #     self.publish_status("Detenido: demasiado cerca de la persona")
        # elif adjustment != 0.0:
        #     self.publish_status("Esquivando obstáculo")
        # else:
        #     self.publish_status("Siguiendo a la persona")

        # self.get_logger().info(f"Distancia: {distance_to_person:.2f} m, Ángulo: {math.degrees(angle_to_person):.2f}°, Vel. Lineal: {vx:.2f}, Vel. Angular: {wz:.2f}")

    def stop_robot(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.velocity_publisher.publish(cmd_msg)
        #self.get_logger().info("Robot detenido.")

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
        node.destroy_node()

if __name__ == '__main__':
    main()
