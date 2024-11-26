import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point
from rclpy.time import Time
import math

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        
        # Estado del nodo de seguimiento
        self.tracking_enabled = False
        self.last_obstacle_state = None  # Estado previo del obstáculo

        # Servicio para habilitar o deshabilitar el seguimiento
        self.create_service(SetBool, 'enable_tracking', self.enable_tracking_callback)

        # Publicadores y suscripciones
        self.person_position_subscription = self.create_subscription(Point, '/person_position', self.person_position_callback, 10)

        self.status_publisher = self.create_publisher(String, '/tracking/status', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/commands/velocity', 10)
        self.person_detected_subscription = self.create_subscription(Bool, '/person_detected', self.detection_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.shutdown_subscription = self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        
        self.person_detected = False
        self.person_position = None  # Posición actual de la persona
        self.last_person_update_time = None  # Última vez que se actualizó la posición de la persona
        self.timeout_duration = 2.0  # Segundos antes de detener el robot si no hay actualizaciones


        self.get_logger().info("Nodo de Seguimiento iniciado")
        self.publish_status("Nodo de Seguimiento iniciado.")

    def enable_tracking_callback(self, request, response):
        self.tracking_enabled = request.data
        response.success = True
        response.message = f"Tracking {'enabled' if self.tracking_enabled else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def detection_callback(self, msg):
        self.person_detected = msg.data

    def person_position_callback(self, msg):
        self.person_position = msg
        self.last_person_update_time = self.get_clock().now()  # Actualiza el tiempo
        self.get_logger().info(f"Posición recibida: x={msg.x:.2f}, y={msg.y:.2f}")


    def avoid_obstacles(self, input_msg):
        closest_distance = min(input_msg.ranges)
        obstacle_angle_index = input_msg.ranges.index(closest_distance)
        angle_to_obstacle = input_msg.angle_min + obstacle_angle_index * input_msg.angle_increment

        if closest_distance < 0.4:  # Distancia mínima de seguridad
            self.get_logger().warn(f"Obstáculo detectado a {closest_distance:.2f} m en ángulo {math.degrees(angle_to_obstacle):.2f}°")
            # Cambiar la dirección del robot ligeramente para evitar el obstáculo
            return -0.3 * angle_to_obstacle  # Devuelve un ajuste angular para esquivar
        return 0.0  # No se requiere ajuste si no hay obstáculo


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

    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))

    def listener_callback(self, input_msg):
        if not self.tracking_enabled or not self.person_detected or not self.person_position:
            return

        # Verificar si la posición de la persona ha expirado
        if self.last_person_update_time is None or (self.get_clock().now() - self.last_person_update_time).nanoseconds * 1e-9 > self.timeout_duration:
            self.get_logger().warn("Tiempo de espera agotado. Deteniendo robot.")
            self.stop_robot()
            return

        # Usa la posición de la persona para calcular la dirección y la distancia
        distance_to_person = math.sqrt(self.person_position.x**2 + self.person_position.y**2)
        angle_to_person = math.atan2(self.person_position.y, self.person_position.x)

        # Esquivar obstáculos
        adjustment = self.avoid_obstacles(input_msg)

        # Velocidad lineal proporcional
        vx = min(0.45, max(0.0, 0.45 * (distance_to_person - 0.1))) if distance_to_person > 0.1 else 0.0

        angle_difference = -angle_to_person
        max_angular_velocity = 1.6
        wz = 2.0 * angle_difference
        if abs(wz) > max_angular_velocity:
            wz = max_angular_velocity if wz > 0 else -max_angular_velocity

        # Publicar mensaje de velocidad
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = vx
        cmd_vel_msg.angular.z = wz
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
        self.get_logger().info(f"Distancia: {distance_to_person:.2f} m, Ángulo: {math.degrees(angle_to_person):.2f}°, Vel. Lineal: {vx:.2f}, Vel. Angular: {wz:.2f}")

    def stop_robot(self):
        control_msg = Twist()
        control_msg.linear.x = 0.0
        control_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(control_msg)
        self.get_logger().info("Robot detenido.")


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