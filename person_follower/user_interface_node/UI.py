import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32MultiArray, ColorRGBA
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import subprocess


class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('user_interface_node')

        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de Interfaz de Usuario desactivado.")
            return

        self.initialize_shutdown_listener()

        # Subscripciones
        self.create_subscription(String, '/camera/status', self.camera_status_callback, 10)
        self.create_subscription(String, '/detection/status', self.detection_status_callback, 10)
        self.create_subscription(String, '/tracking/status', self.tracking_status_callback, 10)
        self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        self.create_subscription(Float32MultiArray, '/clusters/general', self.general_clusters_callback, 10)
        self.create_subscription(Float32MultiArray, '/clusters/legs', self.leg_clusters_callback, 10)
        self.create_subscription(Point, '/expected_person_position', self.person_position_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publicadores
        self.marker_pub = self.create_publisher(Marker, '/visualization/person_marker', 10)
        self.leg_cluster_pub = self.create_publisher(Marker, '/visualization/leg_clusters', 10)
        self.general_cluster_pub = self.create_publisher(Marker, '/visualization/general_clusters', 10)
        self.robot_marker_pub = self.create_publisher(Marker, '/visualization/robot_marker', 10)
        self.robot_path_pub = self.create_publisher(Marker, '/visualization/robot_path', 10)

        self.publish_robot_marker()

        # Estado del sistema
        self.person_detected = False
        self.camera_status = "Desconocido"
        self.detection_status = "Desconocido"
        self.tracking_status = "Desconocido"
        self.previous_status = {}
        self.robot_path = []

        self.start_rviz()
        self.timer = self.create_timer(5.0, self.display_status)
        self.get_logger().info("Nodo de Interfaz de Usuario iniciado.")

    def start_rviz(self):
        rviz_config_path = '/home/usuario/ros2_ws/src/rviz/config.rviz'
        try:
            self.rviz_process = subprocess.Popen(['rviz2', '-d', rviz_config_path])
            self.get_logger().info(f"RViz2 iniciado con configuración: {rviz_config_path}")
        except FileNotFoundError as e:
            self.get_logger().error(f"No se pudo iniciar RViz2. Verifica la instalación. Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error al iniciar RViz2: {e}")

    def stop_rviz(self):
        if hasattr(self, 'rviz_process') and self.rviz_process:
            self.rviz_process.terminate()
            self.get_logger().info("RViz2 detenido.")

    def camera_status_callback(self, msg): self.camera_status = msg.data
    def detection_status_callback(self, msg): self.detection_status = msg.data
    def tracking_status_callback(self, msg): self.tracking_status = msg.data
    def person_detected_callback(self, msg): self.person_detected = msg.data

    def person_position_callback(self, msg: Point):
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "person"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        self.marker_pub.publish(marker)

    def leg_clusters_callback(self, msg: Float32MultiArray):
        self.publish_cluster_marker(msg.data, "legs", 1.0, 0.0, 0.0)

    def general_clusters_callback(self, msg: Float32MultiArray):
        self.publish_cluster_marker(msg.data, "general", 0.0, 0.0, 1.0)

    def publish_robot_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = 100
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.05
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        self.robot_marker_pub.publish(marker)

    def publish_cluster_marker(self, data, ns, r, g, b):
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color = ColorRGBA(r=r, g=g, b=b, a=1.0)

        for i in range(0, len(data), 2):
            p = Point()
            p.x = data[i]
            p.y = data[i + 1]
            p.z = 0.0
            marker.points.append(p)

        if ns == "legs":
            self.leg_cluster_pub.publish(marker)
        elif ns == "general":
            self.general_cluster_pub.publish(marker)

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose.position
        p = Point(x=pose.x, y=pose.y, z=0.0)
        self.robot_path.append(p)

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_path"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        marker.points = self.robot_path[-200:]  # Limitar a últimos 200 puntos

        self.robot_path_pub.publish(marker)

    def publish_system_status_text(self):
        text = (
            f"📷 Cámara: {self.camera_status}\n"
            f"🔍 Detección: {self.detection_status}\n"
            f"📡 Seguimiento: {self.tracking_status}\n"
            f"👤 Persona: {'Sí' if self.person_detected else 'No'}"
        )
    
        marker = Marker()
        marker.header.frame_id = "odom"  # También puedes usar "map" si trabajas con localización
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "system_status"
        marker.id = 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
    
        # Posición fija en el espacio (esquina del entorno visible)
        marker.pose.position.x = -2.0
        marker.pose.position.y = 2.5
        marker.pose.position.z = 1.5
        marker.pose.orientation.w = 1.0
    
        marker.scale.z = 0.35
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = text
    
        self.marker_pub.publish(marker)


    def display_status(self):
        current_status = {
            "Cámara": self.camera_status,
            "Detección": self.detection_status,
            "Seguimiento": self.tracking_status,
            "Persona detectada": "Sí" if self.person_detected else "No"
        }
        if current_status != self.previous_status:
            print("\n=== Estado del Sistema ===")
            for key, value in current_status.items():
                print(f"{key}: {value}")
            print("==========================")
            self.previous_status = current_status
            self.publish_system_status_text()

    def initialize_shutdown_listener(self):
        self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.shutdown_confirmation_publisher = self.create_publisher(Bool, '/shutdown_confirmation', 10)

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Cierre del sistema detectado. Enviando confirmación.")
            self.stop_rviz()
            try:
                self.shutdown_confirmation_publisher.publish(Bool(data=True))
            except Exception as e:
                self.get_logger().error(f"Error al publicar confirmación de apagado: {e}")
            finally:
                self.destroy_node()

    def destroy_node(self):
        self.stop_rviz()
        self.get_logger().info("Nodo de Interfaz de Usuario detenido.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UserInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo Interfaz de Usuario detenido manualmente.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
