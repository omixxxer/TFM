# Librerías del sistema y ROS 2 necesarias
import os
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import subprocess

class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('user_interface_node')

        # --- Parámetros configurables ---
        self.declare_parameter('enabled', True)
        self.declare_parameter('visualization_enabled', True)
        self.enabled             = self.get_parameter('enabled').value
        self.visualization_enabled = self.get_parameter('visualization_enabled').value

        # Si el nodo no está habilitado, termina su ejecución
        if not self.enabled:
            self.get_logger().info("Nodo de Interfaz de Usuario desactivado.")
            return

        # --- Variables de estado ---
        self.person_detected   = False
        self.camera_status     = "Desconocido"
        self.detection_status  = "Desconocido"
        self.tracking_status   = "Desconocido"
        self.control_mode      = "AUTO"     
        self.previous_status   = {}
        self.teleop_status = "N/A"

        # Flags internos
        self.robot_marker_published = False
        self.last_cluster_publish_time = self.get_clock().now()

        # --- Inicialización de shutdown ---
        self.initialize_shutdown_listener()

        # --- Subscripciones a tópicos del sistema ---
        self.create_subscription(String, '/camera/status',        self.camera_status_callback,    10)
        self.create_subscription(String, '/detection/status',     self.detection_status_callback, 10)
        self.create_subscription(String, '/tracking/status',      self.tracking_status_callback,  10)
        self.create_subscription(Bool,   '/person_detected',      self.person_detected_callback,  10)
        self.create_subscription(Float32MultiArray, '/clusters/general', self.general_clusters_callback, 10)
        self.create_subscription(Float32MultiArray, '/clusters/legs',    self.leg_clusters_callback,     10)
        self.create_subscription(Point,  '/expected_person_position',    self.person_position_callback,  10)
        self.create_subscription(String, '/control/mode',         self.mode_callback,             10)
        self.create_subscription(String, '/control/teleop_status', self.teleop_status_callback, 10)

        # --- Publicadores de visualización y diagnóstico ---
        self.marker_pub        = self.create_publisher(Marker,         '/visualization/person_marker', 10)
        self.leg_cluster_pub   = self.create_publisher(Marker,         '/visualization/leg_clusters',  10)
        self.general_cluster_pub = self.create_publisher(Marker,       '/visualization/general_clusters', 10)
        self.robot_marker_pub  = self.create_publisher(Marker,         '/visualization/robot_marker', 10)
        self.status_text_pub   = self.create_publisher(Marker,         '/visualization/status_text',  10)
        self.diagnostic_pub    = self.create_publisher(DiagnosticArray,'/diagnostics',              10)

        # --- Lanzar RViz ---
        self.start_rviz()

        # --- Temporizador para refrescar estado (consola + HUD) ---
        self.timer = self.create_timer(5.0, self.display_status)

        self.get_logger().info("Nodo de Interfaz de Usuario iniciado.")

    def start_rviz(self):
        rviz_config = '/home/usuario/ros2_ws/src/rviz/config.rviz'
        self.stop_rviz()
        try:
            self.rviz_process = subprocess.Popen(['rviz2', '-d', rviz_config])
            self.get_logger().info(f"RViz2 iniciado con configuración: {rviz_config}")
        except Exception as e:
            self.get_logger().error(f"No se pudo iniciar RViz2: {e}")

    def stop_rviz(self):
        if hasattr(self, 'rviz_process') and self.rviz_process:
            self.rviz_process.terminate()
            self.get_logger().info("RViz2 detenido.")

    # --- Callbacks de estado ---
    def camera_status_callback(self, msg: String):
        self.camera_status = msg.data

    def detection_status_callback(self, msg: String):
        self.detection_status = msg.data

    def tracking_status_callback(self, msg: String):
        self.tracking_status = msg.data

    def person_detected_callback(self, msg: Bool):
        self.person_detected = msg.data
        
    def teleop_status_callback(self, msg: String):
        self.teleop_status = msg.data


    def mode_callback(self, msg: String):
        if msg.data != self.control_mode:
            self.control_mode = msg.data
            self.get_logger().info(f"Modo de control cambiado a: {self.control_mode}")
            # publicar diagnóstico
            diag = DiagnosticArray()
            status = DiagnosticStatus()
            status.name = "Control Mode"
            status.level = DiagnosticStatus.OK
            status.message = self.control_mode
            status.values = [KeyValue(key="mode", value=self.control_mode)]
            diag.status.append(status)
            self.diagnostic_pub.publish(diag)

    # --- Callbacks de clusters/persona ---
    def person_position_callback(self, msg: Point):
        if not self.visualization_enabled:
            return
        m = Marker()
        m.header.frame_id = "base_footprint"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "person"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = -msg.x
        m.pose.position.y = -msg.y
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.4
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        self.marker_pub.publish(m)

    def leg_clusters_callback(self, msg: Float32MultiArray):
        if not self.visualization_enabled:
            return
        elapsed = (self.get_clock().now() - self.last_cluster_publish_time).nanoseconds * 1e-9
        if elapsed < 0.5:
            return
        self.last_cluster_publish_time = self.get_clock().now()
        data = msg.data
        if not data or len(data) % 2 != 0:
            self.get_logger().warn("Datos inválidos en clusters de piernas.")
            return
        self.publish_cluster_marker(data, "legs", 1.0, 0.0, 0.0)

    def general_clusters_callback(self, msg: Float32MultiArray):
        if not self.visualization_enabled:
            return
        elapsed = (self.get_clock().now() - self.last_cluster_publish_time).nanoseconds * 1e-9
        if elapsed < 0.5:
            return
        self.last_cluster_publish_time = self.get_clock().now()
        data = msg.data
        if not data or len(data) % 2 != 0:
            self.get_logger().warn("Datos inválidos en clusters generales.")
            return
        self.publish_cluster_marker(data, "general", 0.0, 0.0, 1.0)

    def publish_cluster_marker(self, data, ns, r, g, b):
        m = Marker()
        m.header.frame_id = "base_footprint"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        for i in range(0, len(data), 2):
            p = Point(x=-data[i], y=-data[i+1], z=0.0)
            m.points.append(p)
        if ns == "legs":
            self.leg_cluster_pub.publish(m)
        else:
            self.general_cluster_pub.publish(m)

    def publish_robot_marker(self):
        if self.robot_marker_published:
            return
        m = Marker()
        m.header.frame_id = "base_footprint"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "robot"
        m.id = 100
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.z = 0.25
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.5
        m.scale.z = 0.5
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        self.robot_marker_pub.publish(m)
        self.robot_marker_published = True

    def publish_legend_hud(self):
        m = Marker()
        m.header.frame_id = "base_footprint"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "legend_hud"
        m.id = 400
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = 2.0
        m.pose.position.z = 1.5
        m.pose.orientation.w = 1.0
        m.scale.z = 0.3
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        m.text = (
            "Leyenda:\n"
            "🟡 Base Robot\n"
            "🟢 Persona Detectada\n"
            "🔵 Clusters Generales\n"
            "🔴 Clusters Piernas\n"
            "⚙️ Modo Control"
        )
        self.status_text_pub.publish(m)

    def publish_fixed_text_marker(self, ns, text, x, y, z, marker_id):
        marker = Marker()
        marker.header.frame_id = "base_footprint"  # o 'odom' si lo prefieres
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.25
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = text
        self.status_text_pub.publish(marker)

    def display_status(self):
        # RViz: marcador HUD fijo frente al robot
        self.publish_robot_marker()
    
        base_x, base_y = 1.2, 0.0  # Panel frente al robot
    
        self.publish_fixed_text_marker("panel", f"Cámara: {self.camera_status}", base_x, base_y, 1.5, 300)
        self.publish_fixed_text_marker("panel", f"Detección: {self.detection_status}", base_x, base_y, 1.3, 301)
        self.publish_fixed_text_marker("panel", f"Tracking: {self.tracking_status}", base_x, base_y, 1.1, 302)
        self.publish_fixed_text_marker("panel", f"Persona: {'Sí' if self.person_detected else 'No'}", base_x, base_y, 0.9, 303)
        self.publish_fixed_text_marker("panel", f"Modo: {self.control_mode}", base_x, base_y, 0.7, 304)
        self.publish_fixed_text_marker("panel", f"Teleop: {self.teleop_status}", base_x, base_y, 0.5, 305)
    
        if not hasattr(self, 'legend_published'):
            self.publish_legend_hud()
            self.legend_published = True
    
        # Consola
        current = {
            "Camera":    self.camera_status,
            "Detection": self.detection_status,
            "Tracking":  self.tracking_status,
            "Person":    "Sí" if self.person_detected else "No",
            "Mode":      self.control_mode,
            "Teleop":    self.teleop_status
        }
    
        if current != self.previous_status:
            self.get_logger().info("=== Estado del Sistema ===")
            for k,v in current.items():
                self.get_logger().info(f"{k}: {v}")
            self.get_logger().info("==========================")
            self.previous_status = current

    # --- Shutdown handling ---
    def initialize_shutdown_listener(self):
        self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.shutdown_confirmation_pub = self.create_publisher(Bool, '/shutdown_confirmation', 10)

    def shutdown_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Cierre detectado. Enviando confirmación.")
            self.stop_rviz()
            self.shutdown_confirmation_pub.publish(Bool(data=True))
            self.destroy_node()

    def destroy_node(self):
        self.stop_rviz()
        self.get_logger().info("Nodo de Interfaz de Usuario detenido.")
        super().destroy_node()


def main(args=None):
    rclpy.init()
    node = UserInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("UserInterfaceNode detenido con Ctrl-C.")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()