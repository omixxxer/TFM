import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import math
import time
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN

class PersonFollower(Node):

    def __init__(self, enable_rotation=True, enable_visualization=True):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Suscripción al LIDAR
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        
        # Suscripción al topic de la cámara
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Inicializar CvBridge para convertir mensajes de ROS a imágenes de OpenCV
        self.bridge = CvBridge()

        # Variables de estado y parámetros
        self.min_distance = 0.4  # Distancia mínima para evitar colisiones
        self.angle_filter_window = 5  # Tamaño de la ventana para suavizado de ángulos
        self.enable_rotation = enable_rotation
        self.enable_visualization = enable_visualization
        self.prev_angles = []  # Almacena ángulos anteriores para suavizado
        self.person_detected = False
        self.object_detected = False
        self.rotation_triggered = False

        self.get_logger().info("Nodo de seguimiento de personas iniciado.")

    def image_callback(self, msg):
        try:
            # Convertir el mensaje de imagen ROS a una imagen de OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Muestra la imagen procesada
            cv2.imshow("Vista de la cámara", cv_image)
            cv2.waitKey(1)

            # Mensaje de depuración para confirmar que la imagen se está recibiendo
            self.get_logger().info("Imagen recibida desde la cámara.")

        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")

    def detect_person(self, ranges, angle_min, angle_increment):
        points = []
        for i, r in enumerate(ranges):
            if r < 1.5:  # Considerar solo distancias menores a 1.5 metros
                angle = angle_min + i * angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y))

        if len(points) == 0:
            return False

        # Agrupar puntos usando DBSCAN
        points = np.array(points)
        clustering = DBSCAN(eps=0.2, min_samples=3).fit(points)

        labels = clustering.labels_
        num_personas = len(set(labels)) - (1 if -1 in labels else 0)

        if self.enable_visualization:
            self.plot_clusters(points, labels)

        if num_personas > 0:
            return True

        return False

    def plot_clusters(self, points, labels):
        unique_labels = set(labels)
        plt.figure(figsize=(8, 6))

        for label in unique_labels:
            if label == -1:
                color = 'k'  # Color negro para el ruido
            else:
                color = plt.cm.jet(float(label) / len(unique_labels))

            class_member_mask = (labels == label)
            xy = points[class_member_mask]
            plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=color, markeredgecolor='k', markersize=6)

        plt.title('DBSCAN Clustering Result')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        plt.grid()
        plt.show()

    def smooth_angle(self, angle):
        angles_rad = [math.radians(a) for a in self.prev_angles]
        angle_rad = math.radians(angle)

        angles_rad.append(angle_rad)

        if len(angles_rad) > self.angle_filter_window:
            angles_rad.pop(0)

        avg_angle_rad = sum(math.atan2(math.sin(a), math.cos(a)) for a in angles_rad) / len(angles_rad)
        avg_angle_deg = math.degrees(avg_angle_rad)

        return avg_angle_deg

    def listener_callback(self, input_msg):
        angle_min = input_msg.angle_min
        angle_increment = input_msg.angle_increment
        ranges = input_msg.ranges
        
        vx = 0.0  # Inicializar velocidad lineal
        wz = 0.0  # Inicializar velocidad angular

        if self.detect_person(ranges, angle_min, angle_increment):
            self.person_detected = True
            self.object_detected = False
        else:
            self.person_detected = False

        if not self.person_detected and not self.object_detected:
            self.object_detected = True

        if self.person_detected:
            min_range_index = ranges.index(min(ranges))
            angle_to_person = angle_min + min_range_index * angle_increment

            if self.prev_angles:
                angle_to_person = self.smooth_angle(angle_to_person - angle_min)

            if min(ranges) < self.min_distance:
                vx = 0.05  # Velocidad reducida
            else:
                vx = 0.35  # Velocidad normal

            target_angle = angle_to_person
            angle_difference = -target_angle

            max_angular_velocity = 0.6
            if abs(angle_difference) > max_angular_velocity:
                angle_difference = max_angular_velocity if angle_difference > 0 else -max_angular_velocity

            wz = 2.0 * angle_difference

            output_msg = Twist()
            output_msg.linear.x = vx
            output_msg.angular.z = wz
            self.publisher_.publish(output_msg)

            self.rotation_triggered = False
            
        else:
            if self.enable_rotation and not self.rotation_triggered:
                self.rotate_180_degrees()

    def rotate_180_degrees(self):
        output_msg = Twist()
        output_msg.angular.z = math.pi * 2  # Rotar 360 grados
        self.publisher_.publish(output_msg)
        time.sleep(2)  # Esperar 2 segundos después de la rotación
        output_msg.angular.z = 0.0  # Detener la rotación
        self.publisher_.publish(output_msg)
        self.rotation_triggered = True  # Marcar como completada la rotación

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower(enable_rotation=True, enable_visualization=False)
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

