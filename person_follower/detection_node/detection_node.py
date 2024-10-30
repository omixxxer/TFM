import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
import numpy as np
from sklearn.cluster import DBSCAN
from cv_bridge import CvBridge

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridge = CvBridge()
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.detection_publisher = self.create_publisher(String, '/detection_info', 10)
        self.get_logger().info("Nodo de Detección iniciado")

    def lidar_callback(self, msg):
        points = self.process_lidar_data(msg.ranges, msg.angle_min, msg.angle_increment)
        if points:
            self.detect_clusters(points)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Procesar imagen si es necesario para detección

    def process_lidar_data(self, ranges, angle_min, angle_increment):
        points = []
        for i, r in enumerate(ranges):
            if r < 1.5:
                angle = angle_min + i * angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y))
        return points if points else None

    def detect_clusters(self, points):
        points = np.array(points)
        clustering = DBSCAN(eps=0.2, min_samples=3).fit(points)
        labels = clustering.labels_
        if len(set(labels)) - (1 if -1 in labels else 0) > 0:
            self.detection_publisher.publish(String(data="Persona detectada"))

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

