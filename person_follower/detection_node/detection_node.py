# detection_node/detection_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

class DetectionNode(Node):
    def __init__(self, enable_visualization=True):
        super().__init__('detection_node')
        self.detection_publisher = self.create_publisher(Bool, '/person_detected', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.enable_visualization = enable_visualization
        self.get_logger().info("Nodo de Detección iniciado")

    def lidar_callback(self, msg):
        person_detected = self.detect_person(msg.ranges, msg.angle_min, msg.angle_increment)
        self.detection_publisher.publish(Bool(data=person_detected))

    def detect_person(self, ranges, angle_min, angle_increment):
        points = []
        for i, r in enumerate(ranges):
            if r < 1.5:
                angle = angle_min + i * angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y))

        if not points:
            return False

        points = np.array(points)
        clustering = DBSCAN(eps=0.2, min_samples=3).fit(points)
        labels = clustering.labels_

        if self.enable_visualization:
            self.plot_clusters(points, labels)

        return len(set(labels)) - (1 if -1 in labels else 0) > 0

    def plot_clusters(self, points, labels):
        unique_labels = set(labels)
        plt.figure(figsize=(8, 6))
        for label in unique_labels:
            color = 'k' if label == -1 else plt.cm.jet(float(label) / len(unique_labels))
            xy = points[labels == label]
            plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=color, markeredgecolor='k', markersize=6)
        plt.title('DBSCAN Clustering Result')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        plt.grid()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode(enable_visualization=True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
