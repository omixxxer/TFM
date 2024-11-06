# camera_node/camera_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Verificar si el nodo está habilitado
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value
        if not self.enabled:
            self.get_logger().info("Nodo de Cámara desactivado.")
            self.publish_status("Nodo de Cámara desactivado.")
            return
            
           
        # Publicador de estado
        self.status_publisher = self.create_publisher(String, '/camera/status', 10)
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_image)
        self.get_logger().info("Nodo de Cámara iniciado, publicando en '/camera/image_raw'.")
        self.publish_status("Nodo de Cámara iniciado.")

    def publish_status(self, message):
        self.status_publisher.publish(String(data=message))
    
    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(msg)
                self.get_logger().info("Imagen capturada y publicada.")
            except Exception as e:
                self.get_logger().error(f"Error al publicar la imagen: {e}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    if node.enabled:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

