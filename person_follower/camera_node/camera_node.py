import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declaración de parámetros ajustables para el nodo 
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de Cámara desactivado.")
            return

        # Inicializar lógica de cierre
        self.initialize_shutdown_listener()
        
        # Declaración de parámetros
        self.declare_parameter('use_yolo', False)
        self.declare_parameter('use_slam', True)
        self.declare_parameter('yolov4_weights_path', '')
        self.declare_parameter('yolov4_cfg_path', '')
        self.declare_parameter('coco_names_path', '')

        # Leer valores de los parámetros
        self.use_yolo = self.get_parameter('use_yolo').value
        self.use_slam = self.get_parameter('use_slam').value

        if not self.enabled:
            self.get_logger().info("Nodo de Cámara desactivado.")
            return

        # Inicializar publicadores
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, '/image_raw', 10)
        self.map_publisher = self.create_publisher(Image, '/map_visual', 10)
        self.status_publisher = self.create_publisher(String, '/camera/status', 10)
        self.shutdown_subscription = self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)

        # Inicializar captura de cámara
        self.cap = cv2.VideoCapture(0)  # Logitech C270
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Si YOLO está activado, inicializarlo
        if self.use_yolo:
            self.get_logger().info("Inicializando YOLO...")
            self.initialize_yolo()
        
        # Si SLAM está activado, inicializarlo
        if self.use_slam:
            self.get_logger().info("Inicializando ORB-SLAM3...")
            self.initialize_slam()
        
        self.timer = self.create_timer(0.1, self.capture_frame)
        self.get_logger().info("Nodo de Cámara iniciado correctamente.")
    
    def initialize_yolo(self):
        weights_path = self.get_parameter('yolov4_weights_path').value
        cfg_path = self.get_parameter('yolov4_cfg_path').value
        names_path = self.get_parameter('coco_names_path').value
        
        if not weights_path or not cfg_path or not names_path:
            self.get_logger().error("Rutas de YOLO no proporcionadas. Desactivando YOLO.")
            self.use_yolo = False
            return
        
        self.net = cv2.dnn.readNet(weights_path, cfg_path)
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers().flatten()]
        with open(names_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
    
    def initialize_slam(self):
        self.get_logger().info("ORB-SLAM3 aún no está implementado en este nodo.")
    
    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("No se pudo capturar la imagen de la cámara.")
            return
        
        # Publicar imagen cruda
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_publisher.publish(img_msg)
        
        if self.use_yolo:
            frame = self.process_yolo(frame)
        
        if self.use_slam:
            map_visual = self.process_slam(frame)
            map_msg = self.bridge.cv2_to_imgmsg(map_visual, encoding="mono8")
            self.map_publisher.publish(map_msg)
    
    def process_yolo(self, frame):
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        
        height, width, _ = frame.shape
        boxes, confidences, class_ids = [], [], []
        
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and self.classes[class_id] == "person":
                    center_x, center_y, w, h = (detection[:4] * np.array([width, height, width, height])).astype('int')
                    x, y = center_x - w // 2, center_y - h // 2
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        for i in indexes.flatten():
            x, y, w, h = boxes[i]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        return frame
    
    def process_slam(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return gray  # Por ahora, devuelve la imagen en escala de grises
    
    def initialize_shutdown_listener(self):
        """Inicializa el suscriptor para manejar el cierre del sistema."""
        self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.shutdown_confirmation_publisher = self.create_publisher(Bool, '/shutdown_confirmation', 10)

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
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido manualmente.")
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
