import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
import time
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')


        # Declaración de parámetros ajustables para el nodo 
        self.declare_parameter('enabled', True)
        self.enabled = self.get_parameter('enabled').value

        if not self.enabled:
            self.get_logger().info("Nodo de Cámara desactivado.")
            return

        # Declarar parámetros necesarios
        self.declare_parameter('yolov4_weights_path', '')
        self.declare_parameter('yolov4_cfg_path', '')
        self.declare_parameter('coco_names_path', '')

        # Leer los valores de los parámetros
        weights_path = self.get_parameter('yolov4_weights_path').value
        cfg_path = self.get_parameter('yolov4_cfg_path').value
        names_path = self.get_parameter('coco_names_path').value

        # Validar que las rutas no estén vacías
        if not weights_path or not cfg_path or not names_path:
            self.get_logger().error("Rutas de YOLO no proporcionadas. Revisa el archivo de lanzamiento.")
            return

        # Inicializar suscriptor al topic de imágenes publicadas
        if self.enabled:
            self.bridge = CvBridge()
            self.image_subscriber = self.create_subscription(
                Image,
                '/image_raw',  # Ajusta el topic si es necesario
                self.process_image_callback,
                10
            )

        # Publicador de estado
        self.status_publisher = self.create_publisher(String, '/camera/status', 10)

        self.shutdown_subscription = self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)

        # Inicializar lógica de cierre
        self.initialize_shutdown_listener()

        # Control de frecuencia
        self.last_processed_time = time.time()
        self.processing_interval = 0.5  # Procesar una imagen cada 0.5 segundos

        if self.enabled:
            # Inicializar YOLO
            self.get_logger().info(f"Cargando modelo YOLO desde:\nPesos: {weights_path}\nConfig: {cfg_path}\nClases: {names_path}")
            self.net = cv2.dnn.readNet(weights_path, cfg_path)
            
            # Obtener nombres de las capas y manejar los índices de salida
            self.layer_names = self.net.getLayerNames()
            unconnected_out_layers = self.net.getUnconnectedOutLayers()

            # Manejo del cambio en getUnconnectedOutLayers
            if isinstance(unconnected_out_layers, np.ndarray):
                self.output_layers = [self.layer_names[i - 1] for i in unconnected_out_layers.flatten()]
            else:
                self.output_layers = [self.layer_names[i[0] - 1] for i in unconnected_out_layers]

            # Leer nombres de las clases
            with open(names_path, "r") as f:
                self.classes = [line.strip() for line in f.readlines()]

            self.get_logger().info("Nodo de Cámara personalizado iniciado, procesando imágenes con YOLO.")
        

    def process_image_callback(self, msg):
        """Callback para procesar las imágenes recibidas."""
        current_time = time.time()
        if current_time - self.last_processed_time < self.processing_interval:
            return  # Saltar procesamiento si no ha pasado suficiente tiempo

        self.last_processed_time = current_time
        try:
            self.get_logger().info("Imagen recibida. Iniciando procesamiento con YOLO...")

            # Convertir mensaje ROS a imagen OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.get_logger().info("Imagen convertida a formato OpenCV.")

            # Procesar la imagen para detectar personas con YOLO
            processed_frame = self.process_image(frame)
            self.get_logger().info("Procesamiento de imagen completado.")

            processed_frame_resized = cv2.resize(processed_frame, (1280, 720))    

            # Mostrar la imagen procesada
            cv2.namedWindow("Processed Image", cv2.WINDOW_NORMAL)
            cv2.imshow("Processed Image", processed_frame_resized)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error procesando la imagen: {e}")

    def process_image(self, frame):
        """Realiza el procesamiento adicional de la imagen para detectar personas usando YOLO."""
        # Preparar la imagen para YOLO
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # Inicializar listas para los resultados de la detección
        height, width, channels = frame.shape
        boxes = []
        confidences = []
        class_ids = []

        # Analizar las detecciones
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and self.classes[class_id] == "person":
                    # Obtener las coordenadas del cuadro delimitador
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Aplicar supresión de no máximos para eliminar detecciones redundantes
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return frame

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
                cv2.destroyAllWindows()  # Liberar ventanas de OpenCV al cerrar el nodo

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido manualmente.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
