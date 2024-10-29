import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VisionProcessor(Node):
    
    def __init__(self):
        super().__init__('vision_processor')
        
        # Suscripción al topic de la cámara
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Asegúrate de que este sea el topic correcto
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        print("Nodo de procesamiento de visión iniciado.")
    
    def image_callback(self, msg):
        try:
            # Convertir el mensaje de imagen ROS a una imagen de OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Muestra la imagen procesada
            cv2.imshow("Imagen original", cv_image)
            
            # Configura 'q' para cerrar la ventana y terminar el nodo
            if cv2.waitKey(10) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    vision_processor = VisionProcessor()
    rclpy.spin(vision_processor)
    
    # Finaliza y cierra las ventanas de OpenCV
    vision_processor.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
