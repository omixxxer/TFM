import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String, Bool
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import time


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Parámetros de activación y visualización
        self.declare_parameter('enabled', True)
        self.declare_parameter('visualize', True)
        self.enabled = self.get_parameter('enabled').value
        self.visualize = self.get_parameter('visualize').value

        if not self.enabled:
            self.get_logger().info("Nodo de Cámara desactivado.")
            return

        self.get_logger().info("Inicializando nodo Cámara con MediaPipe...")

        # Inicializar bridge y suscripción con cola mínima
        qos = QoSProfile(depth=1)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, qos)

        # Publicadores
        self.keypoints_pub = self.create_publisher(Float32MultiArray, '/pose/keypoints', 10)
        self.status_pub = self.create_publisher(String, '/camera/status', 10)
        self.shutdown_sub = self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.visual_detection_pub = self.create_publisher(Bool, '/person_detected_visual', 10)
        self.shutdown_confirmation_pub = self.create_publisher(Bool, '/shutdown_confirmation', 10)
        # Publicador para comandos de gesto
        self.gesture_pub = self.create_publisher(String, '/gesture_command', 10)

        # Inicializar MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(static_image_mode=False,
                                      model_complexity=1,
                                      enable_segmentation=False,
                                      min_detection_confidence=0.5,
                                      min_tracking_confidence=0.5)

        self.processing_interval = 0.3  # segundos
        self.last_processed_time = time.time()

        self.publish_status("Nodo Cámara con MediaPipe iniciado.")

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_processed_time < self.processing_interval:
            return
        self.last_processed_time = current_time

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            results = self.pose.process(image_rgb)

            keypoints = Float32MultiArray()
            annotated_image = frame.copy()

            if results.pose_landmarks:
                valid_keypoints = 0
                for landmark in results.pose_landmarks.landmark:
                    keypoints.data.extend([landmark.x, landmark.y, landmark.z, landmark.visibility])
                    if landmark.visibility > 0.5:
                        valid_keypoints += 1

                # Publicar keypoints
                self.keypoints_pub.publish(keypoints)
                self.get_logger().info("Keypoints publicados.")

                # Publicar detección visual si hay suficientes keypoints visibles
                person_detected = valid_keypoints >= 3
                self.visual_detection_pub.publish(Bool(data=person_detected))
                self.get_logger().info(f"Detección visual publicada: {'Sí' if person_detected else 'No'}")

                # ---------------------------
                # Reconocimiento de gestos
                # ---------------------------
                # Se hará una detección en base a la mano derecha utilizando algunos landmarks clave:
                # RIGHT_WRIST, RIGHT_THUMB, RIGHT_INDEX y RIGHT_PINKY.
                gesture = None
                try:
                    # Utilizamos las constantes de la enumeración para mayor claridad.
                    right_wrist = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST]
                    right_thumb = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_THUMB]
                    right_index = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_INDEX]
                    right_pinky = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_PINKY]

                    # Solo procesamos si todos tienen buena visibilidad.
                    if (right_wrist.visibility > 0.5 and right_thumb.visibility > 0.5 and 
                        right_index.visibility > 0.5 and right_pinky.visibility > 0.5):

                        # Definición de un umbral para diferenciar: (0.05 es un valor experimental)
                        threshold = 0.05
                        # "Pulgar arriba" para iniciar seguimiento:
                        # Se interpreta si el pulgar está arriba del muñeca y el índice y el meñique se encuentran por debajo.
                        if ((right_thumb.y < right_wrist.y - threshold) and 
                            (right_index.y > right_wrist.y) and 
                            (right_pinky.y > right_wrist.y)):
                            gesture = "start_tracking"
                        # "Mano abierta" para detener seguimiento:
                        # Se interpreta si el pulgar, el índice y el meñique están todos elevados (más arriba que el muñeca).
                        elif ((right_thumb.y < right_wrist.y - threshold) and 
                              (right_index.y < right_wrist.y - threshold) and 
                              (right_pinky.y < right_wrist.y - threshold)):
                            gesture = "stop_tracking"

                except Exception as ge:
                    self.get_logger().error(f"Error detectando gesto: {ge}")

                if gesture:
                    gesture_msg = String()
                    gesture_msg.data = gesture
                    self.gesture_pub.publish(gesture_msg)
                    self.get_logger().info(f"Gesto detectado: {gesture}")

                # ---------------------------
                # Dibujar landmarks para visualización
                if self.visualize:
                    self.mp_drawing.draw_landmarks(
                        annotated_image,
                        results.pose_landmarks,
                        self.mp_pose.POSE_CONNECTIONS,
                        landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                        connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2)
                    )

            else:
                # Si no hay landmarks, publicamos False en detección visual
                self.visual_detection_pub.publish(Bool(data=False))

            if self.visualize:
                cv2.imshow("Salida con pose estimada", annotated_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error en procesamiento de imagen: {e}")

    def publish_status(self, message):
        self.status_pub.publish(String(data=message))

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Cierre del sistema detectado. Enviando confirmación.")
            try:
                self.shutdown_confirmation_pub.publish(Bool(data=True))
            except Exception as e:
                self.get_logger().error(f"Error al publicar confirmación de apagado: {e}")
            finally:
                cv2.destroyAllWindows()
                self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo Cámara detenido manualmente.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
