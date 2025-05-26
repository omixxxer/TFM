import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String, Bool
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import time

class VisualDetectionNode(Node):
    def __init__(self):
        super().__init__('visual_detection_node')

        # Parámetros configurables
        self.declare_parameter('enabled', True)
        self.declare_parameter('visualize', True)
        self.declare_parameter('processing_interval', 1.0)              # segundos entre procesados
        self.declare_parameter('min_pose_landmarks', 3)                # mínimos landmarks visibles
        self.declare_parameter('gesture_threshold', 0.05)              # umbral relativo para gesto

        # Parámetros MediaPipe Pose
        self.declare_parameter('pose_static_image_mode', False)
        self.declare_parameter('pose_model_complexity', 1)
        self.declare_parameter('pose_enable_segmentation', False)
        self.declare_parameter('pose_min_detection_confidence', 0.5)
        self.declare_parameter('pose_min_tracking_confidence', 0.5)

        # Parámetros MediaPipe Hands
        self.declare_parameter('hands_static_image_mode', False)
        self.declare_parameter('hands_max_num_hands', 1)
        self.declare_parameter('hands_model_complexity', 1)
        self.declare_parameter('hands_min_detection_confidence', 0.5)
        self.declare_parameter('hands_min_tracking_confidence', 0.5)

        # Cargar parámetros
        self.enabled = self.get_parameter('enabled').value
        self.visualize = self.get_parameter('visualize').value
        self.processing_interval = self.get_parameter('processing_interval').value
        self.min_pose_landmarks = self.get_parameter('min_pose_landmarks').value
        self.gesture_threshold = self.get_parameter('gesture_threshold').value
        # Cargar MediaPipe Pose
        pose_static = self.get_parameter('pose_static_image_mode').value
        pose_complexity = self.get_parameter('pose_model_complexity').value
        pose_segmentation = self.get_parameter('pose_enable_segmentation').value
        pose_min_det = self.get_parameter('pose_min_detection_confidence').value
        pose_min_track = self.get_parameter('pose_min_tracking_confidence').value
        # Cargar MediaPipe Hands
        hands_static = self.get_parameter('hands_static_image_mode').value
        hands_max = self.get_parameter('hands_max_num_hands').value
        hands_complexity = self.get_parameter('hands_model_complexity').value
        hands_min_det = self.get_parameter('hands_min_detection_confidence').value
        hands_min_track = self.get_parameter('hands_min_tracking_confidence').value

        if not self.enabled:
            self.get_logger().info("Nodo de Cámara desactivado.")
            return

        self.get_logger().info("Inicializando nodo de Cámara")

        # Variables de estado
        self.bridge = CvBridge()
        self.last_processed_time = time.time()

        # Publicadores y suscripciones
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )
        self.keypoints_pub = self.create_publisher(
            Float32MultiArray,
            '/pose/keypoints',
            10
        )
        self.visual_detection_pub = self.create_publisher(
            Bool,
            '/person_detected_visual',
            10
        )
        self.gesture_pub = self.create_publisher(
            String,
            '/gesture_command',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/camera/status',
            10
        )
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_processed',
            10
        )
        self.shutdown_sub = self.create_subscription(
            Bool,
            '/system_shutdown',
            self.shutdown_callback,
            10
        )
        self.shutdown_confirmation_pub = self.create_publisher(
            Bool,
            '/shutdown_confirmation',
            10
        )

        # Inicializar MediaPipe Pose y Hands con parámetros
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=pose_static,
            model_complexity=pose_complexity,
            enable_segmentation=pose_segmentation,
            min_detection_confidence=pose_min_det,
            min_tracking_confidence=pose_min_track
        )
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=hands_static,
            max_num_hands=hands_max,
            model_complexity=hands_complexity,
            min_detection_confidence=hands_min_det,
            min_tracking_confidence=hands_min_track
        )
        self.mp_drawing = mp.solutions.drawing_utils

        self.publish_status("Nodo Cámara con MediaPipe iniciado.")

    def image_callback(self, msg):
        # Controlar frecuencia de procesamiento
        current_time = time.time()
        if current_time - self.last_processed_time < self.processing_interval:
            return
        self.last_processed_time = current_time

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            annotated_image = frame.copy()

            # 1) Procesar Pose
            results_pose = self.pose.process(image_rgb)
            pose_landmarks = results_pose.pose_landmarks
            visible_count = 0
            if pose_landmarks:
                for lm in pose_landmarks.landmark:
                    if lm.visibility > 0.5:
                        visible_count += 1
                # Publicar keypoints
                keypoints_msg = Float32MultiArray()
                for lm in pose_landmarks.landmark:
                    keypoints_msg.data.extend([lm.x, lm.y, lm.z, lm.visibility])
                self.keypoints_pub.publish(keypoints_msg)

                # Dibujar landmarks si corresponde
                if self.visualize:
                    self.mp_drawing.draw_landmarks(
                        annotated_image,
                        pose_landmarks,
                        self.mp_pose.POSE_CONNECTIONS
                    )

            person_pose = (visible_count >= self.min_pose_landmarks)

            # 2) Procesar Hands/Gestos
            gesture = None
            results_hands = self.hands.process(image_rgb)
            hand_detected = False
            if results_hands.multi_hand_landmarks:
                hand_detected = True
                for hand_landmarks in results_hands.multi_hand_landmarks:
                    gesture = self.detect_hand_gesture(hand_landmarks)
                    if gesture:
                        self.gesture_pub.publish(String(data=gesture))
                        self.get_logger().info(f"Gesto detectado: {gesture}")
                    if self.visualize:
                        self.mp_drawing.draw_landmarks(
                            annotated_image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS
                        )

            # 3) Publicar detección combinada (pose o gesto)
            combined_detection = person_pose or hand_detected
            self.visual_detection_pub.publish(Bool(data=combined_detection))

            # 4) Publicar imagen procesada
            if self.visualize:
                image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
                self.image_pub.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f"Error en procesamiento de imagen: {e}")

    def detect_hand_gesture(self, hand_landmarks):
        try:
            wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
            thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
            pinky_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]

            th = self.gesture_threshold
            gesture = None
            if (thumb_tip.y < wrist.y - th and
                index_tip.y > wrist.y and
                pinky_tip.y > wrist.y):
                gesture = "start_tracking"
            elif (thumb_tip.y < wrist.y - th and
                  index_tip.y < wrist.y - th and
                  pinky_tip.y < wrist.y - th):
                gesture = "stop_tracking"
            return gesture
        except Exception as e:
            self.get_logger().error(f"Error en reconocimiento de gesto: {e}")
            return None

    def publish_status(self, message):
        self.status_pub.publish(String(data=message))

    def shutdown_callback(self, msg):
        if msg.data:
            self.get_logger().info("Cierre del sistema detectado. Enviando confirmación.")
            self.shutdown_confirmation_pub.publish(Bool(data=True))
            cv2.destroyAllWindows()
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo Cámara detenido manualmente.")
    finally:
        # Cerramos ventanas de OpenCV
        import cv2
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == '__main__':
    main()
