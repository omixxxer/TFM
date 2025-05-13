import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String, Bool
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import time

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('enabled', True)
        self.declare_parameter('visualize', True)
        self.enabled = self.get_parameter('enabled').value
        self.visualize = self.get_parameter('visualize').value

        if not self.enabled:
            self.get_logger().info("Nodo de Cámara desactivado.")
            return

        self.get_logger().info("Inicializando nodo de Cámara")

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile_sensor_data)

        self.keypoints_pub = self.create_publisher(Float32MultiArray, '/pose/keypoints', 10)
        self.status_pub = self.create_publisher(String, '/camera/status', 10)
        self.shutdown_sub = self.create_subscription(Bool, '/system_shutdown', self.shutdown_callback, 10)
        self.visual_detection_pub = self.create_publisher(Bool, '/person_detected_visual', 10)
        self.shutdown_confirmation_pub = self.create_publisher(Bool, '/shutdown_confirmation', 10)
        self.gesture_pub = self.create_publisher(String, '/gesture_command', 10)

        # Publisher de imagen para RViz2
        self.image_pub = self.create_publisher(Image, '/camera/image_processed', 10)

        # Inicializar MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False,
                                      model_complexity=1,
                                      enable_segmentation=False,
                                      min_detection_confidence=0.5,
                                      min_tracking_confidence=0.5)

        # Inicializar MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False,
                                         max_num_hands=1,
                                         model_complexity=1,
                                         min_detection_confidence=0.5,
                                         min_tracking_confidence=0.5)

        self.mp_drawing = mp.solutions.drawing_utils

        self.processing_interval = 1.0
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
            annotated_image = frame.copy()

            # Procesar pose (cuerpo)
            results_pose = self.pose.process(image_rgb)
            person_detected = False
            if results_pose.pose_landmarks:
                keypoints = Float32MultiArray()
                valid_keypoints = 0
                for landmark in results_pose.pose_landmarks.landmark:
                    keypoints.data.extend([landmark.x, landmark.y, landmark.z, landmark.visibility])
                    if landmark.visibility > 0.5:
                        valid_keypoints += 1

                self.keypoints_pub.publish(keypoints)
                person_detected = valid_keypoints >= 3

                if self.visualize:
                    self.mp_drawing.draw_landmarks(
                        annotated_image,
                        results_pose.pose_landmarks,
                        self.mp_pose.POSE_CONNECTIONS,
                        landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                        connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2)
                    )

            self.visual_detection_pub.publish(Bool(data=person_detected))

            # Procesar hands (gestos)
            results_hands = self.hands.process(image_rgb)
            if results_hands.multi_hand_landmarks:
                for hand_landmarks in results_hands.multi_hand_landmarks:
                    if self.visualize:
                        self.mp_drawing.draw_landmarks(
                            annotated_image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS)

                    gesture = self.detect_hand_gesture(hand_landmarks)
                    if gesture:
                        self.gesture_pub.publish(String(data=gesture))
                        self.get_logger().info(f"Gesto detectado: {gesture}")

            # Publicar imagen procesada a RViz2
            if self.visualize:
                image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
                self.image_pub.publish(image_msg)
                #cv2.imshow("Pose + Hands", annotated_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error en procesamiento de imagen: {e}")

    def detect_hand_gesture(self, hand_landmarks):
        try:
            wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
            thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
            pinky_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]

            threshold = 0.05
            gesture = None

            if (thumb_tip.y < wrist.y - threshold and
                index_tip.y > wrist.y and
                pinky_tip.y > wrist.y):
                gesture = "start_tracking"
            elif (thumb_tip.y < wrist.y - threshold and
                  index_tip.y < wrist.y - threshold and
                  pinky_tip.y < wrist.y - threshold):
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
