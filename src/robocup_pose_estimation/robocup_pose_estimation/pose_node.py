#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        
        # MediaPipe Setup
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=2,
            enable_segmentation=False,
            min_detection_confidence=0.5)
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.bridge = CvBridge()
        
        # Publishers
        self.left_angle_pub = self.create_publisher(Float32, '/angles/left_arm', 10)
        self.right_angle_pub = self.create_publisher(Float32, '/angles/right_arm', 10)

        self.PROJECTION_THRESHOLD = 20  # Grados mínimos para activar proyección
        self.PROJECTION_COLOR_LEFT = (255, 0, 0)  # Azul para brazo izquierdo
        self.PROJECTION_COLOR_RIGHT = (0, 0, 255) # Rojo para brazo derecho
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.get_logger().info('Pose Estimator Node iniciado')

    def calculate_angle(self, a, b, c):
        """Calcula el ángulo entre tres puntos en el espacio 3D"""
        a = np.array([a.x, a.y, a.z]) if hasattr(a, 'x') else np.array(a)
        b = np.array([b.x, b.y, b.z]) if hasattr(b, 'x') else np.array(b)
        c = np.array([c.x, c.y, c.z]) if hasattr(c, 'x') else np.array(c)
        
        ba = a - b
        bc = c - b
        
        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)
        
        return np.degrees(angle)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            results = self.pose.process(image_rgb)
            
            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                # Obtener landmarks clave
                left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
                right_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]
                left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
                right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
                left_elbow = landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW]
                right_elbow = landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW]
                
                # Publicar coordenadas de caderas
                left_hip_msg = Point()
                left_hip_msg.x = left_hip.x
                left_hip_msg.y = left_hip.y
                left_hip_msg.z = left_hip.z
                
                right_hip_msg = Point()
                right_hip_msg.x = right_hip.x
                right_hip_msg.y = right_hip.y
                right_hip_msg.z = right_hip.z
                left_wrist = landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST]
                right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST]

                # Calcular y publicar ángulos
                left_angle = self.calculate_angle(left_hip, left_shoulder, left_elbow)
                right_angle = self.calculate_angle(right_hip, right_shoulder, right_elbow)
                
                self.left_angle_pub.publish(Float32(data=float(left_angle)))
                self.right_angle_pub.publish(Float32(data=float(right_angle)))

                if left_angle > self.PROJECTION_THRESHOLD:
                    self.draw_3d_projection(cv_image, left_elbow, left_wrist, self.PROJECTION_COLOR_LEFT)
        
                if right_angle > self.PROJECTION_THRESHOLD:
                    self.draw_3d_projection(cv_image, right_elbow, right_wrist, self.PROJECTION_COLOR_RIGHT)
                
                # Resto del código de visualización...
                self.mp_drawing.draw_landmarks(
                    cv_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
                
                # Mostrar ángulos en imagen
                cv2.putText(cv_image, f"L: {left_angle:.1f}°", (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                cv2.putText(cv_image, f"R: {right_angle:.1f}°", (50, 100), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                
                cv2.imshow('Pose Estimation', cv_image)
                cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')


    def draw_3d_projection(self, image, elbow, wrist, color):
        """Dibuja proyección con longitud inversamente proporcional a la profundidad"""
        height, width = image.shape[:2]
        
        # Convertir coordenadas normalizadas a píxeles
        elbow_x = int(elbow.x * width)
        elbow_y = int(elbow.y * height)
        wrist_x = int(wrist.x * width)
        wrist_y = int(wrist.y * height)
        
        # Configuración dinámica (puedes mover estos parámetros al __init__)
        MIN_LENGTH = 50    # Longitud mínima (cuando el brazo está muy cerca)
        MAX_LENGTH = 3000   # Longitud máxima (cuando el brazo está lejos)
        BASE_SCALE = 0.8   # Escala relativa al tamaño de la imagen (0.0 a 1.0)
        
        # Calcular dirección del vector
        dir_x = wrist_x - elbow_x
        dir_y = wrist_y - elbow_y
        angle = np.arctan2(dir_y, dir_x)
        
        # Ajustar longitud basada en profundidad (z)
        # Invertimos la relación: z pequeño (lejos) = mayor longitud
        depth_factor = max(0.1, min(1.0, elbow.z))  # Aseguramos rango 0.1-1.0
        dynamic_length = int(MIN_LENGTH + (MAX_LENGTH - MIN_LENGTH) * depth_factor)
        
        # Calcular punto final (asegurando que no salga de la imagen)
        end_x = int(wrist_x + dynamic_length * np.cos(angle))
        end_y = int(wrist_y + dynamic_length * np.sin(angle))
        end_x = max(0, min(width-1, end_x))
        end_y = max(0, min(height-1, end_y))
        
        # Grosor y tamaño dinámicos
        thickness = int(2 + 3 * (1 - depth_factor))  # Más delgado cuando está lejos
        pointer_size = int(10 + 10 * depth_factor)   # Más pequeño cuando está lejos
        
        # Dibujar línea de proyección con anti-aliasing
        cv2.arrowedLine(
            image,
            (wrist_x, wrist_y),
            (end_x, end_y),
            color,
            thickness=thickness,
            tipLength=0.15 + 0.1 * depth_factor,  # Punta más grande cuando está cerca
            line_type=cv2.LINE_AA
        )
        
        # Dibujar puntero doble (círculo + cruz)
        cv2.circle(image, (end_x, end_y), pointer_size, color, 2)
        cv2.line(
            image,
            (end_x - pointer_size, end_y),
            (end_x + pointer_size, end_y),
            color,
            2,
            cv2.LINE_AA
        )
        cv2.line(
            image,
            (end_x, end_y - pointer_size),
            (end_x, end_y + pointer_size),
            color,
            2,
            cv2.LINE_AA
        )
        
        # Opcional: Mostrar texto con la distancia
        if depth_factor > 0.3:  # Solo mostrar si no está muy lejos
            cv2.putText(
                image,
                f"{dynamic_length}px",
                (end_x + 15, end_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA
            )

def main(args=None):
    rclpy.init(args=args)
    pose_estimator = PoseEstimator()
    rclpy.spin(pose_estimator)
    pose_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()