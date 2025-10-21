#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import pickle
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from sklearn.metrics.pairwise import euclidean_distances
from ament_index_python.packages import get_package_share_directory

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub_id = self.create_publisher(Int32, '/face_id', 10)
        self.init_db()
        self.init_models()

    def init_db(self):
        # ruta al fichero
        self.db_path = os.path.expanduser('~/.face_rec_db.pkl')
        # si existe, lo borramos para empezar limpio
        if os.path.exists(self.db_path):
            os.remove(self.db_path)
        # siempre creamos una db vacía
        self.db = {
            "ids": [],             # [1, 2, 3, …]
            "mean_embeddings": [], # [[…], […], …]
            "counts": [],          # [n1, n2, …]
            "nombres": []          # ["Persona 1", …]
        }
        # arrancamos el contador en 1
        self.id_counter = 1

    def init_models(self):
        # -- Obtén el directorio share de este paquete --
        pkg_share = get_package_share_directory('face_recognition')
        models_dir = os.path.join(pkg_share, 'models')

        # -- Rutas absolutas a cada archivo del modelo --
        prototxt_path  = os.path.join(models_dir, 'deploy.prototxt')
        caffemodel_path= os.path.join(models_dir, 'res10_300x300_ssd_iter_140000.caffemodel')
        t7model_path   = os.path.join(models_dir, 'nn4.small2.v1.t7')

        # -- Carga los modelos --
        self.detector = cv2.dnn.readNetFromCaffe(prototxt_path, caffemodel_path)
        self.embedder = cv2.dnn.readNetFromTorch(t7model_path)

        self.UMBRAL = 0.9
        self.MIN_CONF = 0.9

    def obtener_embedding(self, rostro):
        r = cv2.resize(rostro, (96, 96))
        blob = cv2.dnn.blobFromImage(r, 1/255, (96, 96), (0,0,0), swapRB=True)
        self.embedder.setInput(blob)
        emb = self.embedder.forward().flatten()
        return emb / np.linalg.norm(emb)

    def asignar_id(self, emb):
        if not self.db['mean_embeddings']:
            return None
        dist = euclidean_distances([emb], self.db['mean_embeddings'])[0]
        idx = np.argmin(dist)
        return self.db['ids'][idx] if dist[idx] < self.UMBRAL else None

    def actualizar_media(self, idx, emb):
        n = self.db['counts'][idx]
        mu = self.db['mean_embeddings'][idx]
        mu_new = (n * mu + emb) / (n + 1)
        self.db['mean_embeddings'][idx] = mu_new / np.linalg.norm(mu_new)
        self.db['counts'][idx] += 1

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300,300)), 1.0,
                                     (300,300), (104,177,123))
        self.detector.setInput(blob)
        detections = self.detector.forward()

        for i in range(detections.shape[2]):
            conf = float(detections[0,0,i,2])
            if conf < self.MIN_CONF:
                continue

            box = detections[0,0,i,3:7] * np.array([w,h,w,h])
            x1, y1, x2, y2 = box.astype(int)
            if (x2 - x1) < 50 or (y2 - y1) < 50:
                continue

            rostro = frame[y1:y2, x1:x2]
            emb = self.obtener_embedding(rostro)
            face_id = self.asignar_id(emb)

            if face_id is None:
                face_id = self.id_counter
                self.db['ids'].append(face_id)
                self.db['mean_embeddings'].append(emb)
                self.db['counts'].append(1)
                self.db['nombres'].append(f'Persona {face_id}')
                self.id_counter += 1
                with open(self.db_path, 'wb') as f:
                    pickle.dump(self.db, f)
            else:
                idx = self.db['ids'].index(face_id)
                self.actualizar_media(idx, emb)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f'ID {face_id}', (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
            self.pub_id.publish(Int32(data=face_id))

        cv2.imshow('face_recognition', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()