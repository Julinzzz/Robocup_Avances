import cv2
import numpy as np
import os
import pickle
from sklearn.metrics.pairwise import euclidean_distances

# ————————————————
# 1. Configuración
# ————————————————
FACE_DB            = "rostros_db.pkl"
UMBRAL_DISTANCIA   = 0.9    # distancia L2 máxima para considerar mismo rostro
MIN_CONF_DETECT    = 0.9    # confianza mínima detección

# ————————————————
# 2. Modelos preentrenados
# ————————————————
DETECTOR = cv2.dnn.readNetFromCaffe(
    "deploy.prototxt",
    "res10_300x300_ssd_iter_140000.caffemodel"
)
EMBEDDER = cv2.dnn.readNetFromTorch("nn4.small2.v1.t7")  # FaceNet

# ————————————————
# 3. Carga o inicializa la base de datos
# ————————————————
if os.path.exists(FACE_DB):
    os.remove(FACE_DB)
db = {
    "ids": [],             # [1, 2, 3, …]
    "mean_embeddings": [], # [[…], […], …]
    "counts": [],          # [n1, n2, …]
    "nombres": []          # ["Persona 1", …]
}
id_counter = 1

# ————————————————
# 4. Función para obtener embedding normalizado
# ————————————————
def obtener_embedding(rostro):
    rostro = cv2.resize(rostro, (96, 96))
    blob = cv2.dnn.blobFromImage(
        rostro, 1.0/255, (96, 96),
        (0, 0, 0), swapRB=True, crop=False
    )
    EMBEDDER.setInput(blob)
    emb = EMBEDDER.forward().flatten()
    # normalizar a norma 1
    return emb / np.linalg.norm(emb)

# ————————————————
# 5. Asignación de ID usando distancia L2 a vectores medios
# ————————————————
def asignar_id(embedding):
    if not db["mean_embeddings"]:
        return None
    # calcula distancias L2 a cada media
    dist = euclidean_distances([embedding], db["mean_embeddings"])[0]
    idx_min = np.argmin(dist)
    if dist[idx_min] < UMBRAL_DISTANCIA:
        return db["ids"][idx_min]
    return None

# ————————————————
# 6. Actualiza media incremental
# ————————————————
def actualizar_media(idx, emb):
    n_old = db["counts"][idx]
    mu_old = db["mean_embeddings"][idx]
    mu_new = (n_old * mu_old + emb) / (n_old + 1)
    # renormalizar para conservar norma 1
    mu_new = mu_new / np.linalg.norm(mu_new)
    db["mean_embeddings"][idx] = mu_new
    db["counts"][idx] += 1

# ————————————————
# 7. Loop principal de video
# ————————————————
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # detectar rostros
    blob = cv2.dnn.blobFromImage(
        cv2.resize(frame, (300, 300)), 1.0,
        (300, 300), (104.0, 177.0, 123.0)
    )
    DETECTOR.setInput(blob)
    detecciones = DETECTOR.forward()

    for i in range(detecciones.shape[2]):
        conf = float(detecciones[0, 0, i, 2])
        if conf < MIN_CONF_DETECT:
            continue

        # obtener bounding box
        box = (detecciones[0, 0, i, 3:7] *
               np.array([frame.shape[1], frame.shape[0],
                         frame.shape[1], frame.shape[0]]))
        x1, y1, x2, y2 = box.astype("int")
        w, h = x2 - x1, y2 - y1
        if w < 50 or h < 50:
            continue

        rostro = frame[y1:y1+h, x1:x1+w]
        emb = obtener_embedding(rostro)

        # intento de asignar ID
        id_rostro = asignar_id(emb)
        if id_rostro is not None:
            # coincidencia: actualiza la media
            idx = db["ids"].index(id_rostro)
            actualizar_media(idx, emb)
        else:
            # nuevo rostro: crea nueva entrada
            id_rostro = id_counter
            db["ids"].append(id_rostro)
            db["mean_embeddings"].append(emb)
            db["counts"].append(1)
            db["nombres"].append(f"Persona {id_rostro}")
            id_counter += 1

        # dibuja rectángulo y etiqueta
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"ID {id_rostro}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)

        # guarda la base de datos tras cada nuevo añadido
        if id_rostro == id_counter - 1:
            with open(FACE_DB, "wb") as f:
                pickle.dump(db, f)

    cv2.imshow("Reconocimiento Facial Mejorado", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
