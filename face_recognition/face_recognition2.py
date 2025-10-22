import sys
import os
import cv2
import torch
import numpy as np
from PIL import Image
from torchvision import transforms

# 1) Añade al PYTHONPATH la carpeta swinface_project
SWIN_ROOT   = os.path.expanduser('~/Downloads/SwinFace')
PROJECT_DIR = os.path.join(SWIN_ROOT, 'swinface_project')
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

# 2) Importa la configuración y el constructor de modelo desde el repo
from configs.config_pretrain import get_config
from model                import build_swinface

# 3) Configuración de dispositivo y ruta al checkpoint
DEVICE    = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
CKPT_PATH = os.path.join(SWIN_ROOT, 'checkpoints', 'checkpoint_step__79999_gpu_0.pt')
IMG_SIZE  = 224  # tamaño de entrada del modelo

# 4) Índices de atributos en el vector de 40 de CelebA
ATTR_IDX = {
    'glasses':    15,
    'male':       20,
    'hair_color': {'blond': 8, 'brown': 9, 'black': 11},
    'hair_type':  {'straight': 5, 'wavy': 3}
}

# 5) Carga el modelo SwinFace con el checkpoint
def load_model(ckpt_path):
    cfg   = get_config()
    model = build_swinface(cfg)
    checkpoint = torch.load(ckpt_path, map_location='cpu')
    model.load_state_dict(checkpoint['model'], strict=False)
    model.to(DEVICE).eval()
    return model

# 6) Transformación estándar ImageNet + recorte central
transform = transforms.Compose([
    transforms.CenterCrop(IMG_SIZE),
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406],
                         [0.229, 0.224, 0.225]),
])

# 7) Función para extraer los 4 atributos de interés
def analyze_face(img_bgr, model):
    # OpenCV BGR → PIL RGB
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(img_rgb)
    x = transform(pil_img).unsqueeze(0).to(DEVICE)

    with torch.no_grad():
        outputs = model(x)
        logits  = outputs['attr'][0]            # tensor [40]
        probs   = torch.sigmoid(logits).cpu().numpy()

    # 7.1) Gafas y género
    has_glasses = probs[ATTR_IDX['glasses']] > 0.5
    is_male     = probs[ATTR_IDX['male']]     > 0.5

    # 7.2) Color de pelo: elige la clase con probabilidad máxima
    hc_map = ATTR_IDX['hair_color']
    hair_color = max(hc_map.keys(), key=lambda k: probs[hc_map[k]])

    # 7.3) Tipo de pelo: straight vs wavy
    ht_map = ATTR_IDX['hair_type']
    hair_type = max(ht_map.keys(), key=lambda k: probs[ht_map[k]])

    return {
        'glasses':    bool(has_glasses),
        'gender':     'male' if is_male else 'female',
        'hair_color': hair_color,
        'hair_type':  hair_type,
        'probs':      probs  # si quieres ver todas las probabilidades
    }

# 8) Demo en webcam
def main():
    # 8.1) Carga el modelo
    print("Cargando modelo SwinFace...")
    model = load_model(CKPT_PATH)
    print("Modelo cargado en", DEVICE)

    # 8.2) Inicializa la webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("No se pudo abrir la cámara")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 8.3) Analiza la imagen completa como rostro
        attrs = analyze_face(frame, model)

        # 8.4) Dibuja los resultados en pantalla
        text = (f"{attrs['gender']}, {attrs['hair_color']}, "
                f"{attrs['hair_type']}, "
                f"{'glasses' if attrs['glasses'] else 'no glasses'}")
        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("SwinFace Attributes", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()