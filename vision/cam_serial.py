import yaml
from ultralytics import YOLO
import cv2, serial, time

# ========================
# CONFIGURAÇÕES
# ========================
MODEL_PATH = "runs/detect/train2/weights/best.pt"
DATASET_YAML = "DATA-SET-23-9/data.yaml"
PORTA_ARDUINO = "COM11"
BAUD_RATE = 115200
CONFIANCA_MIN = 0.90
# ========================

# Carregar labels
with open(DATASET_YAML, "r") as f:
    data = yaml.safe_load(f)
labels = data["names"]

print("\nClasses disponíveis:")
for i, label in enumerate(labels):
    print(f"{i}: {label}")

# Escolher 1 classe
choice = int(input("\nDigite o número da classe que deseja monitorar: "))
classe_escolhida = labels[choice]
print(f"\nClasse selecionada: {classe_escolhida}")

# Conectar ao Arduino
arduino = serial.Serial(PORTA_ARDUINO, BAUD_RATE, timeout=1)
time.sleep(2)

# Carregar modelo
model = YOLO(MODEL_PATH)

# Abrir câmera
cap = cv2.VideoCapture(0)

# Forçar resolução máxima (Full HD 1920x1080)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, imgsz=1280)  # pode ajustar imgsz (640, 720, 1080, 1280)

    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            if cls == choice and conf >= CONFIANCA_MIN:
                x, y, w, h = box.xywh[0]  # centro da caixa
                x, y = int(x), int(y)

                msg = f"{x},{y}\n"
                print("Enviando:", msg.strip())
                arduino.write(msg.encode())

    annotated = results[0].plot()
    cv2.imshow("Detecção em tempo real", annotated)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC para sair
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
