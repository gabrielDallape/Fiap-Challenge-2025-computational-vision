import yaml
from ultralytics import YOLO
import cv2, serial, time

# ========================
# CONFIGURAÇÕES
# ========================
MODEL_PATH = "runs/detect/train2/weights/best.pt"
DATASET_YAML = "DATA-SET-23-9/data.yaml"
PORTA_ARDUINO = "COM11"   # troque se necessário
BAUD_RATE = 115200
CONFIANCA_MIN = 0.90     # confiança mínima
COOLDOWN = 3             # segundos entre envios
# ========================

# Carregar labels do dataset
with open(DATASET_YAML, "r") as f:
    data = yaml.safe_load(f)
labels = data["names"]

print("\nClasses disponíveis:")
for i, label in enumerate(labels):
    print(f"{i}: {label}")

# Escolher classes
choices = input("\nDigite os números das classes que deseja monitorar (ex: 0,2): ")
choices = [int(x.strip()) for x in choices.split(",") if x.strip().isdigit()]

# Mapear classes escolhidas em A, B, C...
selected = {cls: chr(65+idx) for idx, cls in enumerate(choices)}
print("\nClasses selecionadas:")
for cls, letter in selected.items():
    print(f"{labels[cls]} -> {letter}")

# Conectar Arduino
arduino = serial.Serial(PORTA_ARDUINO, BAUD_RATE, timeout=1)
time.sleep(2)  # aguarda inicialização

# Carregar modelo
model = YOLO(MODEL_PATH)

# Abrir câmera
cap = cv2.VideoCapture(0)
last_sent = {cls: 0 for cls in choices}

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)

    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            if cls in selected and conf >= CONFIANCA_MIN:
                x, y, w, h = box.xywh[0]
                x, y = int(x), int(y)  # centro em pixels

                now = time.time()
                if now - last_sent[cls] > COOLDOWN:
                    msg = f"{x},{y} {selected[cls]}\n"
                    print("Enviando:", msg.strip())
                    arduino.write(msg.encode())
                    last_sent[cls] = now

    annotated = results[0].plot()
    cv2.imshow("Detecção em tempo real", annotated)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC para sair
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
