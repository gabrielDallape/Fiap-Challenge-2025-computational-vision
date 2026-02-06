import cv2
import serial
import time
import os
import numpy as np
from ultralytics import YOLO
from serial.tools import list_ports
from collections import deque

# =========================
# CONFIG
# =========================
MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "models", "best.pt")

# Serial
ARDUINO_PORT = "auto"
BAUD_RATE = 115200
BAUD_CANDIDATES = [BAUD_RATE, 57600, 38400, 19200, 9600]
SERIAL_LINE_ENDING = "\n"
GLOBAL_TX_MIN_INTERVAL_MS = 1000
LIMIT_ONE_LINE_PER_LOOP = True
AVOID_DTR_TOGGLE = True

# Camera
CAMERA_INDEX = 1
TRY_INDEXES = [CAMERA_INDEX, 0, 2, 3, 4]

# YOLO
YOLO_IMG_SIZE = 960
YOLO_CONF_GLOBAL = 0.03
YOLO_IOU = 0.45

# Cooldowns
COOLDOWN_MS_CLASS = 6000

# Scales / mapping
G_INVERT = True
G_SCALE_MAX = 9000
B_SCALE_MAX = 9000
DELAY_MAX_MS = 60000

DYN_PULL_MAX = 250

# Live tuning - VALORES PADRÃO GLOBAIS
G_OFFSET_DEFAULT = -340
G_OFFSET_STEP = 10
G_OFFSET_MIN = -2000
G_OFFSET_MAX = 2000

DELAY_FACTOR_DEFAULT = 0.200
DF_STEP = 0.005
DELAY_FACTOR_MIN = 0.000
DELAY_FACTOR_MAX = 100.000

VELOCITY_DEFAULT = 100
VELOCITY_MIN = 0
VELOCITY_MAX = 999
VELOCITY_STEP = 1

# ===== PRESETS POR CLASSE =====
# Configure valores específicos para cada classe
# Classes não listadas usarão os valores DEFAULT acima

# Confiança mínima por classe
DEFAULT_MIN_CONF = 0.15
CLASS_MIN_CONF = {
    "DUCOCO": 0.55,
    "coca_cola": 0.15,
    "creme_nestle": 0.50,
    "extrato_turma_monica": 0.05,
    "garrafa_agua": 1.95,
    "margarina_dorianna": 0.75,
    "pasta_colgate": 0.50,
    "sabonete_dove": 0.55,
    "sardinha_coqueiro": 0.55,
    "todinho": 0.65,
}

# G_OFFSET por classe (ajuste fino do posicionamento)
CLASS_G_OFFSET = {
    "DUCOCO": 400,
    "coca_cola": 0,
    "creme_nestle": 380,
    "extrato_turma_monica": 0,
    "garrafa_agua": 0,
    "margarina_dorianna": 0,
    "pasta_colgate": 480,
    "sabonete_dove": 320,
    "sardinha_coqueiro": 560,
    "todinho": 440,
}

# DELAY_FACTOR por classe (controla timing)
CLASS_DELAY_FACTOR = {
    "DUCOCO": 0.217,
    "coca_cola": 0.200,
    "creme_nestle": 0.202,
    "extrato_turma_monica": 0.200,
    "garrafa_agua": 0.200,
    "margarina_dorianna": 0.200,
    "pasta_colgate": 0.190,
    "sabonete_dove": 0.200,
    "sardinha_coqueiro": 0.220,
    "todinho": 0.210,
}

# VELOCITY por classe (velocidade de movimento)
CLASS_VELOCITY = {
    "DUCOCO": 150,
    "coca_cola": 150,
    "creme_nestle": 150,
    "extrato_turma_monica": 150,
    "garrafa_agua": 150,
    "margarina_dorianna": 150,
    "pasta_colgate": 150,
    "sabonete_dove": 150,
    "sardinha_coqueiro": 150,
    "todinho": 150,
}

# =========================
# CLASSES & SERVO
# =========================
CLASSES = [
    "DUCOCO",
    "coca_cola",
    "creme_nestle",
    "extrato_turma_monica",
    "garrafa_agua",
    "margarina_dorianna",
    "pasta_colgate",
    "sabonete_dove",
    "sardinha_coqueiro",
    "todinho"
]

SERVO_MIN = 100
SERVO_MAX = 150
SERVO_DEFAULT = 120
SERVO_DEG_PER_CLASS = {
    "DUCOCO": 135,
    "coca_cola": 103,
    "creme_nestle": 137,
    "extrato_turma_monica": 120,
    "garrafa_agua": 110,
    "margarina_dorianna": 130,
    "pasta_colgate": 140,
    "sabonete_dove": 145,
    "sardinha_coqueiro": 150,
    "todinho": 130,
}

def clamp(v, a, b): return max(a, min(b, v))
def servo_for(label): return int(clamp(SERVO_DEG_PER_CLASS.get(label, SERVO_DEFAULT), SERVO_MIN, SERVO_MAX))

print("Available classes:")
for i, c in enumerate(CLASSES):
    print(f"{i}: {c}")

# =========================
# Sequência
# =========================
def ask_steps():
    print("\nCreate picking sequence (ordered).")
    print("Example: 3,2,6   (extrato_turma_monica, creme_nestle, pasta_colgate)")
    s = input("Enter class indices in order: ").strip()
    order = []
    for tok in s.replace(" ", "").split(","):
        tok = tok.strip()
        if not tok: continue
        try:
            idx = int(tok)
        except:
            print(f"[WARN] invalid '{tok}', skipping."); continue
        if 0 <= idx < len(CLASSES):
            order.append(idx)
        else:
            print(f"[WARN] out-of-range {idx}, skipping.")
    if not order:
        raise RuntimeError("No indices for sequence. Aborting.")

    steps = []
    print("\nFor each step, set BOX (servo comes from table).")
    for k, idx in enumerate(order, 1):
        label = CLASSES[idx]
        box = input(f"Step {k}: BOX for '{label}' (e.g., A, B, C1): ").strip()
        if not box:
            raise RuntimeError(f"Empty box for '{label}'.")
        steps.append({"label": label, "box": box, "servo": servo_for(label)})
    return steps

STEPS = ask_steps()
CURRENT_STEP = 0
def active_label(): return STEPS[CURRENT_STEP]["label"]
def active_box():   return STEPS[CURRENT_STEP]["box"]
def active_servo(): return STEPS[CURRENT_STEP]["servo"]

print("\n[INFO] Sequence:")
for i, st in enumerate(STEPS, 1):
    print(f"  {i:02d}) {st['label']} -> {st['box']} | servo {st['servo']}")

# =========================
# Serial utils
# =========================
DETECTED_PORT = None
DETECTED_BAUD = None
_last_global_tx_ms = 0

def scan_ports():
    return [(p.device, p.description or "", p.hwid or "") for p in list_ports.comports()]

def candidate_ports():
    ports = scan_ports()
    priority, others = [], []
    keys = ("arduino", "wch", "ch340", "cp210", "ftdi", "usb-serial", "silicon labs", "usb")
    for dev, desc, hwid in ports:
        blob = f"{dev} {desc} {hwid}".lower()
        (priority if any(k in blob for k in keys) else others).append((dev, desc, hwid))
    return priority + others

def try_open(port, baud, timeout=0.2):
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=timeout,
                            write_timeout=timeout, inter_byte_timeout=timeout)
        if not AVOID_DTR_TOGGLE:
            try:
                ser.setDTR(False); ser.setRTS(False); time.sleep(0.05)
                ser.setDTR(True);  ser.setRTS(True)
            except Exception: pass
        ser.reset_input_buffer(); ser.reset_output_buffer(); time.sleep(1.6)
        return ser
    except Exception as e:
        print(f"[WARN] Open failed {port}@{baud}: {e}"); return None

def connect_arduino(port_pref, baud_default):
    global DETECTED_PORT, DETECTED_BAUD
    ports = candidate_ports()
    if port_pref != "auto":
        ports = [(port_pref, "user", "")] + [p for p in ports if p[0] != port_pref]
    print("[INFO] Scanning serial ports:")
    for dev, desc, hwid in scan_ports():
        print(f"   - {dev:6} | {desc} | {hwid}")
    bauds = [baud_default] + [b for b in BAUD_CANDIDATES if b != baud_default]
    for dev, _, _ in ports or []:
        for b in bauds:
            ser = try_open(dev, b)
            if ser:
                DETECTED_PORT, DETECTED_BAUD = dev, b
                print(f"[INFO] Connected at {DETECTED_PORT} @ {DETECTED_BAUD} bps")
                return ser
    print("[ERROR] No serial port worked. Close any Serial Monitor and try again.")
    return None

arduino = connect_arduino(ARDUINO_PORT, BAUD_RATE)
def now_ms(): return int(time.time()*1000)

def serial_send_payload(line: str):
    global _last_global_tx_ms
    if not arduino: return
    t = now_ms()
    if t - _last_global_tx_ms < GLOBAL_TX_MIN_INTERVAL_MS:
        return
    try:
        arduino.write((line + SERIAL_LINE_ENDING).encode("ascii", errors="ignore"))
        _last_global_tx_ms = t
        print(f"[TX] {t}ms -> {line}")
    except Exception as e:
        print(f"[ERROR] Serial write failed: {e}")

# =========================
# YOLO & Camera
# =========================
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"Model not found: {MODEL_PATH}")
model = YOLO(MODEL_PATH)
print("[INFO] YOLO loaded")

def open_camera(indices):
    for idx in indices:
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            print(f"[INFO] Camera opened at index {idx}")
            return cap, idx
        if cap: cap.release()
    return None, None

cap, cam_idx = open_camera(TRY_INDEXES)
if cap is None:
    raise Exception(f"Failed to open camera (tried: {TRY_INDEXES})")

FRAME_W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
FRAME_H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] Resolution: {FRAME_W}x{FRAME_H} (cam index {cam_idx})")

# =========================
# UI
# =========================
WINDOW_NAME = "YOLO Control System"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
try:
    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
except Exception:
    pass

UI_W, UI_H = 1920, 1080

# Cores
COL_BG = (20, 20, 25)
COL_CARD = (30, 33, 40)
COL_ACCENT = (0, 180, 255)
COL_SUCCESS = (50, 205, 50)
COL_WARNING = (255, 180, 0)
COL_DANGER = (255, 70, 70)
COL_TEXT = (240, 240, 245)
COL_TEXT_DIM = (140, 145, 155)
COL_BORDER = (55, 58, 65)

BTN_RECTS = {}
SLIDER_RECTS = {}  # Para sliders arrastáveis

# Runtime state - Carrega presets da primeira classe
first_class_label = STEPS[0]['label']
G_OFFSET = CLASS_G_OFFSET.get(first_class_label, G_OFFSET_DEFAULT)
DELAY_FACTOR = CLASS_DELAY_FACTOR.get(first_class_label, DELAY_FACTOR_DEFAULT)
VELOCITY = CLASS_VELOCITY.get(first_class_label, VELOCITY_DEFAULT)

VEL_INPUT_ACTIVE = False
VEL_INPUT_TEXT = str(VELOCITY)
VEL_INPUT_RECT = (0,0,0,0)

# Estado de arrasto
DRAGGING_SLIDER = None

LOG_MAX = 25
log_lines = deque(maxlen=LOG_MAX)
last_sent_line = "-"
last_class_send_ms = {}

def log(msg: str):
    ts = time.strftime("%H:%M:%S")
    log_lines.appendleft(f"[{ts}] {msg}")

# ===== UI HELPERS =====
def draw_text(img, text, x, y, size=0.6, color=COL_TEXT, thickness=1):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness, cv2.LINE_AA)

def draw_card(img, x, y, w, h, title=None):
    cv2.rectangle(img, (x, y), (x+w, y+h), COL_CARD, -1)
    cv2.rectangle(img, (x, y), (x+w, y+h), COL_BORDER, 1)
    if title:
        draw_text(img, title, x+15, y+30, 0.75, COL_TEXT, 2)
        cv2.line(img, (x+10, y+45), (x+w-10, y+45), COL_BORDER, 1)

def draw_button(img, name, label, x, y, w=140, h=50, color=COL_ACCENT):
    cv2.rectangle(img, (x, y), (x+w, y+h), color, -1)
    cv2.rectangle(img, (x, y), (x+w, y+h), COL_BORDER, 1)
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    draw_text(img, label, x + (w-tw)//2, y + (h+th)//2, 0.6, (255,255,255), 2)
    BTN_RECTS[name] = (x, y, x+w, y+h)
    return x + w + 15

def draw_slider(img, name, x, y, w, label, value, vmin, vmax):
    draw_text(img, label, x, y, 0.65, COL_TEXT, 1)
    
    # Valor no lado direito
    if isinstance(value, float):
        val_text = f"{value:.3f}"
    else:
        val_text = f"{value:+d}" if value < 0 else str(value)
    draw_text(img, val_text, x+w-100, y, 0.7, COL_ACCENT, 2)
    
    # Barra
    bar_y = y + 15
    bar_h = 12
    cv2.rectangle(img, (x, bar_y), (x+w, bar_y+bar_h), (15,15,20), -1)
    cv2.rectangle(img, (x, bar_y), (x+w, bar_y+bar_h), COL_BORDER, 1)
    
    # Preenchimento
    frac = (value - vmin) / (vmax - vmin) if vmax != vmin else 0
    fill_w = int(w * frac)
    cv2.rectangle(img, (x, bar_y), (x+fill_w, bar_y+bar_h), COL_ACCENT, -1)
    
    # Indicador arrastável
    px = int(x + frac * w)
    cv2.circle(img, (px, bar_y+bar_h//2), 10, COL_TEXT, -1)
    cv2.circle(img, (px, bar_y+bar_h//2), 10, COL_ACCENT, 2)
    
    # Salva área do slider para detecção de clique
    SLIDER_RECTS[name] = (x, bar_y-10, x+w, bar_y+bar_h+10, vmin, vmax)

def draw_info_box(img, x, y, w, h, label, value, color=COL_ACCENT):
    cv2.rectangle(img, (x, y), (x+w, y+h), (20,23,30), -1)
    cv2.rectangle(img, (x, y), (x+w, y+h), color, 2)
    draw_text(img, label, x+15, y+25, 0.55, COL_TEXT_DIM, 1)
    draw_text(img, str(value), x+15, y+55, 0.85, COL_TEXT, 2)

def draw_input(img, label, text, x, y, w, h, active=False):
    draw_text(img, label, x, y-12, 0.55, COL_TEXT_DIM, 1)
    bg = (40,43,50) if active else (20,23,30)
    border_col = COL_ACCENT if active else COL_BORDER
    cv2.rectangle(img, (x, y), (x+w, y+h), bg, -1)
    cv2.rectangle(img, (x, y), (x+w, y+h), border_col, 2 if active else 1)
    
    display = text + ("_" if active and int(time.time()*2)%2==0 else "")
    draw_text(img, display, x+15, y+h-15, 0.75, COL_TEXT, 2)

def draw_progress(img, x, y, w, steps, current):
    step_w = w // len(steps)
    for i in range(len(steps)):
        sx = x + i * step_w + step_w//2
        
        # Círculo
        if i < current:
            col = COL_SUCCESS
            cv2.circle(img, (sx, y), 16, col, -1)
        elif i == current:
            col = COL_ACCENT
            cv2.circle(img, (sx, y), 16, col, -1)
        else:
            col = COL_TEXT_DIM
            cv2.circle(img, (sx, y), 16, col, 2)
        
        # Linha conectora
        if i < len(steps) - 1:
            line_col = COL_SUCCESS if i < current else COL_BORDER
            cv2.line(img, (sx+16, y), (sx+step_w-16, y), line_col, 2)
        
        # Número
        num = str(i+1)
        (tw, th), _ = cv2.getTextSize(num, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        draw_text(img, num, sx-tw//2, y+th//2, 0.5, (255,255,255), 1)
        
        # Label
        lbl = steps[i]['label'][:10]
        (tw2, _), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        text_col = COL_TEXT if i == current else COL_TEXT_DIM
        draw_text(img, lbl, sx-tw2//2, y+32, 0.45, text_col, 1)

def map_to_scale(v, vmin, vmax, out_max, invert=False):
    if vmax == vmin: return 0
    v = clamp(v, vmin, vmax)
    if invert: v = vmax - (v - vmin)
    norm = (v - vmin) / (vmax - vmin)
    return clamp(int(round(norm * out_max)), 0, out_max)

def compute_g_and_delay(cx, cy):
    g = map_to_scale(cx, 0, FRAME_W, G_SCALE_MAX, invert=G_INVERT)
    mid = G_SCALE_MAX // 2
    dyn = (g - mid) / float(mid)
    dyn_off = int(round(dyn * DYN_PULL_MAX))
    g_corr = g - dyn_off
    g_send = clamp(g_corr + G_OFFSET, 0, G_SCALE_MAX)

    base = map_to_scale(cy, 0, FRAME_H, B_SCALE_MAX, invert=True)
    delay_ms = int(round(clamp(base * DELAY_FACTOR, 0, DELAY_MAX_MS)))
    return g_send, base, delay_ms, g, dyn_off

def goto_next_step():
    global CURRENT_STEP, G_OFFSET, DELAY_FACTOR, VELOCITY, VEL_INPUT_TEXT
    if CURRENT_STEP + 1 < len(STEPS):
        CURRENT_STEP += 1
        st = STEPS[CURRENT_STEP]
        
        # Carrega presets da nova classe ativa
        new_label = st['label']
        G_OFFSET = CLASS_G_OFFSET.get(new_label, G_OFFSET_DEFAULT)
        DELAY_FACTOR = CLASS_DELAY_FACTOR.get(new_label, DELAY_FACTOR_DEFAULT)
        VELOCITY = CLASS_VELOCITY.get(new_label, VELOCITY_DEFAULT)
        VEL_INPUT_TEXT = str(VELOCITY)
        
        log(f"STEP -> {CURRENT_STEP+1}/{len(STEPS)}: {st['label']} -> {st['box']} (servo {st['servo']})")
        log(f"Loaded presets: G={G_OFFSET:+d} DF={DELAY_FACTOR:.3f} V={VELOCITY}")
    else:
        log("STEP -> end (staying)")

def class_ready(label, nowms):
    last = last_class_send_ms.get(label, 0)
    return (nowms - last) >= COOLDOWN_MS_CLASS or last == 0

# Mouse
def on_mouse(event, x, y, flags, param):
    global VEL_INPUT_ACTIVE, VEL_INPUT_TEXT, G_OFFSET, DELAY_FACTOR, DRAGGING_SLIDER
    
    # Arrasto de slider
    if event == cv2.EVENT_LBUTTONDOWN:
        for name, (sx, sy, sx2, sy2, vmin, vmax) in SLIDER_RECTS.items():
            if sx <= x <= sx2 and sy <= y <= sy2:
                DRAGGING_SLIDER = name
                # Atualiza valor imediatamente
                frac = clamp((x - sx) / (sx2 - sx), 0, 1)
                new_val = vmin + frac * (vmax - vmin)
                
                if name == "G_OFFSET":
                    G_OFFSET = int(round(new_val))
                elif name == "DELAY_FACTOR":
                    DELAY_FACTOR = round(new_val, 3)
                elif name == "CONFIDENCE":
                    CLASS_MIN_CONF[active_label()] = round(new_val, 2)
                return
    
    elif event == cv2.EVENT_MOUSEMOVE and DRAGGING_SLIDER:
        name, (sx, sy, sx2, sy2, vmin, vmax) = [(n, r) for n, r in SLIDER_RECTS.items() if n == DRAGGING_SLIDER][0]
        frac = clamp((x - sx) / (sx2 - sx), 0, 1)
        new_val = vmin + frac * (vmax - vmin)
        
        if name == "G_OFFSET":
            G_OFFSET = int(round(new_val))
        elif name == "DELAY_FACTOR":
            DELAY_FACTOR = round(new_val, 3)
        elif name == "CONFIDENCE":
            CLASS_MIN_CONF[active_label()] = round(new_val, 2)
        return
    
    elif event == cv2.EVENT_LBUTTONUP:
        if DRAGGING_SLIDER:
            log(f"Adjusted {DRAGGING_SLIDER}")
            DRAGGING_SLIDER = None
        return
    
    if event != cv2.EVENT_LBUTTONDOWN: return

    # Input velocity
    x1,y1,x2,y2 = VEL_INPUT_RECT
    if x1 <= x <= x2 and y1 <= y <= y2:
        VEL_INPUT_ACTIVE = True
        return

    # Botões
    for name, (rx1, ry1, rx2, ry2) in list(BTN_RECTS.items()):
        if rx1 <= x <= rx2 and ry1 <= y <= ry2:
            if name == "RESET":
                reset_tuning()
            elif name == "NEXT":
                goto_next_step()
            break

def reset_tuning():
    global G_OFFSET, DELAY_FACTOR, VELOCITY, VEL_INPUT_TEXT
    G_OFFSET = G_OFFSET_DEFAULT
    DELAY_FACTOR = DELAY_FACTOR_DEFAULT
    VELOCITY = VELOCITY_DEFAULT
    VEL_INPUT_TEXT = str(VELOCITY)
    log("Reset tuning to defaults")

cv2.setMouseCallback(WINDOW_NAME, on_mouse)

# =========================
# Loop principal
# =========================
while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Capture failed."); break

    # YOLO
    res = model(frame, imgsz=YOLO_IMG_SIZE, conf=YOLO_CONF_GLOBAL, iou=YOLO_IOU, verbose=False)
    dets = []
    if res and len(res) > 0 and getattr(res[0], "boxes", None) is not None:
        try:
            dets = res[0].boxes.data.cpu().numpy()
        except Exception:
            dets = []

    # Desenha detecções na frame
    frame_display = frame.copy()
    target = active_label()
    min_conf = CLASS_MIN_CONF.get(target, DEFAULT_MIN_CONF)
    candidates = []
    
    for d in dets:
        x1, y1, x2, y2, conf, cls = d
        conf = float(conf); cls = int(cls)
        label = CLASSES[cls] if 0 <= cls < len(CLASSES) else f"id_{cls}"
        
        color = (0, 255, 0) if label == target and conf >= min_conf else (100, 100, 100)
        cv2.rectangle(frame_display, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        cv2.putText(frame_display, f"{label} {conf:.2f}", (int(x1), int(y1)-5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        
        if label == target and conf >= min_conf:
            cxi, cyi = int((x1+x2)/2.0), int((y1+y2)/2.0)
            candidates.append({"cx": cxi, "cy": cyi, "conf": conf})
            cv2.circle(frame_display, (cxi, cyi), 5, (0, 255, 255), -1)

    # Envio
    tnow = now_ms()
    sent_this_loop = False
    if candidates:
        for c in candidates[:3]:
            g_send, base, delay_ms, g_raw, dyn_off = compute_g_and_delay(c["cx"], c["cy"])
            if arduino and class_ready(target, tnow) and (not sent_this_loop or not LIMIT_ONE_LINE_PER_LOOP):
                line = f"{g_send} {active_box()} {delay_ms} {active_servo()} {VELOCITY}"
                serial_send_payload(line)
                last_class_send_ms[target] = tnow
                last_sent_line = line
                log(f"SENT: {line}")
                sent_this_loop = True
            log(f"{target} conf={c['conf']:.2f} | G={g_send:04d} | Delay={delay_ms}ms")

    # ===== UI =====
    ui = np.zeros((UI_H, UI_W, 3), dtype=np.uint8)
    ui[:] = COL_BG

    # Header
    cv2.rectangle(ui, (0, 0), (UI_W, 70), COL_CARD, -1)
    draw_text(ui, "YOLO CONTROL SYSTEM", 30, 45, 1.0, COL_TEXT, 2)
    
    status_text = f"Port: {DETECTED_PORT or 'N/A'} @ {DETECTED_BAUD or 0} bps"
    status_color = COL_SUCCESS if arduino else COL_DANGER
    cv2.circle(ui, (UI_W-180, 35), 8, status_color, -1)
    draw_text(ui, status_text, UI_W-430, 42, 0.5, COL_TEXT_DIM, 1)

    # Barra de progresso
    draw_progress(ui, 30, 105, UI_W-60, STEPS, CURRENT_STEP)

    # LAYOUT REORGANIZADO: Camera | Status (menor) + Controls (maior)
    cam_x, cam_y = 30, 160
    cam_w, cam_h = 900, 860
    
    status_x, status_y = cam_x + cam_w + 20, 160
    status_w = 930
    status_h = 340
    
    ctrl_x, ctrl_y = status_x, status_y + status_h + 20
    ctrl_w = status_w
    ctrl_h = 520

    # CARD: Câmera
    draw_card(ui, cam_x, cam_y, cam_w, cam_h, "CAMERA FEED")
    
    display_h = cam_h - 70
    display_w = int(FRAME_W * display_h / FRAME_H)
    if display_w > cam_w - 40:
        display_w = cam_w - 40
        display_h = int(FRAME_H * display_w / FRAME_W)
    
    frame_resized = cv2.resize(frame_display, (display_w, display_h))
    offset_x = cam_x + (cam_w - display_w) // 2
    offset_y = cam_y + 60
    ui[offset_y:offset_y+display_h, offset_x:offset_x+display_w] = frame_resized

    # CARD: Status (COMPACTO)
    draw_card(ui, status_x, status_y, status_w, status_h, "STATUS")
    
    y_info = status_y + 65
    draw_info_box(ui, status_x+20, y_info, 280, 75, "Target Class", target, COL_ACCENT)
    draw_info_box(ui, status_x+320, y_info, 280, 75, "Target Box", active_box(), COL_ACCENT)
    draw_info_box(ui, status_x+620, y_info, 280, 75, "Servo", f"{active_servo()} deg", COL_WARNING)
    
    y_info += 95
    draw_info_box(ui, status_x+20, y_info, 420, 75, f"Step: {CURRENT_STEP+1}/{len(STEPS)}", f"Velocity: {VELOCITY}", COL_SUCCESS)
    draw_info_box(ui, status_x+460, y_info, 440, 75, "Last TX", last_sent_line[:30], COL_ACCENT)
    
    y_info += 95
    det_count = len([d for d in dets if CLASSES[int(d[5])] == target and float(d[4]) >= min_conf])
    draw_text(ui, f"Detections: {det_count} | Camera: {FRAME_W}x{FRAME_H} @ {cam_idx}", 
              status_x+20, y_info, 0.55, COL_TEXT_DIM, 1)

    # CARD: Controles (GRANDE E ESPAÇOSO)
    draw_card(ui, ctrl_x, ctrl_y, ctrl_w, ctrl_h, "CONTROLS")
    
    y_slider = ctrl_y + 70
    spacing = 70
    
    # Sliders grandes e arrastáveis
    draw_slider(ui, "G_OFFSET", ctrl_x+30, y_slider, ctrl_w-60, 
                f"G Offset", G_OFFSET, G_OFFSET_MIN, G_OFFSET_MAX)
    
    y_slider += spacing
    draw_slider(ui, "DELAY_FACTOR", ctrl_x+30, y_slider, ctrl_w-60,
                f"Delay Factor", DELAY_FACTOR, 0, 1.0)
    
    y_slider += spacing
    cur_conf = CLASS_MIN_CONF.get(target, DEFAULT_MIN_CONF)
    draw_slider(ui, "CONFIDENCE", ctrl_x+30, y_slider, ctrl_w-60,
                f"Confidence [{target[:15]}]", cur_conf, 0, 1.0)
    
    # Input velocity
    y_slider += spacing + 10
    VEL_INPUT_RECT = (ctrl_x+30, y_slider, ctrl_x+250, y_slider+55)
    draw_input(ui, "Velocity", VEL_INPUT_TEXT if VEL_INPUT_ACTIVE else str(VELOCITY),
               ctrl_x+30, y_slider, 220, 55, VEL_INPUT_ACTIVE)
    
    # Botões grandes
    y_btn = ctrl_y + ctrl_h - 70
    bx = ctrl_x + 30
    draw_button(ui, "RESET", "RESET", bx, y_btn, 200, 55, COL_DANGER)
    draw_button(ui, "NEXT", "NEXT STEP", ctrl_x+ctrl_w-230, y_btn, 200, 55, COL_SUCCESS)

    # Footer
    footer_y = UI_H - 40
    cv2.rectangle(ui, (0, footer_y), (UI_W, UI_H), COL_CARD, -1)
    draw_text(ui, "ARRASTE OS SLIDERS | ENTER/SPACE/N: Next | [/]: Vel +-1 | R: Reconnect | ESC: Quit",
              30, footer_y+25, 0.5, COL_TEXT_DIM, 1)

    cv2.imshow(WINDOW_NAME, ui)

    # ===== Keyboard =====
    key = cv2.waitKey(1) & 0xFFFFFFFF

    if VEL_INPUT_ACTIVE:
        if key in (13, 10):
            try:
                val = int(VEL_INPUT_TEXT) if VEL_INPUT_TEXT.strip() != "" else VELOCITY
            except: val = VELOCITY
            VELOCITY = clamp(val, VELOCITY_MIN, VELOCITY_MAX)
            VEL_INPUT_TEXT = str(VELOCITY)
            VEL_INPUT_ACTIVE = False
            log(f"Velocity set to {VELOCITY}")
        elif key in (27,):
            VEL_INPUT_TEXT = str(VELOCITY)
            VEL_INPUT_ACTIVE = False
        elif key in (8, 255):
            VEL_INPUT_TEXT = VEL_INPUT_TEXT[:-1] if len(VEL_INPUT_TEXT) > 0 else ""
        elif key != 0xFFFFFFFF:
            ch = key & 0xFF
            if 48 <= ch <= 57 and len(VEL_INPUT_TEXT) < 4:
                VEL_INPUT_TEXT += chr(ch)
        continue

    if key == 27: break
    elif key in (ord('r'), ord('R')):
        log("Reconnecting serial...")
        if arduino:
            try: arduino.close()
            except Exception: pass
        arduino = connect_arduino(ARDUINO_PORT, BAUD_RATE)
    elif key in (13, 10, 32, ord('n'), ord('N')):
        goto_next_step()
    elif key == ord(']'):
        VELOCITY = clamp(VELOCITY + VELOCITY_STEP, VELOCITY_MIN, VELOCITY_MAX)
        VEL_INPUT_TEXT = str(VELOCITY); log(f"Velocity = {VELOCITY}")
    elif key == ord('['):
        VELOCITY = clamp(VELOCITY - VELOCITY_STEP, VELOCITY_MIN, VELOCITY_MAX)
        VEL_INPUT_TEXT = str(VELOCITY); log(f"Velocity = {VELOCITY}")
    elif key == ord('0'):
        reset_tuning()

# Teardown
cap.release()
cv2.destroyAllWindows()
if arduino:
    try: arduino.close()
    except Exception: pass
print("[INFO] Clean exit.")