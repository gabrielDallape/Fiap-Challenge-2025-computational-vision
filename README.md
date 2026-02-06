# Automated Product Sorting with Computer Vision

> Robotic arm system that uses **YOLOv8** real-time object detection to identify supermarket products on a conveyor belt and autonomously sort them into designated boxes.

**FIAP Challenge 2025 - Computational Vision**

---

## Overview

This project integrates **computer vision** with **robotic automation** to create a pick-and-place system for supermarket products. A camera detects products on a moving conveyor belt using a custom-trained YOLOv8 model, computes the optimal gripper position, and sends commands to an Arduino-controlled robotic arm that picks and sorts items into the correct box.

### Key Features

- **Real-time detection** of 10 supermarket product classes using YOLOv8 Nano
- **Interactive control panel** built with OpenCV for live parameter tuning
- **Serial communication protocol** between Python and Arduino at 115200 baud
- **Per-class calibration** with individual confidence thresholds, servo angles, and offsets
- **Multi-step picking sequences** with configurable box destinations
- **Dual-axis stepper control** with simultaneous homing and non-blocking cruise mode

---

## System Architecture

```
Camera Feed (USB)
      |
      v
 YOLO Detector (Python)  ──>  Interactive UI (OpenCV)
      |
      v
Coordinate Mapping
 (pixel -> robot coords)
      |
      v
Serial Protocol (115200 baud)
      |
      v
Arduino Controller
      |
      ├── BASE axis    ── Conveyor belt stepper (DRV8825)
      ├── GRIPPER axis  ── Extend/retract stepper (DRV8825)
      ├── SERVO         ── Gripper open/close (PWM)
      └── RELAY         ── Electromagnet activation
      |
      v
 Pick & Place
 (Box A or Box B)
```

---

## Product Classes

The model detects **10 supermarket products**, each with individually calibrated parameters:

| # | Product | Servo Angle | Confidence |
|---|---------|:-----------:|:----------:|
| 0 | DUCOCO | 135 | 0.55 |
| 1 | Coca-Cola | 103 | 0.15 |
| 2 | Creme Nestle | 137 | 0.50 |
| 3 | Extrato Turma da Monica | 120 | 0.05 |
| 4 | Garrafa de Agua | 110 | 1.95 |
| 5 | Margarina Dorianna | 130 | 0.75 |
| 6 | Pasta Colgate | 140 | 0.50 |
| 7 | Sabonete Dove | 145 | 0.55 |
| 8 | Sardinha Coqueiro | 150 | 0.55 |
| 9 | Todinho | 130 | 0.65 |

---

## Tech Stack

### Software
| Component | Technology |
|-----------|------------|
| Object Detection | YOLOv8 Nano (Ultralytics) |
| Vision Processing | OpenCV |
| Control Interface | Python 3.12 |
| Serial Communication | PySerial |
| Dataset Management | Roboflow |

### Hardware
| Component | Specification |
|-----------|--------------|
| Microcontroller | Arduino (ATmega) |
| Stepper Drivers | 2x DRV8825 |
| Motors | 2x NEMA Stepper |
| Gripper | Servo + Electromagnetic Relay |
| Camera | USB Webcam |

---

## Project Structure

```
.
├── arduino/
│   └── robot_controller/
│       └── robot_controller.ino    # Stepper/servo/relay control firmware
├── vision/
│   ├── main.py                     # Main control system with YOLO + interactive UI
│   ├── cam_serial.py               # Camera and serial communication module
│   └── arduino_pick.py             # Arduino picking sequence helper
├── models/
│   └── best.pt                     # Fine-tuned YOLOv8 Nano weights (10 classes)
├── assets/                         # Project images and diagrams
├── .gitignore
├── requirements.txt
└── README.md
```

---

## Getting Started

### Prerequisites

- Python 3.10+
- Arduino IDE
- USB camera
- Arduino board with stepper drivers and mechanical arm assembled

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/gabrielDallape/Fiap-Challenge-2025-computational-vision.git
   cd Fiap-Challenge-2025-computational-vision
   ```

2. **Install Python dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Upload firmware to Arduino**
   - Open `arduino/robot_controller/robot_controller.ino` in Arduino IDE
   - Select the correct board and port
   - Upload

4. **Run the vision system**
   ```bash
   python vision/main.py
   ```

5. **Configure the picking sequence**
   - The system will prompt you to select product classes and box destinations
   - Use the interactive UI to fine-tune parameters in real-time

### Serial Command Protocol

Commands sent from Python to Arduino follow this format:

```
<G_POSITION> <BOX> <DELAY_MS> <SERVO_DEG> <VELOCITY>
```

| Parameter | Range | Description |
|-----------|-------|-------------|
| G_POSITION | 0-9000 | Gripper X position (logical scale) |
| BOX | A or B | Target box destination |
| DELAY_MS | 0-60000 | Base movement timing (ms) |
| SERVO_DEG | 100-150 | Gripper servo angle |
| VELOCITY | 0-999 | Motor speed |

**Example:** `3000 A 250 140 150` - Position gripper at 3000, place in Box A, 250ms delay, servo at 140 degrees, speed 150.

---

## Dataset

The model was trained on a custom dataset of **7,463 images** across 10 supermarket product classes, managed via Roboflow with brightness augmentation.

- **Dataset:** [Produtos Supermercado on Roboflow](https://universe.roboflow.com/fat-atew2/produtos-supermercado-xp5lm/dataset/11)
- **Format:** YOLOv8
- **License:** CC BY 4.0

---

## Interactive UI Controls

The OpenCV-based control panel provides:

- **Draggable sliders** for G Offset, Delay Factor, and Confidence
- **Velocity input** field with keyboard shortcuts
- **Progress bar** showing current picking step
- **Live detection feed** with bounding boxes and confidence scores
- **Status cards** displaying target class, box, servo angle, and last command sent

**Keyboard Shortcuts:**
| Key | Action |
|-----|--------|
| `Enter` / `Space` / `N` | Next step |
| `]` / `[` | Increase/decrease velocity |
| `R` | Reconnect serial |
| `0` | Reset tuning to defaults |
| `ESC` | Quit |

---

## License

Dataset licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
