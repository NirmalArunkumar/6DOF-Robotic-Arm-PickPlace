# 6-DOF Robotic Arm with Vision-Guided IK

A complete embedded + computer-vision pipeline for a 6-degree-of-freedom robotic arm. The arm autonomously locates objects with a YOLOv8 detector, converts pixel coordinates to world coordinates via a calibrated homography, solves the inverse kinematics analytically, and drives six servo joints with closed-loop potentiometer feedback over a PCA9685 PWM driver.

> **Note:** The firmware (`main.cpp`) is written in Arduino style. Our STM32 burned one day before the demo, so we pivoted to an Arduino-compatible board with what we had on hand.

---

## Repository Structure

| File | Description |
|------|-------------|
| `main.cpp` | Firmware: FK/IK solver, closed-loop servo control, serial command interface |
| `final_part1.py` | Python host script: YOLOv8 inference + camera-to-robot coordinate pipeline |
| `train.py` | YOLOv8 fine-tuning script for the custom object class |
| `best.pt` | Trained YOLOv8 model weights |
| `calibration.json` | Camera calibration: pixel ↔ real-world (cm) point correspondences |
| `Algorithm design IKP.pdf` | Full algorithm derivation: DH parameters, IK math, control architecture |

---

## Algorithm Overview

See [`Algorithm design IKP.pdf`](Algorithm%20design%20IKP.pdf) for the complete derivation. A summary:

### 1. Denavit-Hartenberg Kinematic Model

The arm is modelled with standard DH parameters across 6 joints:

| Joint | a (cm) | d (cm) | α (rad) |
|-------|--------|--------|---------|
| 1 (Base)     | −1.0  | 9.0   | π/2  |
| 2 (Shoulder) | 10.5  | 0.0   | 0    |
| 3 (Elbow)    | 0.0   | 0.0   | π/2  |
| 4 (Forearm)  | 0.0   | 14.4  | −π/2 |
| 5 (Wrist P)  | 0.0   | 0.0   | π/2  |
| 6 (Wrist R)  | 0.0   | 18.0  | 0    |

### 2. Forward Kinematics

Iteratively applies DH rotation matrices and translation vectors to accumulate the end-effector position and orientation from the base frame.

### 3. Analytical Inverse Kinematics

Given a target position `(xd, yd, zd)` and desired end-effector orientation `Rd`:

1. **Wrist centre** — offset the target back along the approach vector by `d6` to find the wrist centre `(xc, yc, zc)`.
2. **θ₁** — base rotation: `atan2(yc, xc)`.
3. **θ₂, θ₃** — geometric solution using the triangle formed by links A2 and the combined `√(A3² + D4²)` link, applying the law of cosines.
4. **θ₄, θ₅, θ₆** — wrist Euler angles extracted from `R₃₆ = R₀₃ᵀ · Rd` (ZYZ decomposition with a quadrant flip to keep all angles in the servo range).

### 4. Closed-Loop Servo Control

Each encoded joint runs a two-phase move:

- **Phase 1 — open-loop ramp:** all joints move synchronously at a fixed angular step rate.
- **Phase 2 — closed-loop correction:** potentiometer ADC readings are converted to degrees via a linear calibration and compared to the target. A proportional nudge (`err × 0.6`) is applied iteratively until the error falls within `±2.5°` or a stall/timeout is detected.

### 5. Vision Pipeline

1. **Detection:** YOLOv8 runs on a host machine and returns the bounding-box centre pixel of the target object.
2. **Calibration:** `calibration.json` stores ground-truth pixel ↔ real-world (cm) pairs. A least-squares affine fit maps any detected pixel directly to `(x, y)` in the robot base frame.
3. **Execution:** The host sends an `M x y z` command over serial; the firmware solves IK and drives the arm.

---

## Hardware

- **Microcontroller:** Arduino (originally STM32, replaced after failure)
- **PWM Driver:** PCA9685 (I²C, 50 Hz, channels 0–7)
- **Servos:** 6× hobby servos (180°)
- **Position Feedback:** 4× potentiometers (base, shoulder, elbow, wrist pitch) wired to analog pins A0–A3
- **Camera:** USB camera on host PC

### Wiring Summary

| Encoder | Analog Pin | PCA9685 Channel |
|---------|-----------|-----------------|
| Base        | A0 | CH6 |
| Shoulder    | A1 | CH5 |
| Elbow       | A2 | CH7 |
| Wrist Pitch | A3 | CH2 |

---

## Serial Command Interface

Connect at **9600 baud** and send commands over the serial monitor:

| Command | Description |
|---------|-------------|
| `H` | Move to home position |
| `M x y z` | Move end-effector to (x, y, z) cm — closed-loop |
| `D x y z` | Dry-run: solve and print IK without moving |
| `F t1 t2 t3 t4 t5 t6` | Forward kinematics from DH angles (degrees) |
| `J joint deg` | Set a single joint in DH degrees |
| `G 0/1` | Open / close gripper |
| `S ch ang` | Set raw servo angle on PCA9685 channel |
| `P` | Print current positions and live encoder readings |
| `C` | Run encoder calibration routine |
| `E` | Read and print all encoders immediately |
| `R` | Re-initialise PCA9685 |
| `T ch` | Wiggle test on channel `ch` |
| `?` | Print this menu and current status |

---

## Getting Started

### Firmware

1. Open `main.cpp` in the Arduino IDE (or PlatformIO).
2. Install dependencies:
   - `Adafruit PWM Servo Driver Library`
3. Flash to the board.
4. Open the serial monitor at 9600 baud.
5. Run `C` to calibrate encoders, note the ADC values, update `ENC_ADC_LOW` / `ENC_ADC_HIGH` in the code, and reflash.

### Python Host (Vision Pipeline)

```bash
pip install ultralytics opencv-python numpy
python final_part1.py
```

Ensure the camera index and serial port are set correctly at the top of the script.

### Training a Custom Model

```bash
# Prepare a YOLO-format dataset, then:
python train.py
```

---

## Calibration

`calibration.json` maps pixel coordinates from the overhead camera to real-world positions in centimetres. To recalibrate:

1. Place markers at known positions on the work surface.
2. Record their pixel coordinates from the camera feed.
3. Update `calibration.json` with the new `{ "pixel": [u, v], "real": [x, y] }` pairs.
4. Re-run the host script; the affine transform is recomputed at startup.

---

## Project
MIT license - anyone can modify and share.

