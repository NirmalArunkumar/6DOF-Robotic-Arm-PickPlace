

import cv2
import serial
import time
import json
import numpy as np
from ultralytics import YOLO

MODEL_PATH       = 'best.pt'
CALIBRATION_FILE = 'calibration.json'
SERIAL_PORT      = '/dev/tty.usbserial-0001' 
SERIAL_BAUD      = 9600

CONF_THRESHOLD   = 0.5
CUBE_MAX_AREA    = 50000
STABLE_SECS      = 3.0
MOVE_THRESHOLD   = 15

Z_SAFE           = 12.0
Z_PICK           = 2.0
Z_PLACE          = 15.0
Z_LIFT           = 15.0

GRIP_CLOSE_MS    = 800
GRIP_OPEN_MS     = 500
"2nd digit offset from robot from origin to real robot oring, 3rd is callibration"
ORIGIN_OFFSET_X =  10.5
ORIGIN_OFFSET_Y  =1.85
"-6.0 was the origin offset"


ser = None

def connect_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=2)
        time.sleep(2)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  [ARM] {line}")
        print(f"  Connected to {SERIAL_PORT}")
        return True
    except Exception as e:
        print(f"  ERROR: {e}")
        return False

def send_cmd(cmd):
    if ser is None:
        print(f"  [SIM] {cmd}")
        return True
    print(f"  [TX] {cmd}")
    ser.write(f"{cmd}\n".encode('utf-8'))
    time.sleep(0.1)
    start = time.time()
    while (time.time() - start) < 10:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  [ARM] {line}")
                if line.startswith("ERR:"):
                    return False
        else:
            time.sleep(0.05)
            if ser.in_waiting == 0 and (time.time() - start) > 1:
                break
    return True

def move_to(x, y, z):
    return send_cmd(f"M {x + ORIGIN_OFFSET_X:.1f} {y + ORIGIN_OFFSET_Y:.1f} {z:.1f}")

def gripper_open():
    send_cmd("G 0")
    time.sleep(GRIP_OPEN_MS / 1000.0)

def gripper_close():
    send_cmd("G 1")
    time.sleep(GRIP_CLOSE_MS / 1000.0)

def go_home():
    send_cmd("H")


H = None  # 3x3 homography matrix

def load_calibration():
    global H
    try:
        with open(CALIBRATION_FILE, 'r') as f:
            cal_data = json.load(f)
        pixel_pts = np.array([p["pixel"] for p in cal_data], dtype=np.float32)
        real_pts  = np.array([p["real"]  for p in cal_data], dtype=np.float32)
        H, _ = cv2.findHomography(pixel_pts, real_pts, cv2.RANSAC)
        print(f"  Calibration loaded ({len(cal_data)} points, homography)")
        return True
    except Exception as e:
        print(f"  No calibration: {e}")
        return False

def pixel_to_cm(px, py):
    if H is None:
        return 0.0, 0.0
    pt = np.array([[[px, py]]], dtype=np.float32)
    result = cv2.perspectiveTransform(pt, H)
    return round(float(result[0][0][0]), 1), round(float(result[0][0][1]), 1)

def run_calibration(model, cap):
    cal_points = []
    typed_text = ""
    waiting    = False
    pending_pixel = None

    print("\n=== CALIBRATION MODE ===")
    print("Place cube, press S, type X,Y in cm, press ENTER. Q when done.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        img_h, img_w = frame.shape[:2]
        results = model(frame, verbose=False)[0]
        cx, cy  = None, None

        for box in results.boxes:
            if box.conf[0].item() < CONF_THRESHOLD:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(frame, f"Pixel: ({cx},{cy})",
                        (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

        cv2.putText(frame, f"Points: {len(cal_points)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)

        if waiting:
            cv2.rectangle(frame, (0, img_h//2-50), (img_w, img_h//2+50), (0,0,0), -1)
            cv2.putText(frame, f"Pixel: {pending_pixel}  Type X,Y: {typed_text}",
                        (20, img_h//2+10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        else:
            cv2.putText(frame, "S=save point  Q=done",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        cv2.imshow("Calibration", frame)
        key = cv2.waitKey(1) & 0xFF

        if waiting:
            if key in [ord(c) for c in '0123456789.,-']:
                typed_text += chr(key)
            elif key == 8:
                typed_text = typed_text[:-1]
            elif key == 13:
                try:
                    parts = typed_text.replace(' ','').split(',')
                    x_real, y_real = float(parts[0]), float(parts[1])
                    cal_points.append({"pixel": list(pending_pixel), "real": [x_real, y_real]})
                    print(f"  Saved: pixel {pending_pixel} → ({x_real},{y_real}) cm")
                except:
                    print("  Bad input — use X,Y format")
                typed_text = ""
                waiting    = False
        else:
            if key == ord('s') and cx is not None:
                pending_pixel = (cx, cy)
                waiting       = True
                typed_text    = ""
            elif key == ord('q'):
                break

    cv2.destroyWindow("Calibration")
    if len(cal_points) >= 4:
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(cal_points, f, indent=2)
        print(f"  Saved {len(cal_points)} points")
        load_calibration()
    else:
        print(f"  Need 4+ points, got {len(cal_points)}")

def pick_and_place(cube_x, cube_y, bowl_x, bowl_y):
    cube_y = -cube_y
    bowl_y = -bowl_y
    print(f"\n{'='*50}")
    print(f"  PICK ({cube_x:.1f},{cube_y:.1f}) → PLACE ({bowl_x:.1f},{bowl_y:.1f})")
    print(f"{'='*50}")

    # 1. Open gripper
    print("  Step 1: Open gripper")
    gripper_open()

    # 2. Move above cube
    print(f"  Step 2: Above cube at Z={Z_SAFE}")
    if not move_to(cube_x, cube_y, Z_SAFE):
        print("  FAILED — aborting"); go_home(); return False

    # 3. Lower to cube
    print(f"  Step 3: Lower to cube at Z={Z_PICK}")
    if not move_to(cube_x, cube_y, Z_PICK):
        print("  FAILED — aborting"); go_home(); return False

    # 4. Close gripper
    print("  Step 4: Close gripper")
    gripper_close()

    # 5. Lift
    print(f"  Step 5: Lift to Z={Z_LIFT}")
    move_to(cube_x, cube_y, Z_LIFT)

    # 6. Move above bowl
    print(f"  Step 6: Above bowl at Z={Z_LIFT}")
    move_to(bowl_x, bowl_y, Z_LIFT)

    # 7. Lower to place
    print(f"  Step 7: Lower to Z={Z_PLACE}")
    move_to(bowl_x, bowl_y, Z_PLACE)

    # 8. Open gripper
    print("  Step 8: Open gripper")
    gripper_open()

    # 9. Lift
    print(f"  Step 9: Retreat to Z={Z_LIFT}")
    move_to(bowl_x, bowl_y, Z_LIFT)

    # 10. Home
    print("  Step 10: Home")
    go_home()

    print("  DONE\n")
    return True


def main():
    model = YOLO(MODEL_PATH)
    load_calibration()
    print(f"\n  Connecting to arm on {SERIAL_PORT}")
    connect_serial()
    cap = cv2.VideoCapture(0)
    trackers  = {}
    all_sent  = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        img_h, img_w = frame.shape[:2]
        results  = model(frame, verbose=False)[0]
        key      = cv2.waitKey(1) & 0xFF
        seen_keys = set()

        cubes = []
        bowls = []

        for i, box in enumerate(results.boxes):
            if box.conf[0].item() < CONF_THRESHOLD:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx   = int((x1 + x2) / 2)
            cy   = int((y1 + y2) / 2)
            area = (x2 - x1) * (y2 - y1)

            cls_name = 'bowl' if area > CUBE_MAX_AREA else 'cube'
            x_cm, y_cm = pixel_to_cm(cx, cy)
            color = (255, 165, 0) if cls_name == 'bowl' else (0, 255, 0)

            obj = {
                'cls':  cls_name,
                'x_cm': x_cm,
                'y_cm': y_cm,
                'cx':   cx,
                'cy':   cy,
                'conf': box.conf[0].item(),
                'area': area
            }

            if cls_name == 'cube':
                cubes.append(obj)
            else:
                bowls.append(obj)

         
            seen_keys.add(i)
            if i not in trackers:
                trackers[i] = {'stable_since': time.time(), 'last_cx': cx, 'last_cy': cy}
            t = trackers[i]
            if t['last_cx'] is not None:
                moved = abs(cx - t['last_cx']) + abs(cy - t['last_cy'])
                if moved > MOVE_THRESHOLD:
                    t['stable_since'] = time.time()
                    all_sent = False
            t['last_cx'] = cx
            t['last_cy'] = cy
            elapsed = time.time() - t['stable_since']

            # Draw
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(frame, f"{cls_name} ({x_cm},{y_cm})cm",
                        (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(frame, f"conf:{box.conf[0].item():.2f} area:{area:.0f}px",
                        (int(x1), int(y1)-25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            # Stability bar
            bar_y    = int(y2) + 5
            bar_fill = int(min(elapsed / STABLE_SECS, 1.0) * 100)
            cv2.rectangle(frame, (int(x1), bar_y), (int(x1)+100, bar_y+6), (50,50,50), -1)
            cv2.rectangle(frame, (int(x1), bar_y), (int(x1)+bar_fill, bar_y+6), color, -1)

    
        for k in list(trackers.keys()):
            if k not in seen_keys:
                del trackers[k]
        if not seen_keys:
            all_sent = False

     
        all_stable = (
            len(cubes) > 0 and len(bowls) > 0 and
            all(
                (time.time() - trackers[i]['stable_since']) >= STABLE_SECS
                for i in seen_keys if i in trackers
            )
        )

        # Auto trigger 
        if all_stable and not all_sent and H is not None:
            all_sent = True
            print(f"\n  Detected {len(cubes)} cube(s) and {len(bowls)} bowl(s) — stable!")
            for cube in cubes:
                best_bowl = min(bowls, key=lambda b:
                    (b['x_cm']-cube['x_cm'])**2 + (b['y_cm']-cube['y_cm'])**2)
                pick_and_place(cube['x_cm'], cube['y_cm'],
                               best_bowl['x_cm'], best_bowl['y_cm'])

        # Manual trigger 
        if key == ord(' ') and len(cubes) > 0 and len(bowls) > 0 and H is not None:
            all_sent = True
            print(f"\n  Manual trigger — {len(cubes)} cube(s) {len(bowls)} bowl(s)")
            for cube in cubes:
                best_bowl = min(bowls, key=lambda b:
                    (b['x_cm']-cube['x_cm'])**2 + (b['y_cm']-cube['y_cm'])**2)
                pick_and_place(cube['x_cm'], cube['y_cm'],
                               best_bowl['x_cm'], best_bowl['y_cm'])


    cap.release()
    if ser:
        ser.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
