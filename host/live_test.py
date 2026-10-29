"""
Closed-loop experiment: ESP32-CAM -> Wi-Fi -> host (YOLO) -> command -> robot.

Each run is saved to data/experiments/<class>/runNN/ as video.mp4 (annotated
frames), frames.csv (per-frame timing and detection) and meta.json (run
summary and settings).

MANUAL state keys:
  i/k/j/l = move | SPACE = stop | a/s/d/f/g/h = target class
  ENTER = start run | q = quit
VERDICT state keys:
  y/n = maneuver executed or not (saves the run)
"""

import csv
import json
import time
import threading
from pathlib import Path
from datetime import datetime

import cv2
import requests
from ultralytics import YOLO

# ------------------------------ Settings ------------------------------

ROBOT_IP = "192.168.4.1"
STREAM   = f"http://{ROBOT_IP}:81/stream"
CMD_URL  = f"http://{ROBOT_IP}/cmd"

MODEL    = "models/yolov11n.pt"
CONF_MIN = 0.70

# Bounding box width at which the robot acts. Calibrated in MANUAL state by
# placing the robot at the turning distance and reading bbox_w on screen.
BBOX_W_THRESHOLD = 90     # px

SPEED          = 180
RUN_TIMEOUT_S  = 10       # a run without a decision ends here
RESEND_MS      = 300      # the firmware stops the motors after 900 ms without a command
FLIP           = True     # same mirroring used in capture_image.py
VIDEO_FPS      = 7

# Duration of each action, in ms
ACTION_DURATION = {"left": 800, "right": 800, "backward": 1000, "forward": 1500}

ROOT = Path("data/experiments")

# ------------------------- Keys and mappings --------------------------

# Same keys and labels as capture_image.py
CLASSES = {
    ord("a"): "R-26_frente",
    ord("s"): "R-25b_direita",
    ord("d"): "R-25a_esquerda",
    ord("f"): "R-3_sentido-proibido",
    ord("g"): "R-4a_proibido-esquerda",
    ord("h"): "R-4b_proibido-direita",
}

# Manual driving, used to position the robot before each run
MANUAL = {
    ord("i"): "forward",
    ord("k"): "backward",
    ord("j"): "left",
    ord("l"): "right",
    ord(" "): "stop",
}

# Model class name -> command.
# R-4a prohibits turning left, so the robot turns right; R-4b the opposite.
MODEL_CLASS_COMMAND = {
    "R-25a": "left",
    "R-25b": "right",
    "R-26":  "forward",
    "R-3":   "backward",
    "R-4a":  "right",
    "R-4b":  "left",
}

# Expected command for each target class, used to score the decision
EXPECTED_COMMAND = {
    "R-26_frente":            "forward",
    "R-25b_direita":          "right",
    "R-25a_esquerda":         "left",
    "R-3_sentido-proibido":   "backward",
    "R-4a_proibido-esquerda": "right",
    "R-4b_proibido-direita":  "left",
}

COLUMNS = ["frame_id", "t_rel_s", "t_capture_ms", "t_inference_ms",
           "t_decision_ms", "t_command_ms", "detected_class", "confidence",
           "bbox_w", "bbox_h", "state", "command_sent"]

# ------------------------------- Setup --------------------------------

model = YOLO(MODEL)


class StreamReader:
    # Reads the stream continuously in a background thread and keeps only the
    # latest frame, so the main loop always processes the most recent image
    # instead of a queue of old ones.
    def __init__(self, url):
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            raise SystemExit(f"Could not open stream: {url}")
        self.frame = None
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if ok:
                self.frame = f

    def read(self):
        f = self.frame
        return (f is not None), f

    def release(self):
        self.running = False
        self.cap.release()


cap = StreamReader(STREAM)

target_class = "R-25a_esquerda"
state = "MANUAL"

folder = None
run_name = None
video = None
rows = []
frame_id = 0
t_start = 0.0
t_prev_capture = None
active_command = None
last_send = 0.0
issued_command = None
decided_class = None
decided_conf = None
t_action_end = 0.0
t_run_end = None

print("MANUAL: i/k/j/l = move | SPACE = stop")
print("class: a=R-26  s=R-25b  d=R-25a  f=R-3  g=R-4a  h=R-4b")
print("ENTER = start run | y/n = verdict at the end | q = quit")

# ----------------------------- Main loop ------------------------------

while True:
    ok, frame = cap.read()
    if not ok:
        continue
    t_capture = time.perf_counter()
    if FLIP:
        frame = cv2.flip(frame, 1)

    # (1) Capture + transmission
    t_capture_ms = None
    if t_prev_capture is not None:
        t_capture_ms = (t_capture - t_prev_capture) * 1000.0
    t_prev_capture = t_capture

    # (2) Inference
    t0 = time.perf_counter()
    r = model.predict(frame, conf=CONF_MIN, verbose=False)[0]
    t_inference_ms = (time.perf_counter() - t0) * 1000.0

    # Keep only the most confident detection in the frame
    det_name = ""
    det_conf = 0.0
    x1 = y1 = x2 = y2 = 0
    has_det = False
    if len(r.boxes):
        i = int(r.boxes.conf.argmax())
        x1, y1, x2, y2 = map(int, r.boxes.xyxy[i].tolist())
        det_conf = float(r.boxes.conf[i])
        det_name = r.names[int(r.boxes.cls[i])]
        has_det = True

    # (3) Decision
    t0 = time.perf_counter()
    now = time.perf_counter()

    if state == "SEARCHING":
        active_command = "forward"
        if has_det:
            state = "APPROACHING"

    elif state == "APPROACHING":
        active_command = "forward"
        if not has_det:
            state = "SEARCHING"
        elif (x2 - x1) >= BBOX_W_THRESHOLD:
            # The decision is made once and locked, so that the sign leaving
            # the frame during the maneuver cannot change it.
            issued_command = None
            # Longest prefix first, so that "R-4a" is not matched as "R-4"
            for key in sorted(MODEL_CLASS_COMMAND, key=len, reverse=True):
                if det_name.startswith(key):
                    issued_command = MODEL_CLASS_COMMAND[key]
                    break
            decided_class = det_name
            decided_conf = det_conf
            if issued_command is None:
                # Unknown class name: end the run instead of driving forward
                # until the timeout
                print(f"Class '{det_name}' is not in MODEL_CLASS_COMMAND")
                state = "VERDICT"
            else:
                active_command = issued_command
                t_action_end = now + ACTION_DURATION[issued_command] / 1000.0
                state = "ACTING"

    elif state == "ACTING":
        # Inference keeps running and being logged during the maneuver, but
        # no longer affects the decision
        if now >= t_action_end:
            state = "VERDICT"

    if state in ("SEARCHING", "APPROACHING", "ACTING") and \
       (now - t_start) > RUN_TIMEOUT_S:
        state = "VERDICT"

    t_decision_ms = (time.perf_counter() - t0) * 1000.0

    # (4) Command
    t_command_ms = None
    command_sent = ""

    # Stop now instead of waiting up to 900 ms for the firmware failsafe
    if state == "VERDICT" and t_run_end is None:
        t_run_end = now
        t0 = time.perf_counter()
        try:
            requests.get(CMD_URL, params={"dir": "stop", "speed": SPEED}, timeout=1.0)
        except requests.RequestException:
            pass
        t_command_ms = (time.perf_counter() - t0) * 1000.0
        command_sent = "stop"
        active_command = "stop"

    # Periodic resend keeps the firmware failsafe from stopping the motors
    if state in ("SEARCHING", "APPROACHING", "ACTING") and \
       (now - last_send) * 1000.0 >= RESEND_MS:
        t0 = time.perf_counter()
        try:
            requests.get(CMD_URL, params={"dir": active_command, "speed": SPEED},
                         timeout=1.0)
        except requests.RequestException:
            pass
        t_command_ms = (time.perf_counter() - t0) * 1000.0
        command_sent = active_command
        last_send = now

    # ----------------------------- Display ----------------------------
    view = frame.copy()
    if has_det:
        cv2.rectangle(view, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(view, f"{det_name} {det_conf:.2f}", (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        color = (0, 0, 255) if (x2 - x1) >= BBOX_W_THRESHOLD else (0, 255, 255)
        cv2.putText(view, f"bbox_w {x2 - x1}  (threshold {BBOX_W_THRESHOLD})",
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    cv2.putText(view, f"[{state}]", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(view, f"target: {target_class}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(view, f"cmd: {active_command or '-'}", (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    if state == "VERDICT":
        cv2.putText(view, "maneuver executed?  y / n", (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # ------------------------------ Logging ---------------------------
    if state in ("SEARCHING", "APPROACHING", "ACTING"):
        frame_id += 1
        rows.append({
            "frame_id": frame_id,
            "t_rel_s": round(now - t_start, 3),
            "t_capture_ms": round(t_capture_ms, 2) if t_capture_ms is not None else "",
            "t_inference_ms": round(t_inference_ms, 2),
            "t_decision_ms": round(t_decision_ms, 3),
            "t_command_ms": round(t_command_ms, 2) if t_command_ms is not None else "",
            "detected_class": det_name,
            "confidence": round(det_conf, 3) if has_det else "",
            "bbox_w": x2 - x1 if has_det else "",
            "bbox_h": y2 - y1 if has_det else "",
            "state": state,
            "command_sent": command_sent,
        })
        if video is not None:
            video.write(view)

    cv2.imshow("experiment", view)
    k = cv2.waitKey(1) & 0xFF

    # ------------------------------- Keys -----------------------------
    if k == ord("q"):
        break

    if state == "VERDICT" and k in (ord("y"), ord("n")):
        # The robot already stopped on the first frame of this state
        if video is not None:
            video.release()      # the mp4 header is only written on release
            video = None

        with open(folder / "frames.csv", "w", newline="") as fp:
            w = csv.DictWriter(fp, fieldnames=COLUMNS)
            w.writeheader()
            w.writerows(rows)

        with open(folder / "meta.json", "w") as fp:
            json.dump({
                "run": run_name,
                "expected_class": target_class,
                "expected_command": EXPECTED_COMMAND[target_class],
                "detected_class": decided_class,
                "decision_confidence": decided_conf,
                "issued_command": issued_command,
                "correct_decision": issued_command == EXPECTED_COMMAND[target_class],
                "maneuver_executed": k == ord("y"),
                "datetime": datetime.now().isoformat(timespec="seconds"),
                "run_duration_s": round(t_run_end - t_start, 2),
                "n_frames": frame_id,
                "model": MODEL,
                "conf_min": CONF_MIN,
                "bbox_w_threshold": BBOX_W_THRESHOLD,
                "action_duration_ms": ACTION_DURATION.get(issued_command),
                "speed": SPEED,
                "flip": FLIP,
            }, fp, indent=2, ensure_ascii=False)

        print(f"  saved: {folder} | issued={issued_command} "
              f"| expected={EXPECTED_COMMAND[target_class]}")
        rows = []
        frame_id = 0
        active_command = None
        issued_command = decided_class = decided_conf = None
        t_run_end = None
        state = "MANUAL"
        continue

    if state == "MANUAL":
        if k in CLASSES:
            target_class = CLASSES[k]
            print(f"target class -> {target_class}")

        elif k in MANUAL:
            try:
                requests.get(CMD_URL, params={"dir": MANUAL[k], "speed": SPEED},
                             timeout=1.0)
            except requests.RequestException:
                pass

        elif k in (10, 13):   # ENTER is 10 or 13 depending on the OS
            base = ROOT / target_class
            base.mkdir(parents=True, exist_ok=True)
            n = 1
            while (base / f"run{n:02d}").exists():
                n += 1
            run_name = f"run{n:02d}"
            folder = base / run_name
            folder.mkdir()
            height, width = frame.shape[:2]
            video = cv2.VideoWriter(str(folder / "video.mp4"),
                                    cv2.VideoWriter_fourcc(*"mp4v"),
                                    VIDEO_FPS, (width, height))
            rows = []
            frame_id = 0
            t_start = time.perf_counter()
            last_send = 0.0
            t_run_end = None
            state = "SEARCHING"
            print(f"START {target_class} / {run_name}")

# ------------------------------ Shutdown ------------------------------

try:
    requests.get(CMD_URL, params={"dir": "stop", "speed": SPEED}, timeout=1.0)
except requests.RequestException:
    pass
if video is not None:
    video.release()
cap.release()
cv2.destroyAllWindows()