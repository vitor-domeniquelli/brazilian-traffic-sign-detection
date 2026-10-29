"""
Collects labeled images from the ESP32-CAM stream.

Images are saved under data/raw/<session>/<class>/. A session is a set of
captures taken under the same lighting and background, and the dataset is
split into train/valid/test by session rather than by individual image.

Keys: a/s/d/f/g/h/j/k = class | z/x/c/v/b = session | SPACE = save | q = quit
"""

import cv2
from pathlib import Path
from datetime import datetime

STREAM = "http://192.168.4.1:81/stream"
ROOT = Path("data/raw")

# Class names match the labels used in the published dataset
CLASSES = {
    ord("a"): "R-26_frente",
    ord("s"): "R-25b_direita",
    ord("d"): "R-25a_esquerda",
    ord("f"): "R-3_sentido-proibido",
    ord("g"): "R-4a_proibido-esquerda",
    ord("h"): "R-4b_proibido-direita",
    ord("j"): "Vazio",        # no sign in view
    ord("k"): "Multiplaca",   # more than one sign in view
}

SESSIONS = {
    ord("z"): "s01",
    ord("x"): "s02",
    ord("c"): "s03",
    ord("v"): "s04",
    ord("b"): "s05",
}

cap = cv2.VideoCapture(STREAM)
if not cap.isOpened():
    raise SystemExit(f"Could not open stream: {STREAM}")

label = "R-26_frente"
session = "s01"
saved = {}   # (session, class) -> count

print("a/s/d/f/g/h/j/k = class | z/x/c/v/b = session | SPACE = save | q = quit")

while True:
    ok, frame = cap.read()
    if not ok:
        continue

    n = saved.get((session, label), 0)
    frame = cv2.flip(frame, 1)
    view = frame.copy()

    cv2.putText(view, f"{session}  {label}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(view, f"saved in this session/class: {n}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imshow("capture", view)

    k = cv2.waitKey(1) & 0xFF

    if k in CLASSES:
        label = CLASSES[k]
        print(f"class -> {label}")

    elif k in SESSIONS:
        session = SESSIONS[k]
        print(f"session -> {session}")

    elif k == ord(" "):
        folder = ROOT / session / label
        folder.mkdir(parents=True, exist_ok=True)
        # The session is also in the file name so that the origin of each
        # image stays traceable after export to other tools
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        cv2.imwrite(str(folder / f"{session}_{label}_{ts}.jpg"), frame)
        saved[(session, label)] = n + 1
        print(f"  [{session}] {label}: {n + 1}")

    elif k == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()