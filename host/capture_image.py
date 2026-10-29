# training/coletar_imagens.py
# Captura frames do stream da ESP32-CAM, separando por classe e por sessão.
# Sessão = conjunto contíguo de capturas com mesma iluminação/cômodo/fundo.
# O split treino/valid/teste será feito POR SESSÃO, nunca por imagem sorteada.

import cv2
from pathlib import Path
from datetime import datetime

STREAM = "http://192.168.4.1:81/stream"   # <- IP ESP32
RAIZ = Path("data/raw")

CLASSES = {
    ord("a"): "R-26_frente",
    ord("s"): "R-25b_direita",
    ord("d"): "R-25a_esquerda",
    ord("f"): "R-3_sentido-proibido",
    ord("g"): "R-4a_proibido-esquerda",
    ord("h"): "R-4b_proibido-direita",
    ord("j"): "Vazio",
    ord("k"): "Multiplaca",

}

SESSOES = {
    ord("z"): "s01",
    ord("x"): "s02",
    ord("c"): "s03",
    ord("v"): "s04",
    ord("b"): "s05",
}

cap = cv2.VideoCapture(STREAM)
if not cap.isOpened():
    raise SystemExit(f"Não abriu o stream: {STREAM}")

classe = "R-26_frente"
sessao = "s01"
salvos = {}   # (sessao, classe) -> contagem

print("a/s/d/f/g/h/j = classe | z/x/c/v/b = sessão | ESPAÇO = salvar | q = sair")

while True:
    ok, frame = cap.read()
    if not ok:
        continue

    n = salvos.get((sessao, classe), 0)
    vis = frame.copy()
    vis = cv2.flip(vis,1)
    frame = cv2.flip(frame,1)

    cv2.putText(vis, f"{sessao}  {classe}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(vis, f"nesta sessao/classe: {n}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imshow("coleta", vis)

    k = cv2.waitKey(1) & 0xFF

    if k in CLASSES:
        classe = CLASSES[k]
        print(f"classe -> {classe}")

    elif k in SESSOES:
        sessao = SESSOES[k]
        print(f"SESSAO -> {sessao}")

    elif k == ord(" "):
        # Pasta por sessão e classe: o split por sessão vira um mv de pasta.
        pasta = RAIZ / sessao / classe
        pasta.mkdir(parents=True, exist_ok=True)
        # Sessão também no nome do arquivo: se as imagens forem misturadas
        # depois (Roboflow, export), a origem continua rastreável.
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        cv2.imwrite(str(pasta / f"{sessao}_{classe}_{ts}.jpg"), frame)
        salvos[(sessao, classe)] = n + 1
        print(f"  [{sessao}] {classe}: {n + 1}")

    elif k == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()