# host/main.py
# Experimento de malha fechada: ESP32-CAM -> Wi-Fi -> host (YOLO) -> comando -> robo.
# Um arquivo, um laco. Por corrida salva video.mp4, frames.csv e meta.json.

import csv
import json
import time
from pathlib import Path
from datetime import datetime

import cv2
import requests
from ultralytics import YOLO

# ===================== CONFIGURACAO (ajuste aqui) =========================

IP_ROBO = "192.168.4.1"
STREAM  = f"http://{IP_ROBO}:81/stream"
URL_CMD = f"http://{IP_ROBO}/cmd"

MODELO   = "models/yolov11n.pt"
CONF_MIN = 0.70

# Calibrar em MANUAL: posicione o robo na distancia de virar e leia o
# bbox_w impresso na tela.
LIMIAR_BBOX_W = 90        # px

VELOCIDADE        = 180
TIMEOUT_CORRIDA_S = 10     # corrida sem decisao morre aqui
REENVIO_MS        = 300    # firmware para sozinho apos 900 ms sem comando
FLIP              = True   # mesmo espelhamento do coletar_imagens.py
FPS_VIDEO         = 7

# Quanto tempo cada acao dura, em ms. Cronometre e ajuste.
DURACAO = {"esq": 800, "dir": 800, "tras": 1000, "frente": 1500}

RAIZ = Path("data/experimentos")

# ===================== TECLAS E MAPEAMENTOS ===============================

# Mesmo mapa do coletar_imagens.py
CLASSES = {
    ord("a"): "R-26_frente",
    ord("s"): "R-25b_direita",
    ord("d"): "R-25a_esquerda",
    ord("f"): "R-3_sentido-proibido",
    ord("g"): "R-4a_proibido-esquerda",
    ord("h"): "R-4b_proibido-direita",
}

# Direcao manual, para posicionar o robo antes da largada
MANUAL = {
    ord("i"): "frente",
    ord("k"): "tras",
    ord("j"): "esq",
    ord("l"): "dir",
    ord(" "): "parar",
}

# Nome da classe NO MODELO -> comando. Conferir contra os nomes do data.yaml.
# R-4a proibe a esquerda, entao vira a direita; R-4b o inverso.
NOME_MODELO = {
    "R-25a": "esq",
    "R-25b": "dir",
    "R-26":  "frente",
    "R-3":   "tras",
    "R-4a":  "dir",
    "R-4b":  "esq",
}

# Comando esperado por classe da pasta (para conferir a decisao sozinho)
COMANDO_CORRETO = {
    "R-26_frente":            "frente",
    "R-25b_direita":          "dir",
    "R-25a_esquerda":         "esq",
    "R-3_sentido-proibido":   "tras",
    "R-4a_proibido-esquerda": "dir",
    "R-4b_proibido-direita":  "esq",
}

COLUNAS = ["frame_id", "t_rel_s", "t_captura_ms", "t_inferencia_ms",
           "t_decisao_ms", "t_comando_ms", "classe_detectada", "confianca",
           "bbox_w", "bbox_h", "estado", "comando_enviado"]

# ===================== INICIALIZACAO ======================================

modelo = YOLO(MODELO)

#cap = cv2.VideoCapture(STREAM)
#cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
import threading

class LeitorStream:
    # Le o stream sem parar numa thread separada e guarda so o frame mais
    # recente. O descarte do que ficou velho acontece por sobrescrita: nao
    # existe fila para o laco principal consumir.
    def __init__(self, url):
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            raise SystemExit(f"Nao abriu o stream: {url}")
        self.frame = None
        self.rodando = True
        # daemon: thread que nao impede o programa de encerrar
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.rodando:
            ok, f = self.cap.read()
            if ok:
                self.frame = f    # atribuicao simples e atomica: dispensa lock

    def read(self):
        f = self.frame
        return (f is not None), f

    def release(self):
        self.rodando = False
        self.cap.release()

cap = LeitorStream(STREAM)


#if not cap.isOpened():
#    raise SystemExit(f"Nao abriu o stream: {STREAM}")

classe_alvo = "R-25a_esquerda"
estado = "MANUAL"

pasta = None
nome_run = None
video = None
linhas = []
frame_id = 0
t_ini = 0.0
t_cap_anterior = None
comando_ativo = None
ultimo_envio = 0.0
comando_emitido = None
classe_decidida = None
conf_decidida = None
t_fim_acao = 0.0
t_fim_corrida = None

print("MANUAL: i/k/j/l = mover | ESPACO = parar")
print("classe: a=R-26  s=R-25b  d=R-25a  f=R-3  g=R-4a  h=R-4b")
print("ENTER = largar | s/n = veredito no fim | q = sair")

# ===================== LACO PRINCIPAL =====================================

while True:
    ok, frame = cap.read()
    if not ok:
        continue
    t_cap = time.perf_counter()
    if FLIP:
        frame = cv2.flip(frame, 1)

    # (1) captura + transmissao: intervalo entre frames que chegaram.
    # Agregado de proposito: separar exigiria sincronizar o relogio da ESP32.
    t_captura_ms = None
    if t_cap_anterior is not None:
        t_captura_ms = (t_cap - t_cap_anterior) * 1000.0
    t_cap_anterior = t_cap

    # (2) inferencia
    t0 = time.perf_counter()
    r = modelo.predict(frame, conf=CONF_MIN, verbose=False)[0]
    t_inferencia_ms = (time.perf_counter() - t0) * 1000.0

    # so a deteccao mais confiante do frame
    nome_det = ""
    conf_det = 0.0
    x1 = y1 = x2 = y2 = 0
    tem_det = False
    if len(r.boxes):
        i = int(r.boxes.conf.argmax())
        x1, y1, x2, y2 = map(int, r.boxes.xyxy[i].tolist())
        conf_det = float(r.boxes.conf[i])
        nome_det = r.names[int(r.boxes.cls[i])]
        tem_det = True

    # (3) decisao
    t0 = time.perf_counter()
    agora = time.perf_counter()

    if estado == "PROCURANDO":
        comando_ativo = "frente"
        if tem_det:
            estado = "APROXIMANDO"

    elif estado == "APROXIMANDO":
        comando_ativo = "frente"
        if not tem_det:
            estado = "PROCURANDO"
        elif (x2 - x1) >= LIMIAR_BBOX_W:
            # Decisao tomada UMA vez e travada. Se continuasse aberta, a placa
            # saindo do quadro faria o robo mudar de ideia no meio da manobra.
            comando_emitido = None
            for chave in sorted(NOME_MODELO, key=len, reverse=True):
                if nome_det.startswith(chave):      # do mais longo pro mais
                    comando_emitido = NOME_MODELO[chave]   # curto, senao R-4a
                    break                                   # cairia em R-4
            classe_decidida = nome_det
            conf_decidida = conf_det
            if comando_emitido is None:
                # Nome do modelo fora do mapa: encerra em vez de seguir reto,
                # senao o robo andaria ate o timeout e sairia da mesa.
                print(f"!! classe '{nome_det}' nao esta em NOME_MODELO")
                estado = "VEREDITO"
            else:
                comando_ativo = comando_emitido
                t_fim_acao = agora + DURACAO[comando_emitido] / 1000.0
                estado = "AGINDO"

    elif estado == "AGINDO":
        # A inferencia segue rodando e sendo registrada (evidencia do que o
        # modelo via durante a manobra), mas nao muda mais a decisao.
        if agora >= t_fim_acao:
            estado = "VEREDITO"

    if estado in ("PROCURANDO", "APROXIMANDO", "AGINDO") and \
       (agora - t_ini) > TIMEOUT_CORRIDA_S:
        estado = "VEREDITO"

    t_decisao_ms = (time.perf_counter() - t0) * 1000.0

    # (4) comando: reenvio periodico mantem o failsafe do firmware satisfeito
    t_comando_ms = None
    comando_enviado = ""

    # Fim da corrida: para NA HORA. Se dependesse do failsafe, os motores
    # rodariam ate 900 ms alem do fim da manobra e a duracao calibrada
    # nao teria relacao com o giro real.
    if estado == "VEREDITO" and t_fim_corrida is None:
        t_fim_corrida = agora
        t0 = time.perf_counter()
        try:
            requests.get(URL_CMD, params={"dir": "parar",
                                          "vel": VELOCIDADE}, timeout=1.0)
        except requests.RequestException:
            pass
        t_comando_ms = (time.perf_counter() - t0) * 1000.0
        comando_enviado = "parar"
        comando_ativo = "parar"

    if estado in ("PROCURANDO", "APROXIMANDO", "AGINDO") and \
       (agora - ultimo_envio) * 1000.0 >= REENVIO_MS:
        t0 = time.perf_counter()
        try:
            requests.get(URL_CMD, params={"dir": comando_ativo,
                                          "vel": VELOCIDADE}, timeout=1.0)
        except requests.RequestException:
            pass
        t_comando_ms = (time.perf_counter() - t0) * 1000.0
        comando_enviado = comando_ativo
        ultimo_envio = agora

    # ---------------- desenho ----------------
    vis = frame.copy()
    if tem_det:
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(vis, f"{nome_det} {conf_det:.2f}", (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cor = (0, 0, 255) if (x2 - x1) >= LIMIAR_BBOX_W else (0, 255, 255)
        cv2.putText(vis, f"bbox_w {x2 - x1}  (limiar {LIMIAR_BBOX_W})",
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, cor, 2)

    cv2.putText(vis, f"[{estado}]", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(vis, f"alvo: {classe_alvo}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(vis, f"cmd: {comando_ativo or '-'}", (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    if estado == "VEREDITO":
        cv2.putText(vis, "manobra executada?  s / n", (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # ---------------- registro ----------------
    if estado in ("PROCURANDO", "APROXIMANDO", "AGINDO"):
        frame_id += 1
        linhas.append({
            "frame_id": frame_id,
            "t_rel_s": round(agora - t_ini, 3),
            "t_captura_ms": round(t_captura_ms, 2) if t_captura_ms is not None else "",
            "t_inferencia_ms": round(t_inferencia_ms, 2),
            "t_decisao_ms": round(t_decisao_ms, 3),
            "t_comando_ms": round(t_comando_ms, 2) if t_comando_ms is not None else "",
            "classe_detectada": nome_det,
            "confianca": round(conf_det, 3) if tem_det else "",
            "bbox_w": x2 - x1 if tem_det else "",
            "bbox_h": y2 - y1 if tem_det else "",
            "estado": estado,
            "comando_enviado": comando_enviado,
        })
        if video is not None:
            video.write(vis)

    cv2.imshow("experimento", vis)
    k = cv2.waitKey(1) & 0xFF

    # ---------------- teclas ----------------
    if k == ord("q"):
        break

    if estado == "VEREDITO" and k in (ord("s"), ord("n")):
        # Robo ja parou no primeiro frame deste estado; aqui so grava.
        if video is not None:
            video.release()      # o mp4 so ganha cabecalho valido aqui
            video = None

        with open(pasta / "frames.csv", "w", newline="") as fp:
            w = csv.DictWriter(fp, fieldnames=COLUNAS)
            w.writeheader()
            w.writerows(linhas)

        with open(pasta / "meta.json", "w") as fp:
            json.dump({
                "run": nome_run,
                "classe_esperada": classe_alvo,
                "comando_esperado": COMANDO_CORRETO[classe_alvo],
                "classe_detectada": classe_decidida,
                "confianca_decisao": conf_decidida,
                "comando_emitido": comando_emitido,
                "decisao_correta": comando_emitido == COMANDO_CORRETO[classe_alvo],
                "manobra_executada": k == ord("s"),
                "datahora": datetime.now().isoformat(timespec="seconds"),
                "duracao_corrida_s": round(t_fim_corrida - t_ini, 2),
                "n_frames": frame_id,
                "modelo": MODELO,
                "conf_min": CONF_MIN,
                "limiar_bbox_w": LIMIAR_BBOX_W,
                "duracao_acao_ms": DURACAO.get(comando_emitido),
                "velocidade": VELOCIDADE,
                "flip": FLIP,
            }, fp, indent=2, ensure_ascii=False)

        print(f"  salvo: {pasta} | emitido={comando_emitido} "
              f"| esperado={COMANDO_CORRETO[classe_alvo]}")
        linhas = []
        frame_id = 0
        comando_ativo = None
        comando_emitido = classe_decidida = conf_decidida = None
        t_fim_corrida = None
        estado = "MANUAL"
        continue

    if estado == "MANUAL":
        if k in CLASSES:
            classe_alvo = CLASSES[k]
            print(f"classe alvo -> {classe_alvo}")

        elif k in MANUAL:
            try:
                requests.get(URL_CMD, params={"dir": MANUAL[k],
                                              "vel": VELOCIDADE}, timeout=1.0)
            except requests.RequestException:
                pass

        elif k in (10, 13):   # ENTER = largada (10 ou 13 dependendo do SO)
            base = RAIZ / classe_alvo
            base.mkdir(parents=True, exist_ok=True)
            n = 1
            while (base / f"run{n:02d}").exists():
                n += 1
            nome_run = f"run{n:02d}"
            pasta = base / nome_run
            pasta.mkdir()
            altura, largura = frame.shape[:2]
            video = cv2.VideoWriter(str(pasta / "video.mp4"),
                                    cv2.VideoWriter_fourcc(*"mp4v"),
                                    FPS_VIDEO, (largura, altura))
            linhas = []
            frame_id = 0
            t_ini = time.perf_counter()
            ultimo_envio = 0.0
            t_fim_corrida = None
            estado = "PROCURANDO"
            print(f"LARGADA {classe_alvo} / {nome_run}")

# ===================== SAIDA ==============================================

try:
    requests.get(URL_CMD, params={"dir": "parar", "vel": VELOCIDADE}, timeout=1.0)
except requests.RequestException:
    pass
if video is not None:
    video.release()
cap.release()
cv2.destroyAllWindows()