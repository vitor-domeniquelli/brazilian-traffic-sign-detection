# robocar-sinalizacao

Detecção de placas de trânsito para navegação autônoma de um robô móvel. Um robô com ESP32-CAM transmite vídeo por Wi-Fi a um host externo, que executa a detecção e devolve comandos de manobra.

Projeto final da disciplina Tópicos Avançados de IA (UFABC).

## Estrutura
`host/` executa em tempo real, com o robô ligado
`firmware/` sketch da ESP32-CAM (captura, Wi-Fi e motores)
`training/` notebooks de treino (Colab com GPU) e artefatos das execuções
`analise/` processamento dos experimentos e comparação de latência
`models/` pesos treinados
`data/` referências, execuções com o robô e vídeo da competição


## Modelos

| Arquivo | Descrição |
|---|---|
| `models/yolov11n.pt` | YOLO11n, usado nos experimentos com o robô |
| `models/yolov8n.pt` | YOLOv8n, treinado para comparação entre gerações |
| `models/mlp.keras` | MLP do baseline de duas etapas |
| `models/cnn.keras` | CNN do baseline de duas etapas |

As seis classes do projeto: R-25a (virar à esquerda), R-25b (virar à direita), R-26 (seguir em frente), R-3 (sentido proibido), R-4a (proibido virar à esquerda) e R-4b (proibido virar à direita).

## Dados

O conjunto próprio, com 1.232 imagens capturadas pela ESP32-CAM em cinco sessões, está hospedado no Roboflow e é baixado pelos notebooks de treino via API. Não é versionado aqui. Disponível no link: https://universe.roboflow.com/vitor-chagas/robocar-dataset-c

`data/referencias/` contém as seis imagens de referência que geram o conjunto sintético do baseline.

`data/experimentos/` contém as 60 execuções com o robô, organizadas por classe. Cada execução tem `frames.csv` (uma linha por quadro, com detecção e tempos por etapa), `meta.json` (veredito da execução) e `video.mp4`.

`data/video_robocar.mp4` é o vídeo gravado na competição, usado na comparação de latência entre modelos.

## Reproduzir

Os notebooks rodam no Google Colab com GPU. Os scripts em `host/` rodam localmente, com o robô ligado e conectado à rede Wi-Fi criada pela ESP32-CAM.

### 1. Treinar os detectores

Abra `training/treino_YOLO_v11n.ipynb` no Colab, ative a GPU e execute de cima a baixo. O notebook baixa o conjunto de dados do Roboflow, treina o modelo, avalia sobre o conjunto de teste e gera o vídeo anotado. `training/treino_YOLO_v8n.ipynb` faz o mesmo com a geração anterior, sob configuração idêntica.

Os artefatos de cada execução (`args.yaml`, `results.csv`, curvas e matrizes de confusão) estão em `training/resultados/`.

### 2. Treinar o baseline

Abra `training/baseline_hough_cnn.ipynb` no Colab. Ele sintetiza o conjunto de treino a partir das seis imagens em `data/referencias/`, treina a MLP e a CNN, e exporta os pesos para `models/`.

### 3. Executar com o robô

Grave `firmware/firmware-projeto-ia.ino` na ESP32-CAM. Ao ligar, a placa cria uma rede Wi-Fi própria; conecte o host a ela.

`host/capture_image.py` consome o stream e grava quadros, usado para montar o conjunto de dados.

`host/live-test.py` executa o experimento em malha fechada: lê o stream, roda a detecção, decide a manobra e envia o comando. Cada execução grava `frames.csv`, `meta.json` e `video.mp4` em `data/experimentos/`. As constantes no topo do arquivo (IP do robô, modelo, limiar de disparo, velocidade) devem ser ajustadas antes de rodar.

### 4. Analisar os resultados

Abra `analise/analise_resultados.ipynb`. A primeira seção lê `data/experimentos/` e reproduz as tabelas de decisão correta e de latência por etapa do relatório; roda em qualquer máquina, em segundos. A segunda executa os quatro modelos sobre `data/video_robocar.mp4` e mede o custo de inferência de cada um, exigindo GPU.

## Resultados

- Detecção: mAP@0.5 de 0,9830 (YOLO11n) sobre conjunto de teste em ambiente não observado no treino
- Latência de inferência: 15,5 ms (YOLO11n) contra 68,2 ms do baseline, em GPU T4
- Sistema em operação: 4,4 decisões por segundo, com 50 de 60 execuções emitindo a decisão correta
