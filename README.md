# Brazilian Traffic Sign Detection on a Low-Cost Mobile Robot

Code and data for the paper *Can You See the Signs? Cross-Environment Brazilian Traffic Sign Detection with YOLO on a Low-Cost Mobile Robot*.

A four-wheel robot with an ESP32-CAM streams video over Wi-Fi to an external host. The host runs a YOLO detector on each frame and sends motion commands back to the robot. The repository contains the robot firmware, the host scripts, the training and analysis notebooks, the trained models and the data recorded in 60 runs of the robot.

## Repository structure

| Folder | Contents |
|---|---|
| `firmware/` | ESP32-CAM firmware (video stream, Wi-Fi access point and motor control) |
| `host/` | Scripts that run on the inference host, for image capture and closed-loop runs |
| `training/` | Training notebooks (Google Colab) and the results of each training run |
| `analysis/` | Notebook that reproduces the operation results and the inference cost comparison |
| `models/` | Trained weights |
| `data/` | Reference sign images, robot runs and reference video |

## Signs

| Code | Sign | Class label |
|---|---|---|
| R-25a | Turn left | `R-25a_esquerda` |
| R-25b | Turn right | `R-25b_direita` |
| R-26 | Go straight | `R-26_frente` |
| R-3 | No entry | `R-3_sentido-proibido` |
| R-4a | No left turn | `R-4a_proibido-esquerda` |
| R-4b | No right turn | `R-4b_proibido-direita` |

Class labels keep the Portuguese names used in the dataset and in the recorded data.

## Dataset

The dataset has 1,232 images captured by the robot's ESP32-CAM in five sessions, each in a different environment. Sessions 1 to 3 are used for training, session 4 for validation and session 5 for testing. It is hosted on Roboflow Universe under the CC BY 4.0 license and is downloaded by the notebooks through the Roboflow API.

https://universe.roboflow.com/vitor-chagas/robocar-dataset-c (version 2)

Downloading requires a Roboflow API key, which any free account provides. The notebooks ask for the key when they run.

## Models

| File | Description |
|---|---|
| `models/yolov11n.pt` | YOLO11n, used in the robot runs |
| `models/yolov8n.pt` | YOLOv8n, trained with the same configuration for comparison |
| `models/mlp.keras` | MLP classifier of the Hough baseline |
| `models/cnn.keras` | CNN classifier of the Hough baseline |

## Reproducing the results

The notebooks run on Google Colab with a GPU runtime. The analysis notebook also runs locally from the repository root.

| Result in the paper | Source |
|---|---|
| Training of YOLO11n and YOLOv8n (Table II) | `training/train_yolo11n.ipynb`, `training/train_yolov8n.ipynb` |
| Test set metrics and confusion matrix (Table III, Fig. 9) | `training/results/` |
| Hough baseline | `training/baseline_hough_cnn.ipynb` |
| Inference cost on the reference video (Table IV) | `analysis/results_analysis.ipynb`, section 3 |
| Loop latency in operation (Table V) | `analysis/results_analysis.ipynb`, section 2.1 |
| Detections in operation (Table VI) | `analysis/results_analysis.ipynb`, section 2.2 |
| Bounding box widths (Fig. 10) | `analysis/results_analysis.ipynb`, section 2.3 |

`training/results/` holds, for each detector, the training run (`A_v11n`, `A_v8n`), the evaluation on the test set (`A_val_test`) and the `data.yaml` that identifies the dataset version.

Inference times measured on a GPU vary slightly between runs, so section 3 of the analysis notebook gives values close to, but not identical to, those in Table IV.

## Running the robot

### Hardware

KUONGSHUN AD174 four-wheel kit with an ESP32-CAM (AI Thinker, OV3660 sensor), an L298N motor driver and two 18650 cells in series. The two motors on each side are driven together.

| Function | ESP32-CAM GPIO |
|---|---|
| Left motors, forward | 15 |
| Left motors, backward | 33 |
| Right motors, forward | 13 |
| Right motors, backward | 14 |
| Left motors, enable (PWM) | 2 |
| Right motors, enable (PWM) | 12 |
| Light | 4 |

### Firmware

Built with the Arduino IDE and the arduino-esp32 core 3.3.10, board "ESP32 Dev Module" with PSRAM enabled. At startup the board creates the Wi-Fi network `ROBOT-AP` and serves:

| Endpoint | Description |
|---|---|
| `http://192.168.4.1/` | Manual control page |
| `http://192.168.4.1/cmd?dir=<forward,backward,left,right,stop>&speed=<0-255>` | Motion command |
| `http://192.168.4.1/light?level=<0-80>` | Light |
| `http://192.168.4.1/photo` | Single JPEG frame |
| `http://192.168.4.1:81/stream` | MJPEG stream, 640 × 480 |

The motors stop if no command arrives within 900 ms.

### Host

Requires Python with `ultralytics`, `opencv-python` and `requests`. Connect the host to `ROBOT-AP` and run the scripts from the repository root.

- `host/capture_image.py` saves labeled frames from the stream, organized by session and class.
- `host/live_test.py` runs the closed-loop experiment. For each run it saves `video.mp4`, `frames.csv` and `meta.json` to `data/experiments/<class>/runNN/`. The keys are listed at the top of the file.

## Recorded data

`data/experiments/` holds the 60 runs used in the paper, 10 per sign. Each run has

- `frames.csv`, one row per loop iteration, with stage timings and the detection of that frame
- `meta.json`, the run summary and settings
- `video.mp4`, the annotated frames

These runs were recorded with an earlier version of `host/live_test.py` that used Portuguese field names. The current version writes the English names below.

**`frames.csv`**

| Recorded name | Current name | Description |
|---|---|---|
| `frame_id` | `frame_id` | Iteration number within the run |
| `t_rel_s` | `t_rel_s` | Time since the start of the run (s) |
| `t_captura_ms` | `t_capture_ms` | Interval since the previous iteration, that is, the loop period (ms) |
| `t_inferencia_ms` | `t_inference_ms` | Detector inference time (ms) |
| `t_decisao_ms` | `t_decision_ms` | Decision logic time (ms) |
| `t_comando_ms` | `t_command_ms` | Command dispatch time, empty when no command was sent (ms) |
| `classe_detectada` | `detected_class` | Class of the most confident detection, empty if none |
| `confianca` | `confidence` | Confidence of that detection |
| `bbox_w`, `bbox_h` | `bbox_w`, `bbox_h` | Width and height of the detection (px) |
| `estado` | `state` | State of the run |
| `comando_enviado` | `command_sent` | Command sent in this iteration, empty if none |

**`meta.json`**

| Recorded name | Current name |
|---|---|
| `classe_esperada` | `expected_class` |
| `comando_esperado` | `expected_command` |
| `classe_detectada` | `detected_class` |
| `confianca_decisao` | `decision_confidence` |
| `comando_emitido` | `issued_command` |
| `decisao_correta` | `correct_decision` |
| `manobra_executada` | `maneuver_executed` |
| `datahora` | `datetime` |
| `duracao_corrida_s` | `run_duration_s` |
| `modelo` | `model` |
| `limiar_bbox_w` | `bbox_w_threshold` |
| `duracao_acao_ms` | `action_duration_ms` |
| `velocidade` | `speed` |

`run`, `n_frames`, `conf_min` and `flip` keep the same name.

**Values**

| Field | Recorded values | Current values |
|---|---|---|
| state | `PROCURANDO`, `APROXIMANDO`, `AGINDO` | `SEARCHING`, `APPROACHING`, `ACTING` |
| command | `frente`, `tras`, `esq`, `dir`, `parar` | `forward`, `backward`, `left`, `right`, `stop` |

`data/reference_signs/` holds the six reference images used to generate the synthetic dataset of the Hough baseline. `data/reference_video.mp4` is the reference video, recorded in another environment with a different camera, used for the inference cost comparison.
