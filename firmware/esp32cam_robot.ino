/*
 * ESP32-CAM robot firmware
 *
 * Streams MJPEG video from the onboard camera and drives two DC motor pairs
 * through an L298N, following HTTP commands sent by the inference host.
 *
 * Hardware : ESP32-CAM (AI Thinker) + L298N + 2x 18650 (2S)
 * Board    : "ESP32 Dev Module", PSRAM enabled
 * Core     : arduino-esp32 3.3.10
 *
 * Endpoints (access point at 192.168.4.1)
 *   :80/            manual control page
 *   :80/cmd         ?dir=forward|backward|left|right|stop  &speed=0..255
 *   :80/light       ?level=0..LIGHT_MAX  or  ?on=0|1
 *   :80/photo       single JPEG frame
 *   :81/stream      MJPEG stream
 *
 * LEDC channels are assigned explicitly so that the camera clock, the light
 * and the motors each use a separate timer:
 *   channel 0 / timer 0   camera XCLK (reserved by the camera driver)
 *   channel 2 / timer 1   light (GPIO 4)
 *   channel 4 / timer 2   left motor PWM
 *   channel 6 / timer 3   right motor PWM
 */

#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"

/* Wi-Fi access point */
#define AP_SSID        "ROBOT-AP"
#define AP_PASS        "12345678"
#define AP_CHANNEL     1
#define AP_MAX_CONN    1
#define TX_POWER       WIFI_POWER_11dBm

/* Camera */
#define FRAME_SIZE          FRAMESIZE_VGA
#define JPEG_QUALITY        12
#define STREAM_INTERVAL_MS  150

/* Motors */
#define PIN_L_FORWARD   15
#define PIN_L_BACKWARD  33
#define PIN_R_FORWARD   13
#define PIN_R_BACKWARD  14
#define PIN_EN_L         2
#define PIN_EN_R        12

#define PWM_CHANNEL_L    4
#define PWM_CHANNEL_R    6
#define PWM_FREQ      5000
#define PWM_BITS         8

#define DEFAULT_SPEED      180
#define MIN_SPEED           80
#define RAMP_STEP           12
#define RAMP_INTERVAL_MS    15
#define DEADTIME_MS         40
#define COMMAND_TIMEOUT_MS 900   // motors stop if no command arrives within this time
#define SPIN_IN_PLACE     true   // turns reverse the inner wheels instead of stopping them

/* Light */
#define PIN_LIGHT              4
#define LIGHT_CHANNEL          2
#define LIGHT_FREQ          5000
#define LIGHT_BITS             8
#define LIGHT_MAX             80   // brightness limit (0..255)
#define LIGHT_RAMP_STEP        6   // gradual change limits the current step on the battery
#define LIGHT_RAMP_INTERVAL_MS 20

/* AI Thinker ESP32-CAM camera pins */
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

static httpd_handle_t webServer    = NULL;
static httpd_handle_t streamServer = NULL;

static int8_t  dirL = 0, dirR = 0;
static uint8_t targetDutyL = 0, targetDutyR = 0;
static uint8_t dutyL = 0, dutyR = 0;
static uint8_t speed = DEFAULT_SPEED;
static uint32_t lastCommandTime = 0;
static uint32_t lastRampTime = 0;
static const char* currentCommand = "stop";

static uint8_t targetLight = 0, light = 0;
static uint32_t lastLightRampTime = 0;

static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char* STREAM_BOUNDARY     = "\r\n--frame\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/* ------------------------------ Light ------------------------------ */

static void initLight() {
  ledcAttachChannel(PIN_LIGHT, LIGHT_FREQ, LIGHT_BITS, LIGHT_CHANNEL);
  ledcWrite(PIN_LIGHT, 0);
  targetLight = light = 0;
}

static void setLight(int level) {
  targetLight = (uint8_t)constrain(level, 0, LIGHT_MAX);
}

static void updateLightRamp() {
  uint32_t now = millis();
  if (now - lastLightRampTime < LIGHT_RAMP_INTERVAL_MS) return;
  lastLightRampTime = now;
  if (light == targetLight) return;

  if (light < targetLight)
    light = min((int)targetLight, (int)light + LIGHT_RAMP_STEP);
  else
    light = max((int)targetLight, (int)light - LIGHT_RAMP_STEP);
  ledcWrite(PIN_LIGHT, light);
}

/* ------------------------------ Motors ----------------------------- */

static void initMotors() {
  pinMode(PIN_L_FORWARD,  OUTPUT);  digitalWrite(PIN_L_FORWARD,  LOW);
  pinMode(PIN_L_BACKWARD, OUTPUT);  digitalWrite(PIN_L_BACKWARD, LOW);
  pinMode(PIN_R_FORWARD,  OUTPUT);  digitalWrite(PIN_R_FORWARD,  LOW);
  pinMode(PIN_R_BACKWARD, OUTPUT);  digitalWrite(PIN_R_BACKWARD, LOW);

  ledcAttachChannel(PIN_EN_L, PWM_FREQ, PWM_BITS, PWM_CHANNEL_L);
  ledcAttachChannel(PIN_EN_R, PWM_FREQ, PWM_BITS, PWM_CHANNEL_R);
  ledcWrite(PIN_EN_L, 0);
  ledcWrite(PIN_EN_R, 0);
}

static void applyDirection(int pinForward, int pinBackward, int8_t dir) {
  if (dir > 0)      { digitalWrite(pinBackward, LOW); digitalWrite(pinForward, HIGH); }
  else if (dir < 0) { digitalWrite(pinForward, LOW);  digitalWrite(pinBackward, HIGH); }
  else              { digitalWrite(pinForward, LOW);  digitalWrite(pinBackward, LOW); }
}

static void stopNow() {
  targetDutyL = targetDutyR = 0;
  dutyL = dutyR = 0;
  ledcWrite(PIN_EN_L, 0);
  ledcWrite(PIN_EN_R, 0);
  dirL = dirR = 0;
  applyDirection(PIN_L_FORWARD, PIN_L_BACKWARD, 0);
  applyDirection(PIN_R_FORWARD, PIN_R_BACKWARD, 0);
  currentCommand = "stop";
}

static void setMotion(int8_t newL, int8_t newR, const char* name) {
  // A direction reversal cuts the PWM and waits before switching the H-bridge
  bool reversed = (newL != 0 && dirL != 0 && newL != dirL) ||
                  (newR != 0 && dirR != 0 && newR != dirR);
  if (reversed) {
    ledcWrite(PIN_EN_L, 0);
    ledcWrite(PIN_EN_R, 0);
    dutyL = dutyR = 0;
    delay(DEADTIME_MS);
  }
  dirL = newL;
  dirR = newR;
  applyDirection(PIN_L_FORWARD, PIN_L_BACKWARD, dirL);
  applyDirection(PIN_R_FORWARD, PIN_R_BACKWARD, dirR);
  targetDutyL = (dirL == 0) ? 0 : max(speed, (uint8_t)MIN_SPEED);
  targetDutyR = (dirR == 0) ? 0 : max(speed, (uint8_t)MIN_SPEED);
  currentCommand  = name;
  lastCommandTime = millis();
}

static void updateMotorRamp() {
  uint32_t now = millis();
  if (now - lastRampTime < RAMP_INTERVAL_MS) return;
  lastRampTime = now;
  bool changed = false;

  if (dutyL != targetDutyL) {
    dutyL = (dutyL < targetDutyL)
          ? min((int)targetDutyL, (int)dutyL + RAMP_STEP)
          : max((int)targetDutyL, (int)dutyL - RAMP_STEP);
    ledcWrite(PIN_EN_L, dutyL);
    changed = true;
  }
  if (dutyR != targetDutyR) {
    dutyR = (dutyR < targetDutyR)
          ? min((int)targetDutyR, (int)dutyR + RAMP_STEP)
          : max((int)targetDutyR, (int)dutyR - RAMP_STEP);
    ledcWrite(PIN_EN_R, dutyR);
    changed = true;
  }
  // Release the H-bridge once both motors have ramped down to zero
  if (changed && dutyL == 0 && dutyR == 0 && targetDutyL == 0 && targetDutyR == 0) {
    applyDirection(PIN_L_FORWARD, PIN_L_BACKWARD, 0);
    applyDirection(PIN_R_FORWARD, PIN_R_BACKWARD, 0);
    dirL = dirR = 0;
  }
}

/* ------------------------------ Camera ----------------------------- */

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;   config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;   config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;   config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;   config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAME_SIZE;
  config.jpeg_quality = JPEG_QUALITY;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count    = 2;
    config.grab_mode   = CAMERA_GRAB_LATEST;
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count    = 1;
    config.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
    Serial.println("PSRAM not found, check the board settings");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x (%s)\n", err, esp_err_to_name(err));
    return false;
  }
  // Rotate the image 180 degrees to match the camera mounting
  sensor_t* s = esp_camera_sensor_get();
  if (s) { s->set_vflip(s, 1); s->set_hmirror(s, 1); }
  return true;
}

/* -------------------------- HTTP handlers -------------------------- */

static bool readParam(httpd_req_t* req, const char* key, char* out, size_t size) {
  size_t qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen <= 1) return false;
  char* query = (char*)malloc(qlen);
  if (!query) return false;
  bool found = false;
  if (httpd_req_get_url_query_str(req, query, qlen) == ESP_OK)
    found = (httpd_query_key_value(query, key, out, size) == ESP_OK);
  free(query);
  return found;
}

static void sendStatus(httpd_req_t* req) {
  char body[200];
  snprintf(body, sizeof(body),
           "{\"cmd\":\"%s\",\"speed\":%u,\"dutyL\":%u,\"dutyR\":%u,"
           "\"light\":%u,\"heap\":%u}",
           currentCommand, speed, dutyL, dutyR,
           light, (unsigned)ESP.getFreeHeap());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cmdHandler(httpd_req_t* req) {
  char value[16];

  if (readParam(req, "speed", value, sizeof(value))) {
    speed = (uint8_t)constrain(atoi(value), 0, 255);
    if (targetDutyL) targetDutyL = max(speed, (uint8_t)MIN_SPEED);
    if (targetDutyR) targetDutyR = max(speed, (uint8_t)MIN_SPEED);
  }

  if (readParam(req, "dir", value, sizeof(value))) {
    if      (!strcmp(value, "forward"))  setMotion( 1,  1, "forward");
    else if (!strcmp(value, "backward")) setMotion(-1, -1, "backward");
    else if (!strcmp(value, "left"))     setMotion(SPIN_IN_PLACE ? -1 : 0,  1, "left");
    else if (!strcmp(value, "right"))    setMotion( 1, SPIN_IN_PLACE ? -1 : 0, "right");
    else if (!strcmp(value, "stop"))     stopNow();
    else { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid dir"); return ESP_FAIL; }
  }

  lastCommandTime = millis();
  sendStatus(req);
  return ESP_OK;
}

static esp_err_t lightHandler(httpd_req_t* req) {
  char value[16];
  if (readParam(req, "level", value, sizeof(value))) {
    setLight(atoi(value));
  } else if (readParam(req, "on", value, sizeof(value))) {
    setLight(atoi(value) ? LIGHT_MAX : 0);
  }
  sendStatus(req);
  return ESP_OK;
}

static esp_err_t photoHandler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

static esp_err_t streamHandler(httpd_req_t* req) {
  char header[64];
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    int64_t frameStart = esp_timer_get_time();
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }

    size_t len = fb->len;
    snprintf(header, sizeof(header), STREAM_PART, (unsigned)len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, header, strlen(header));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;

    // Hold a fixed frame interval to keep the Wi-Fi load predictable
    int32_t elapsed = (int32_t)((esp_timer_get_time() - frameStart) / 1000);
    int32_t remaining = STREAM_INTERVAL_MS - elapsed;
    vTaskDelay(pdMS_TO_TICKS(remaining > 0 ? remaining : 1));
  }
  return res;
}

static esp_err_t rootHandler(httpd_req_t* req) {
  static const char page[] = R"HTML(<!DOCTYPE html><html><head>
<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>ESP32-CAM Robot</title><style>
body{font-family:sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:10px}
img{max-width:100%;border:1px solid #444;min-height:100px}
button{padding:16px;font-size:15px;margin:3px;border:0;border-radius:8px;
background:#2a4d8f;color:#fff;touch-action:none;user-select:none}
button:active{background:#4a7dcf}
.grid{display:grid;grid-template-columns:repeat(3,80px);justify-content:center;gap:4px;margin-top:10px}
#st{font-family:monospace;font-size:12px;color:#8a8;word-break:break-all}
label{font-size:14px}
</style></head><body>
<h3>ESP32-CAM Robot</h3>
<img id='v'>
<p>
<button onclick="v.src='http://'+location.hostname+':81/stream'">Stream ON</button>
<button onclick="v.src=''">Stream OFF</button>
<button id='lt' style='background:#8f7a2a'>Light</button>
</p>
<div class='grid'>
<div></div><button id='f'>&#9650;</button><div></div>
<button id='l'>&#9664;</button><button id='s' style='background:#8f2a2a'>STOP</button><button id='r'>&#9654;</button>
<div></div><button id='b'>&#9660;</button><div></div>
</div>
<p><label>Speed: <span id='spv'>180</span></label><br>
<input type='range' min='80' max='255' value='180' id='sp' style='width:80%'></p>
<p><label>Light: <span id='lv'>0</span></label><br>
<input type='range' min='0' max='80' value='0' id='ls' style='width:80%'></p>
<div id='st'>--</div>
<script>
const v=document.getElementById('v');
let repeat=null, lightOn=false;
function show(j){st.textContent=JSON.stringify(j);}
function send(d){fetch('/cmd?dir='+d+'&speed='+sp.value)
  .then(r=>r.json()).then(show).catch(()=>{});}
function setLight(n){fetch('/light?level='+n)
  .then(r=>r.json()).then(show).catch(()=>{});}
function hold(id,d){const b=document.getElementById(id);
  const start=e=>{e.preventDefault();send(d);clearInterval(repeat);
                  repeat=setInterval(()=>send(d),300);};
  const end=e=>{e.preventDefault();clearInterval(repeat);send('stop');};
  b.addEventListener('pointerdown',start);
  b.addEventListener('pointerup',end);
  b.addEventListener('pointerleave',end);
  b.addEventListener('pointercancel',end);}
hold('f','forward');hold('b','backward');hold('l','left');hold('r','right');
document.getElementById('s').addEventListener('pointerdown',e=>{
  e.preventDefault();clearInterval(repeat);send('stop');});
document.getElementById('lt').addEventListener('click',()=>{
  lightOn=!lightOn;
  const n=lightOn?ls.max:0;
  ls.value=n;lv.textContent=n;setLight(n);});
sp.addEventListener('input',()=>{spv.textContent=sp.value;});
ls.addEventListener('change',()=>{lv.textContent=ls.value;setLight(ls.value);});
ls.addEventListener('input',()=>{lv.textContent=ls.value;});
</script></body></html>)HTML";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

/* --------------------------- HTTP servers -------------------------- */

static bool startServers() {
  // Control and stream run on separate servers so that a blocking stream
  // does not delay motor commands
  httpd_config_t webCfg = HTTPD_DEFAULT_CONFIG();
  webCfg.server_port      = 80;
  webCfg.ctrl_port        = 32768;
  webCfg.max_uri_handlers = 8;
  webCfg.stack_size       = 8192;
  webCfg.lru_purge_enable = true;

  if (httpd_start(&webServer, &webCfg) != ESP_OK) {
    Serial.println("Failed to start server on port 80");
    return false;
  }
  httpd_uri_t rootUri  = { "/",      HTTP_GET, rootHandler,  NULL };
  httpd_uri_t photoUri = { "/photo", HTTP_GET, photoHandler, NULL };
  httpd_uri_t cmdUri   = { "/cmd",   HTTP_GET, cmdHandler,   NULL };
  httpd_uri_t lightUri = { "/light", HTTP_GET, lightHandler, NULL };
  httpd_register_uri_handler(webServer, &rootUri);
  httpd_register_uri_handler(webServer, &photoUri);
  httpd_register_uri_handler(webServer, &cmdUri);
  httpd_register_uri_handler(webServer, &lightUri);

  httpd_config_t streamCfg = HTTPD_DEFAULT_CONFIG();
  streamCfg.server_port      = 81;
  streamCfg.ctrl_port        = 32769;
  streamCfg.max_uri_handlers = 2;
  streamCfg.stack_size       = 8192;
  streamCfg.lru_purge_enable = true;

  if (httpd_start(&streamServer, &streamCfg) != ESP_OK) {
    Serial.println("Failed to start server on port 81");
    return false;
  }
  httpd_uri_t streamUri = { "/stream", HTTP_GET, streamHandler, NULL };
  httpd_register_uri_handler(streamServer, &streamUri);
  return true;
}

/* ------------------------------ Wi-Fi ------------------------------ */

static bool startWiFi() {
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.setTxPower(TX_POWER);
  return WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CONN);
}

/* ------------------------------ Main ------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(1500);

  initMotors();
  initLight();

  // Wi-Fi must start before the camera
  if (!startWiFi())   { Serial.println("Wi-Fi init failed"); return; }
  if (!initCamera())  return;
  if (!startServers()) return;

  lastCommandTime = millis();

  Serial.print("Control page : http://"); Serial.println(WiFi.softAPIP());
  Serial.print("Stream       : http://"); Serial.print(WiFi.softAPIP());
  Serial.println(":81/stream");
}

void loop() {
  updateMotorRamp();
  updateLightRamp();

  // Failsafe: stop the motors if the host stops sending commands
  if ((targetDutyL || targetDutyR) &&
      (millis() - lastCommandTime > COMMAND_TIMEOUT_MS)) {
    stopNow();
  }
}
