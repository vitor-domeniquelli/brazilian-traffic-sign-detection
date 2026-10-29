/* =====================================================================
 *  ROBO ESP32-CAM
 *  ---------------------------------------------------------------
 *  Alvo   : ESP32-CAM AI Thinker + L298N + 2x 18650 (2S)
 *  Board  : "ESP32 Dev Module"
 *  Core   : arduino-esp32 3.3.10
 *
 *  MAPA DE CANAIS LEDC - NAO DEIXE NENHUM AUTOMATICO
 *    canal 0 / timer 0 .... XCLK da camera   (reservado pelo driver)
 *    canal 2 / timer 1 .... farol GPIO4
 *    canal 4 / timer 2 .... PWM motor esquerdo
 *    canal 6 / timer 3 .... PWM motor direito
 *
 * ===================================================================== */

#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_system.h"

/* ----------------------- Configuracao ----------------------------- */
#define AP_SSID        "ROBO-TESTE"
#define AP_PASS        "12345678"
#define AP_CHANNEL     1          // teste 1, 6 ou 11 se houver perda
#define AP_MAX_CONN    1

#define TX_POWER       WIFI_POWER_11dBm
#define FRAME_SIZE     FRAMESIZE_VGA
#define JPEG_QUALITY   12
#define STREAM_INTERVALO_MS   150
#define STATS_A_CADA_N_FRAMES 30
#define HEARTBEAT_MS          5000

/* --- Motores --- */
#define PIN_L_FRENTE   15
#define PIN_L_TRAS     33
#define PIN_R_FRENTE   13
#define PIN_R_TRAS     14
#define PIN_EN_L        2
#define PIN_EN_R       12

#define CANAL_PWM_L     4
#define CANAL_PWM_R     6
#define PWM_FREQ     5000
#define PWM_BITS        8

#define VEL_PADRAO    180
#define VEL_MINIMA     80
#define RAMPA_PASSO    12
#define RAMPA_MS       15
#define DEADTIME_MS    40
#define TIMEOUT_COMANDO_MS 900
#define GIRO_NO_EIXO   true

/* --- Farol --- */
#define PIN_FAROL       4
#define CANAL_FAROL     2         // timer 1: livre
#define FAROL_FREQ   5000
#define FAROL_BITS      8
#define FAROL_MAX      80         // teto de seguranca (0..255)
#define FAROL_RAMPA_PASSO  6      // suaviza o degrau de corrente
#define FAROL_RAMPA_MS     20
/* ------------------------------------------------------------------ */

/* Pinos AI Thinker ESP32-CAM */
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

static httpd_handle_t servidorWeb    = NULL;
static httpd_handle_t servidorStream = NULL;

static uint32_t heartbeatAnterior = 0;
static uint32_t contadorHeartbeat = 0;
static volatile bool     streamAtivo = false;
static volatile uint32_t framesNoStream = 0;

static int8_t  sentidoL = 0, sentidoR = 0;
static uint8_t dutyAlvoL = 0, dutyAlvoR = 0;
static uint8_t dutyAtualL = 0, dutyAtualR = 0;
static uint8_t velocidade = VEL_PADRAO;
static uint32_t ultimoComando = 0;
static uint32_t ultimaRampa = 0;
static const char* comandoAtual = "parar";

static uint8_t farolAlvo = 0, farolAtual = 0;
static uint32_t ultimaRampaFarol = 0;

static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=quadro";
static const char* STREAM_BOUNDARY     = "\r\n--quadro\r\n";
static const char* STREAM_PARTE        = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/* ------------------------- Diagnostico ---------------------------- */

static const char* motivoReset(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON  - energizacao normal";
    case ESP_RST_EXT:       return "EXT      - pino de reset externo";
    case ESP_RST_SW:        return "SW       - esp_restart() por software";
    case ESP_RST_PANIC:     return "PANIC    - excecao / crash de codigo";
    case ESP_RST_INT_WDT:   return "INT_WDT  - interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "TASK_WDT - task watchdog";
    case ESP_RST_WDT:       return "WDT      - outro watchdog";
    case ESP_RST_BROWNOUT:  return "BROWNOUT - QUEDA DE TENSAO  <<<<<<";
    default:                return "UNKNOWN  - motivo nao identificado";
  }
}

static void marco(const char* etapa) {
  Serial.printf("[%6lu ms] %-28s | heap %6u B | menor %6u B\n",
                (unsigned long)millis(), etapa,
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
  Serial.flush();
}

/* ---------------------------- Farol ------------------------------- */

static void iniciaFarol() {
  bool ok = ledcAttachChannel(PIN_FAROL, FAROL_FREQ, FAROL_BITS, CANAL_FAROL);
  Serial.printf("       farol : pino %d -> canal %d [%s], teto %d/255\n",
                PIN_FAROL, CANAL_FAROL, ok ? "OK" : "FALHOU", FAROL_MAX);
  ledcWrite(PIN_FAROL, 0);
  farolAlvo = farolAtual = 0;
}

static void defineFarol(int nivel) {
  farolAlvo = (uint8_t)constrain(nivel, 0, FAROL_MAX);
}

static void atualizaRampaFarol() {
  uint32_t agora = millis();
  if (agora - ultimaRampaFarol < FAROL_RAMPA_MS) return;
  ultimaRampaFarol = agora;
  if (farolAtual == farolAlvo) return;

  if (farolAtual < farolAlvo)
    farolAtual = min((int)farolAlvo, (int)farolAtual + FAROL_RAMPA_PASSO);
  else
    farolAtual = max((int)farolAlvo, (int)farolAtual - FAROL_RAMPA_PASSO);
  ledcWrite(PIN_FAROL, farolAtual);
}

/* --------------------------- Motores ------------------------------ */

static void iniciaMotores() {
  marco(">>> iniciaMotores()");
  pinMode(PIN_L_FRENTE, OUTPUT);  digitalWrite(PIN_L_FRENTE, LOW);
  pinMode(PIN_L_TRAS,   OUTPUT);  digitalWrite(PIN_L_TRAS,   LOW);
  pinMode(PIN_R_FRENTE, OUTPUT);  digitalWrite(PIN_R_FRENTE, LOW);
  pinMode(PIN_R_TRAS,   OUTPUT);  digitalWrite(PIN_R_TRAS,   LOW);

  bool okL = ledcAttachChannel(PIN_EN_L, PWM_FREQ, PWM_BITS, CANAL_PWM_L);
  bool okR = ledcAttachChannel(PIN_EN_R, PWM_FREQ, PWM_BITS, CANAL_PWM_R);
  Serial.printf("       PWM esq: pino %d -> canal %d [%s]\n",
                PIN_EN_L, CANAL_PWM_L, okL ? "OK" : "FALHOU");
  Serial.printf("       PWM dir: pino %d -> canal %d [%s]\n",
                PIN_EN_R, CANAL_PWM_R, okR ? "OK" : "FALHOU");
  ledcWrite(PIN_EN_L, 0);
  ledcWrite(PIN_EN_R, 0);
  marco("<<< motores prontos (parados)");
}

static void aplicaSentido(int pinFrente, int pinTras, int8_t sentido) {
  if (sentido > 0)      { digitalWrite(pinTras, LOW);   digitalWrite(pinFrente, HIGH); }
  else if (sentido < 0) { digitalWrite(pinFrente, LOW); digitalWrite(pinTras, HIGH);  }
  else                  { digitalWrite(pinFrente, LOW); digitalWrite(pinTras, LOW);   }
}

static void pararImediato() {
  dutyAlvoL = dutyAlvoR = 0;
  dutyAtualL = dutyAtualR = 0;
  ledcWrite(PIN_EN_L, 0);
  ledcWrite(PIN_EN_R, 0);
  sentidoL = sentidoR = 0;
  aplicaSentido(PIN_L_FRENTE, PIN_L_TRAS, 0);
  aplicaSentido(PIN_R_FRENTE, PIN_R_TRAS, 0);
  comandoAtual = "parar";
}

static void defineMovimento(int8_t novoL, int8_t novoR, const char* nome) {
  bool inverteu = (novoL != 0 && sentidoL != 0 && novoL != sentidoL) ||
                  (novoR != 0 && sentidoR != 0 && novoR != sentidoR);
  if (inverteu) {
    ledcWrite(PIN_EN_L, 0);
    ledcWrite(PIN_EN_R, 0);
    dutyAtualL = dutyAtualR = 0;
    delay(DEADTIME_MS);
  }
  sentidoL = novoL;
  sentidoR = novoR;
  aplicaSentido(PIN_L_FRENTE, PIN_L_TRAS, sentidoL);
  aplicaSentido(PIN_R_FRENTE, PIN_R_TRAS, sentidoR);
  dutyAlvoL = (sentidoL == 0) ? 0 : max(velocidade, (uint8_t)VEL_MINIMA);
  dutyAlvoR = (sentidoR == 0) ? 0 : max(velocidade, (uint8_t)VEL_MINIMA);
  comandoAtual  = nome;
  ultimoComando = millis();
}

static void atualizaRampa() {
  uint32_t agora = millis();
  if (agora - ultimaRampa < RAMPA_MS) return;
  ultimaRampa = agora;
  bool mudou = false;

  if (dutyAtualL != dutyAlvoL) {
    dutyAtualL = (dutyAtualL < dutyAlvoL)
               ? min((int)dutyAlvoL, (int)dutyAtualL + RAMPA_PASSO)
               : max((int)dutyAlvoL, (int)dutyAtualL - RAMPA_PASSO);
    ledcWrite(PIN_EN_L, dutyAtualL);
    mudou = true;
  }
  if (dutyAtualR != dutyAlvoR) {
    dutyAtualR = (dutyAtualR < dutyAlvoR)
               ? min((int)dutyAlvoR, (int)dutyAtualR + RAMPA_PASSO)
               : max((int)dutyAlvoR, (int)dutyAtualR - RAMPA_PASSO);
    ledcWrite(PIN_EN_R, dutyAtualR);
    mudou = true;
  }
  if (mudou && dutyAtualL == 0 && dutyAtualR == 0 && dutyAlvoL == 0 && dutyAlvoR == 0) {
    aplicaSentido(PIN_L_FRENTE, PIN_L_TRAS, 0);
    aplicaSentido(PIN_R_FRENTE, PIN_R_TRAS, 0);
    sentidoL = sentidoR = 0;
  }
}

/* --------------------------- Camera ------------------------------- */

static bool iniciaCamera() {
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
    Serial.println("       !! PSRAM Disabled no Tools. Habilite.");
  }

  marco(">>> esp_camera_init()");
  esp_err_t err = esp_camera_init(&config);
  marco("<<< esp_camera_init()");
  if (err != ESP_OK) {
    Serial.printf("!! esp_camera_init FALHOU: 0x%x (%s)\n", err, esp_err_to_name(err));
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) { s->set_vflip(s, 1); s->set_hmirror(s, 1); }
  return true;
}

/* ------------------------ Handlers HTTP --------------------------- */

static bool leParam(httpd_req_t* req, const char* chave, char* saida, size_t tam) {
  size_t qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen <= 1) return false;
  char* query = (char*)malloc(qlen);
  if (!query) return false;
  bool achou = false;
  if (httpd_req_get_url_query_str(req, query, qlen) == ESP_OK)
    achou = (httpd_query_key_value(query, chave, saida, tam) == ESP_OK);
  free(query);
  return achou;
}

static void respondeEstado(httpd_req_t* req) {
  char resposta[200];
  snprintf(resposta, sizeof(resposta),
           "{\"cmd\":\"%s\",\"vel\":%u,\"dutyL\":%u,\"dutyR\":%u,"
           "\"farol\":%u,\"heap\":%u}",
           comandoAtual, velocidade, dutyAtualL, dutyAtualR,
           farolAtual, (unsigned)ESP.getFreeHeap());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, resposta, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handlerCmd(httpd_req_t* req) {
  char valor[16];

  if (leParam(req, "vel", valor, sizeof(valor))) {
    velocidade = (uint8_t)constrain(atoi(valor), 0, 255);
    if (dutyAlvoL) dutyAlvoL = max(velocidade, (uint8_t)VEL_MINIMA);
    if (dutyAlvoR) dutyAlvoR = max(velocidade, (uint8_t)VEL_MINIMA);
  }

  if (leParam(req, "dir", valor, sizeof(valor))) {
    if      (!strcmp(valor, "frente")) defineMovimento( 1,  1, "frente");
    else if (!strcmp(valor, "tras"))   defineMovimento(-1, -1, "tras");
    else if (!strcmp(valor, "esq"))    defineMovimento(GIRO_NO_EIXO ? -1 : 0,  1, "esq");
    else if (!strcmp(valor, "dir"))    defineMovimento( 1, GIRO_NO_EIXO ? -1 : 0, "dir");
    else if (!strcmp(valor, "parar"))  pararImediato();
    else { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "dir invalida"); return ESP_FAIL; }
  }

  ultimoComando = millis();
  respondeEstado(req);
  return ESP_OK;
}

static esp_err_t handlerFarol(httpd_req_t* req) {
  char valor[16];
  if (leParam(req, "nivel", valor, sizeof(valor))) {
    defineFarol(atoi(valor));
    Serial.printf("       farol -> %u (teto %d)\n", farolAlvo, FAROL_MAX);
  } else if (leParam(req, "on", valor, sizeof(valor))) {
    defineFarol(atoi(valor) ? FAROL_MAX : 0);
  }
  respondeEstado(req);
  return ESP_OK;
}

static esp_err_t handlerFoto(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t r = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return r;
}

static esp_err_t handlerStream(httpd_req_t* req) {
  char cabecalho[64];
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  streamAtivo = true;
  framesNoStream = 0;
  uint32_t bytesAcumulados = 0;
  int64_t tJanela = esp_timer_get_time();
  Serial.println(">>> stream INICIADO");

  while (true) {
    int64_t tFrame = esp_timer_get_time();
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { Serial.println("!! stream: fb_get NULL"); res = ESP_FAIL; break; }

    size_t len = fb->len;
    snprintf(cabecalho, sizeof(cabecalho), STREAM_PARTE, (unsigned)len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, cabecalho, strlen(cabecalho));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;

    framesNoStream++;
    bytesAcumulados += len;

    if (framesNoStream % STATS_A_CADA_N_FRAMES == 0) {
      int64_t agora = esp_timer_get_time();
      float seg = (agora - tJanela) / 1000000.0f;
      Serial.printf("    stream: %lu fr | %.1f fps | %.1f KB/s | cmd %-6s | duty %3u/%3u | farol %3u | heap %u B\n",
                    (unsigned long)framesNoStream, STATS_A_CADA_N_FRAMES / seg,
                    (bytesAcumulados / 1024.0f) / seg, comandoAtual,
                    dutyAtualL, dutyAtualR, farolAtual, (unsigned)ESP.getFreeHeap());
      bytesAcumulados = 0;
      tJanela = agora;
    }

#if STREAM_INTERVALO_MS > 0
    int32_t gasto = (int32_t)((esp_timer_get_time() - tFrame) / 1000);
    int32_t sobra = STREAM_INTERVALO_MS - gasto;
    vTaskDelay(pdMS_TO_TICKS(sobra > 0 ? sobra : 1));
#else
    vTaskDelay(pdMS_TO_TICKS(1));
#endif
  }

  streamAtivo = false;
  Serial.printf("<<< stream ENCERRADO apos %lu frames | heap %u B\n",
                (unsigned long)framesNoStream, (unsigned)ESP.getFreeHeap());
  return res;
}

static esp_err_t handlerRaiz(httpd_req_t* req) {
  static const char pagina[] = R"HTML(<!DOCTYPE html><html><head>
<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Robo - Etapa 5</title><style>
body{font-family:sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:10px}
img{max-width:100%;border:1px solid #444;min-height:100px}
button{padding:16px;font-size:15px;margin:3px;border:0;border-radius:8px;
background:#2a4d8f;color:#fff;touch-action:none;user-select:none}
button:active{background:#4a7dcf}
.grid{display:grid;grid-template-columns:repeat(3,80px);justify-content:center;gap:4px;margin-top:10px}
#st{font-family:monospace;font-size:12px;color:#8a8;word-break:break-all}
label{font-size:14px}
</style></head><body>
<h3>Etapa 5 - camera + motores + farol</h3>
<img id='v'>
<p>
<button onclick="v.src='http://'+location.hostname+':81/stream'">Stream ON</button>
<button onclick="v.src=''">Stream OFF</button>
<button id='fa' style='background:#8f7a2a'>Farol</button>
</p>
<div class='grid'>
<div></div><button id='f'>&#9650;</button><div></div>
<button id='e'>&#9664;</button><button id='p' style='background:#8f2a2a'>STOP</button><button id='d'>&#9654;</button>
<div></div><button id='t'>&#9660;</button><div></div>
</div>
<p><label>Velocidade: <span id='vv'>180</span></label><br>
<input type='range' min='80' max='255' value='180' id='sl' style='width:80%'></p>
<p><label>Farol: <span id='fv'>0</span></label><br>
<input type='range' min='0' max='80' value='0' id='fs' style='width:80%'></p>
<div id='st'>--</div>
<script>
const v=document.getElementById('v');
let rep=null, farolLigado=false;
function mostra(j){st.textContent=JSON.stringify(j);}
function envia(d){fetch('/cmd?dir='+d+'&vel='+sl.value)
  .then(r=>r.json()).then(mostra).catch(()=>{});}
function farol(n){fetch('/farol?nivel='+n)
  .then(r=>r.json()).then(mostra).catch(()=>{});}
function segura(id,d){const b=document.getElementById(id);
  const ini=e=>{e.preventDefault();envia(d);clearInterval(rep);
                rep=setInterval(()=>envia(d),300);};
  const fim=e=>{e.preventDefault();clearInterval(rep);envia('parar');};
  b.addEventListener('pointerdown',ini);
  b.addEventListener('pointerup',fim);
  b.addEventListener('pointerleave',fim);
  b.addEventListener('pointercancel',fim);}
segura('f','frente');segura('t','tras');segura('e','esq');segura('d','dir');
document.getElementById('p').addEventListener('pointerdown',e=>{
  e.preventDefault();clearInterval(rep);envia('parar');});
document.getElementById('fa').addEventListener('click',()=>{
  farolLigado=!farolLigado;
  const n=farolLigado?fs.max:0;
  fs.value=n;fv.textContent=n;farol(n);});
sl.addEventListener('input',()=>{vv.textContent=sl.value;});
fs.addEventListener('change',()=>{fv.textContent=fs.value;farol(fs.value);});
fs.addEventListener('input',()=>{fv.textContent=fs.value;});
</script></body></html>)HTML";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, pagina, HTTPD_RESP_USE_STRLEN);
}

/* ------------------------ Servidores HTTP ------------------------- */

static bool iniciaServidores() {
  httpd_config_t cfgWeb = HTTPD_DEFAULT_CONFIG();
  cfgWeb.server_port      = 80;
  cfgWeb.ctrl_port        = 32768;
  cfgWeb.max_uri_handlers = 8;
  cfgWeb.stack_size       = 8192;
  cfgWeb.lru_purge_enable = true;

  marco(">>> httpd porta 80");
  if (httpd_start(&servidorWeb, &cfgWeb) != ESP_OK) {
    Serial.println("!! httpd_start(80) FALHOU");
    return false;
  }
  httpd_uri_t uriRaiz  = { "/",      HTTP_GET, handlerRaiz,  NULL };
  httpd_uri_t uriFoto  = { "/foto",  HTTP_GET, handlerFoto,  NULL };
  httpd_uri_t uriCmd   = { "/cmd",   HTTP_GET, handlerCmd,   NULL };
  httpd_uri_t uriFarol = { "/farol", HTTP_GET, handlerFarol, NULL };
  httpd_register_uri_handler(servidorWeb, &uriRaiz);
  httpd_register_uri_handler(servidorWeb, &uriFoto);
  httpd_register_uri_handler(servidorWeb, &uriCmd);
  httpd_register_uri_handler(servidorWeb, &uriFarol);

  httpd_config_t cfgStream = HTTPD_DEFAULT_CONFIG();
  cfgStream.server_port      = 81;
  cfgStream.ctrl_port        = 32769;
  cfgStream.max_uri_handlers = 2;
  cfgStream.stack_size       = 8192;
  cfgStream.lru_purge_enable = true;

  marco(">>> httpd porta 81");
  if (httpd_start(&servidorStream, &cfgStream) != ESP_OK) {
    Serial.println("!! httpd_start(81) FALHOU");
    return false;
  }
  httpd_uri_t uriStream = { "/stream", HTTP_GET, handlerStream, NULL };
  httpd_register_uri_handler(servidorStream, &uriStream);

  marco("<<< servidores no ar");
  return true;
}

/* ---------------------------- WiFi -------------------------------- */

static bool iniciaWiFi() {
  marco("chamando WiFi.mode()");
  if (!WiFi.mode(WIFI_AP)) Serial.println("!! FALHA em WiFi.mode()");
  delay(100);
  WiFi.setTxPower(TX_POWER);
  Serial.printf("       TX power lida de volta: %d (0,25 dBm)\n", (int)WiFi.getTxPower());
  marco(">>> softAP()");
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CONN);
  marco("<<< softAP()");
  return ok;
}

/* ---------------------------- Setup ------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.setDebugOutput(true);

  Serial.println();
  Serial.println("========================================================");
  Serial.println("  ETAPA 5 - CAMERA + MOTORES + FAROL");
  Serial.println("========================================================");
  Serial.printf("Ultimo reset : %s\n", motivoReset(esp_reset_reason()));
  Serial.printf("PSRAM        : %s\n", psramFound() ? "DETECTADA" : "NAO detectada");
  Serial.println("--------------------------------------------------------");

  iniciaMotores();
  iniciaFarol();

  /* ORDEM VALIDADA: WiFi ANTES da camera. Nao inverta. */
  if (!iniciaWiFi())   { Serial.println("!! WiFi falhou.");   return; }
  if (!iniciaCamera()) { Serial.println("!! Camera falhou."); return; }
  if (!iniciaServidores()) return;

  ultimoComando = millis();

  Serial.println("--------------------------------------------------------");
  Serial.print ("Pagina : http://"); Serial.println(WiFi.softAPIP());
  Serial.print ("Stream : http://"); Serial.print(WiFi.softAPIP());
  Serial.println(":81/stream");
  Serial.println("SETUP CONCLUIDO.");
  Serial.println();
}

void loop() {
  atualizaRampa();
  atualizaRampaFarol();

  if ((dutyAlvoL || dutyAlvoR) &&
      (millis() - ultimoComando > TIMEOUT_COMANDO_MS)) {
    Serial.println("!! FAILSAFE: sem comando, parando motores");
    pararImediato();
  }

  uint32_t agora = millis();
  if (agora - heartbeatAnterior < HEARTBEAT_MS) return;
  heartbeatAnterior = agora;
  contadorHeartbeat++;

  Serial.printf("vivo #%-4lu | %4lu s | clientes %d | stream %s | cmd %-6s | duty %3u/%3u | farol %3u | heap %6u B | menor %6u B\n",
                (unsigned long)contadorHeartbeat, (unsigned long)(agora / 1000),
                WiFi.softAPgetStationNum(), streamAtivo ? "ON " : "OFF",
                comandoAtual, dutyAtualL, dutyAtualR, farolAtual,
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
}
