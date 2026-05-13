// Smart IV Pole — real load cell implementation
// Branch: feature/loadcell
//
// Required libraries (Arduino Library Manager):
//   PubSubClient  (Nick O'Leary)
//   ArduinoJson   (Benoit Blanchon)
//
// WiFi provisioning: use "ESP BLE Prov" app (Espressif)
//   Device name : IV_POLE_BLE
//   Proof-of-Pos: ivpole01
//
// MQTT topics (client-id = iv_pole_01):
//   Subscribe  iv_pole/iv_pole_01/config   ← app sends target flow rate
//   Subscribe  iv_pole/iv_pole_01/cmd      ← tare / calibrate / reset
//   Publish    iv_pole/iv_pole_01/status   → weight, flow rate, ETA
//   Publish    iv_pole/iv_pole_01/alert    → anomaly / IV_COMPLETE

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiProv.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "ads1232.h"
#include "cnn_detector.h"

// ── Image log ─────────────────────────────────────────────────────
#define IMAGE_LOG_PATH  "/imglog.csv"
#define IMAGE_LOG_MAX   200   // 최대 저장 줄 수 (초과 시 덮어씀)

// ── Pin definitions ────────────────────────────────────────────────
#define ADS_DOUT  19   // DRDY/DOUT → IO19
#define ADS_SCLK  18   // SCLK      → IO18
#define ADS_PDWN  23   // PDWN      → IO23
#define ADS_GAIN0 33   // GAIN0     → IO33
#define ADS_GAIN1 32   // GAIN1     → IO32

// ── MQTT ──────────────────────────────────────────────────────────
#define MQTT_BROKER    "192.168.1.100"   // TODO: set your broker IP
#define MQTT_PORT      1883
#define MQTT_CLIENT    "iv_pole_01"

#define T_STATUS  "iv_pole/" MQTT_CLIENT "/status"
#define T_ALERT   "iv_pole/" MQTT_CLIENT "/alert"
#define T_CONFIG  "iv_pole/" MQTT_CLIENT "/config"
#define T_CMD     "iv_pole/" MQTT_CLIENT "/cmd"

// ── Timing ────────────────────────────────────────────────────────
#define WEIGHT_MS       3000   // weight read period (3s = ~3min per CNN window)
#define STATUS_MS       5000   // status publish period
#define MQTT_RETRY_MS   5000

// ── Calibration ───────────────────────────────────────────────────
// Run once: place known weight → read serial output → set this value.
// calib_factor = (raw - tare) / known_grams
#define CALIB_FACTOR_DEFAULT  1642.8623  // TODO: replace after calibration

// ── Drip factor ───────────────────────────────────────────────────
// 1 gtt 당 무게 (g). IV 세트 종류에 따라 다름:
//   20 gtt/mL  →  0.0500 g/gtt  (기본, 일반 성인용)
//   15 gtt/mL  →  0.0667 g/gtt
//   10 gtt/mL  →  0.1000 g/gtt
//   60 gtt/mL  →  0.0167 g/gtt  (소아용 마이크로드립)
// serial 커맨드 "dropfactor <g>" 로 런타임에 교정 가능
#define DRIP_FACTOR_DEFAULT  0.0500f

// ── BLE provisioning ──────────────────────────────────────────────
#define BLE_NAME  "IV_POLE_BLE"
#define PROV_POP  "ivpole01"

// ── Objects ───────────────────────────────────────────────────────
ADS1232      loadCell(ADS_DOUT, ADS_SCLK, ADS_PDWN, ADS_GAIN0, ADS_GAIN1);
WiFiClient   espClient;
PubSubClient mqtt(espClient);
CNNDetector  detector;

// ── IV state ──────────────────────────────────────────────────────
// EMA가 수렴할 때까지 기다리는 워밍업 샘플 수
#define WARMUP_SAMPLES  8

struct IVState {
  float targetFlowRate = 0;                   // g/s
  float finishWeight   = 0;                   // g
  float currentWeight  = 0;                   // g
  float prevWeight     = 0;                   // g
  float currentFlowRate= 0;                   // g/s
  float dripFactor     = DRIP_FACTOR_DEFAULT; // g/gtt — IV 세트에 맞게 교정
  bool  started        = false;
  bool  complete       = false;
  int   warmup         = 0;   // 남은 워밍업 횟수 (>0이면 flow 계산 안 함)
} iv;

unsigned long lastWeightMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastMqttMs   = 0;

// ── BLE provisioning callback ─────────────────────────────────────
void onProvEvent(arduino_event_t *ev) {
  switch (ev->event_id) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WiFi] Disconnected.");
      break;
    case ARDUINO_EVENT_PROV_START:
      Serial.printf("[Prov] BLE ready. Device=%s  PoP=%s\n", BLE_NAME, PROV_POP);
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
      Serial.println("[Prov] Credentials received.");
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("[Prov] WiFi verified.");
      break;
    case ARDUINO_EVENT_PROV_END:
      Serial.println("[Prov] Provisioning done.");
      break;
    default: break;
  }
}

// ── MQTT helpers ──────────────────────────────────────────────────
void publishAlert(const char *type, float measuredFlowRate) {
  StaticJsonDocument<128> doc;
  doc["type"]        = type;
  doc["flowMeasured"]= measuredFlowRate;
  doc["flowTarget"]  = iv.targetFlowRate;
  doc["confidence"]  = detector.getWindowConfidence();
  doc["ts"]          = millis();
  char buf[128];
  serializeJson(doc, buf);
  mqtt.publish(T_ALERT, buf, true);
  Serial.printf("[ALERT] %s  신뢰도:%.0f%%\n", type, detector.getWindowConfidence() * 100.0f);
}

void publishStatus() {
  StaticJsonDocument<256> doc;
  doc["weight"]    = round(iv.currentWeight   * 100) / 100.0;
  doc["flowRate"]  = round(iv.currentFlowRate * 10000) / 10000.0;
  doc["state"]     = detector.getLastState();
  doc["result"]    = detector.getResultLabel();
  doc["confidence"]= round(detector.getWindowConfidence() * 1000) / 1000.0;
  if (iv.targetFlowRate > 0 && iv.currentWeight > iv.finishWeight)
    doc["etaSec"] = (int)((iv.currentWeight - iv.finishWeight) / iv.targetFlowRate);
  char buf[256];
  serializeJson(doc, buf);
  mqtt.publish(T_STATUS, buf);
}

// ── MQTT incoming message ─────────────────────────────────────────
void onMqttMsg(char *topic, byte *payload, unsigned int len) {
  payload[len] = '\0';
  Serial.printf("[MQTT←] %s : %s\n", topic, (char *)payload);

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, len)) return;

  String t = topic;

  if (t == T_CONFIG) {
    // App sends: { "targetFlowRate": 20, "finishWeight": 50 }  (gtt/min)
    if (doc.containsKey("targetFlowRate")) {
      float gttMin = doc["targetFlowRate"].as<float>();
      iv.targetFlowRate = (gttMin / 60.0f) * iv.dripFactor;  // gtt/min → g/s
      detector.reset();
      Serial.printf("[IV] target=%.4f g/s (%.1f gtt/min)\n",
                    iv.targetFlowRate, gttMin);
    }
    if (doc.containsKey("finishWeight"))
      iv.finishWeight = doc["finishWeight"].as<float>();

    if (!iv.started && iv.targetFlowRate > 0) {
      loadCell.resetEma();
      iv.prevWeight = iv.currentWeight;
      iv.started    = true;
      iv.complete   = false;
      iv.warmup     = WARMUP_SAMPLES;
      Serial.println("[IV] Monitoring started (8샘플 안정화 후 시작).");
    }
  }

  if (t == T_CMD) {
    String cmd = doc["cmd"].as<String>();
    if (cmd == "tare") {
      Serial.println("[CMD] Taring...");
      loadCell.tare(20);
      Serial.println("[CMD] Done.");
    } else if (cmd == "calibrate" && doc.containsKey("knownWeight")) {
      loadCell.calibrate(doc["knownWeight"].as<float>());
      Serial.printf("[CMD] Calibrated. factor=%.2f\n", loadCell.getCalibFactor());
    } else if (cmd == "reset") {
      iv = IVState{};
      detector.reset();
      Serial.println("[CMD] Reset.");
    }
  }
}

bool connectMQTT() {
  if (mqtt.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  Serial.printf("[MQTT] Connecting %s:%d ...\n", MQTT_BROKER, MQTT_PORT);
  if (mqtt.connect(MQTT_CLIENT)) {
    mqtt.subscribe(T_CONFIG);
    mqtt.subscribe(T_CMD);
    Serial.println("[MQTT] Connected.");
    return true;
  }
  Serial.printf("[MQTT] Failed, state=%d\n", mqtt.state());
  return false;
}

// ── setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Smart IV Pole (loadcell) ===");

  // LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS 마운트 실패 — 로그 저장 비활성화");
  } else {
    Serial.println("[FS] LittleFS OK");
  }

  // Load cell
  loadCell.begin(128);
  Serial.println("[ADS] Init done. Taring in 2 s...");
  delay(2000);
  loadCell.tare(20);
  loadCell.setCalibFactor(CALIB_FACTOR_DEFAULT);
  Serial.printf("[ADS] Tare=%ld  CalibFactor=%.2f\n",
                loadCell.getTareOffset(), loadCell.getCalibFactor());

  // BLE WiFi provisioning
  WiFi.onEvent(onProvEvent);
  WiFiProv.beginProvision(
    NETWORK_PROV_SCHEME_BLE,
    NETWORK_PROV_SCHEME_HANDLER_FREE_BLE,
    NETWORK_PROV_SECURITY_1,
    PROV_POP,
    BLE_NAME
  );
  Serial.println("[WiFi] Waiting for provisioning (or auto-connect from NVS)...");
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED)
    Serial.println("[WiFi] Not connected — running offline (MQTT disabled).");

  // CNN detector
  if (!detector.begin()) {
    Serial.println("[CNN] Fallback mode active until cnn_model.h is generated.");
  }

  // MQTT
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMqttMsg);
  connectMQTT();

  lastWeightMs = lastStatusMs = millis();
}

// ── CNN 이미지 시각화 & 저장 ──────────────────────────────────────

void printImage() {
  int img[CNN_DIM][CNN_DIM];
  detector.getImage(img);
  float score = detector.getWindowConfidence();

  Serial.println("[CNN] ┌────────┐");
  for (int r = 0; r < CNN_DIM; r++) {
    Serial.print("      │");
    for (int c = 0; c < CNN_DIM; c++) {
      if      (img[r][c] == FLOW_FAST)   Serial.print('+');
      else if (img[r][c] == FLOW_SLOW)   Serial.print('-');
      else                               Serial.print('.');
    }
    Serial.println("│");
  }
  Serial.println("      └────────┘");
  Serial.printf("  + 빠름  · 정상  - 느림   score:%.3f\n", score);
}

// LittleFS에 이미지를 텍스트(ASCII art)로 저장
void saveImageToLog() {
  int img[CNN_DIM][CNN_DIM];
  detector.getImage(img);

  File f = LittleFS.open(IMAGE_LOG_PATH, "a");
  if (!f) { Serial.println("[LOG] 파일 열기 실패"); return; }

  f.printf("=== %lu ms | %s | 신뢰도:%.0f%% ===\n",
           millis(), detector.getResultLabel(),
           detector.getWindowConfidence() * 100.0f);
  for (int r = 0; r < CNN_DIM; r++) {
    f.print("|");
    for (int c = 0; c < CNN_DIM; c++) {
      if      (img[r][c] == FLOW_FAST) f.print('+');
      else if (img[r][c] == FLOW_SLOW) f.print('-');
      else                             f.print('.');
    }
    f.println("|");
  }
  f.println();   // 빈 줄로 구분
  f.close();

  Serial.println("[LOG] 이미지 저장됨");
}

// 저장된 로그 전체 시리얼 출력
void dumpImageLog() {
  File f = LittleFS.open(IMAGE_LOG_PATH, "r");
  if (!f) { Serial.println("[LOG] 저장된 로그 없음"); return; }
  Serial.println("[LOG] ── imglog.csv 시작 ──────────────────");
  Serial.println("ts_ms,p00,p01,...,p77,score");
  while (f.available()) Serial.write(f.read());
  f.close();
  Serial.println("[LOG] ── 끝 ─────────────────────────────");
}

void clearImageLog() {
  LittleFS.remove(IMAGE_LOG_PATH);
  Serial.println("[LOG] 로그 삭제됨");
}

// ── Serial command handler ─────────────────────────────────────────
// Commands (send with newline):
//   target <gtt/min>   set target flow rate and start monitoring
//   finish <g>         set finish weight (stop threshold)
//   tare               tare the load cell
//   reset              reset IV state
//   status             print current state
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("target ")) {
    float gttMin = line.substring(7).toFloat();
    iv.targetFlowRate = (gttMin / 60.0f) * iv.dripFactor;  // gtt/min → g/s
    detector.reset();
    loadCell.resetEma();
    iv.prevWeight = iv.currentWeight;
    iv.started    = true;
    iv.complete   = false;
    iv.warmup     = WARMUP_SAMPLES;
    Serial.printf("[CMD] target=%.4f g/s (%.1f gtt/min)\n", iv.targetFlowRate, gttMin);
    Serial.printf("[CMD] EMA 안정화 중... (%d샘플 후 flow 계산 시작)\n", WARMUP_SAMPLES);

  } else if (line.startsWith("finish ")) {
    iv.finishWeight = line.substring(7).toFloat();
    Serial.printf("[CMD] finishWeight=%.2f g\n", iv.finishWeight);

  } else if (line == "tare") {
    Serial.println("[CMD] Taring...");
    loadCell.tare(20);
    Serial.println("[CMD] Done.");

  } else if (line == "reset") {
    iv = IVState{};
    detector.reset();
    Serial.println("[CMD] Reset.");

  } else if (line == "status") {
    float targetGtt = (iv.dripFactor > 0)
                      ? (iv.targetFlowRate / iv.dripFactor) * 60.0f : 0;
    Serial.printf("[STATUS] W:%.2fg  flow:%.4fg/s  target:%.4fg/s (%.1fgtt/min)\n"
                  "         dripFactor:%.5fg/gtt  결과:%s  신뢰도:%.0f%%  cnn:%s  샘플:%d/64\n",
                  iv.currentWeight, iv.currentFlowRate, iv.targetFlowRate, targetGtt,
                  iv.dripFactor,
                  detector.getResultLabel(),
                  detector.getWindowConfidence() * 100.0f,
                  detector.isTFLiteActive() ? "ON" : "fallback",
                  detector.getSampleCount());
  } else if (line.startsWith("tolerance ")) {
    // 유속 허용 오차 비율 설정 (0.0~1.0)
    // 예) tolerance 0.17  →  목표 60gtt 기준 ±17% = 50~70gtt 정상
    float t = line.substring(10).toFloat();
    if (t <= 0 || t >= 1.0f) {
      Serial.println("[CMD] tolerance 범위: 0.05 ~ 0.50  예) tolerance 0.17");
    } else {
      detector.setTolerance(t);
      float targetGtt = (iv.targetFlowRate > 0 && iv.dripFactor > 0)
                        ? (iv.targetFlowRate / iv.dripFactor) * 60.0f : 0;
      Serial.printf("[CMD] tolerance=±%.0f%%\n", t * 100.0f);
      if (targetGtt > 0)
        Serial.printf("[CMD] 현재 목표 %.0fgtt 기준 정상 범위: %.0f ~ %.0f gtt/min\n",
                      targetGtt, targetGtt * (1.0f - t), targetGtt * (1.0f + t));
    }

  } else if (line.startsWith("dropfactor ")) {
    // IV 세트 드립 팩터 교정 (g/gtt)
    // 예: 20gtt/mL 세트 → dropfactor 0.05
    //     15gtt/mL 세트 → dropfactor 0.0667
    //     10gtt/mL 세트 → dropfactor 0.1
    // 또는 실측 교정: 1분간 측정된 무게감소(g) ÷ 설정 gtt/min 값
    float df = line.substring(11).toFloat();
    if (df <= 0) {
      Serial.println("[CMD] dropfactor 값이 잘못됨 (예: dropfactor 0.05)");
    } else {
      iv.dripFactor = df;
      // 이미 target이 설정돼 있으면 g/s 값 재계산 필요 — target 커맨드를 다시 입력하거나:
      Serial.printf("[CMD] dripFactor=%.5f g/gtt\n", iv.dripFactor);
      Serial.println("[CMD] ※ 변경 후 'target <gtt/min>' 다시 입력해야 g/s 값이 갱신됨");
    }

  } else if (line.startsWith("alpha ")) {
    float a = line.substring(6).toFloat();
    a = constrain(a, 0.05f, 0.5f);
    loadCell.setEmaAlpha(a);
    Serial.printf("[CMD] EMA alpha=%.2f (낮을수록 부드러움)\n", a);

  } else if (line == "image") {
    printImage();

  } else if (line == "savelog") {
    saveImageToLog();

  } else if (line == "dumplog") {
    dumpImageLog();

  } else if (line == "clearlog") {
    clearImageLog();

  } else {
    Serial.println("[CMD] Commands:");
    Serial.println("  target <gtt/min>      유속 설정 및 모니터링 시작");
    Serial.println("  tolerance <0.05~0.5>  허용 오차 비율 (기본 0.17 = ±17%)");
    Serial.println("  dropfactor <g/gtt>    드립 팩터 교정 (20gtt/mL=0.05, 15=0.0667, 10=0.1)");
    Serial.println("  finish <g>            종료 무게 설정");
    Serial.println("  tare                  영점 조정");
    Serial.println("  reset                 초기화");
    Serial.println("  status                현재 상태 출력");
    Serial.println("  alpha <0.05~0.5>      EMA 부드러움 조절");
    Serial.println("  image                 현재 CNN 이미지 출력");
    Serial.println("  savelog / dumplog / clearlog");
  }
}

// ── loop ──────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  handleSerial();

  // WiFi watchdog (non-blocking)
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
  }

  // MQTT keepalive
  if (!mqtt.connected() && now - lastMqttMs > MQTT_RETRY_MS) {
    lastMqttMs = now;
    connectMQTT();
  }
  mqtt.loop();

  // ── Weight reading (every 1 s) ──────────────────────────────────
  if (now - lastWeightMs >= WEIGHT_MS) {
    lastWeightMs = now;

    iv.prevWeight    = iv.currentWeight;
    iv.currentWeight = loadCell.readWeight();

    if (iv.started && !iv.complete) {
      // 워밍업: EMA 수렴 대기, prevWeight를 계속 최신값으로 유지
      if (iv.warmup > 0) {
        iv.prevWeight = iv.currentWeight;
        iv.warmup--;
        Serial.printf("[IV] 안정화 중... W:%.2fg  (%d샘플 남음)\n",
                      iv.currentWeight, iv.warmup);
        lastWeightMs = now;
        return;
      }

      // g/s: (무게 감소량) / (경과 시간)
      // prevWeight - currentWeight = 측정 주기 동안 빠져나간 무게(g)
      // ÷ (WEIGHT_MS/1000) → g/s 로 정규화
      iv.currentFlowRate = (iv.prevWeight - iv.currentWeight)
                           / (WEIGHT_MS / 1000.0f);

      // Feed CNN detector
      detector.addSample(iv.currentFlowRate, iv.targetFlowRate);

      // Anomaly check
      if (detector.detectAnomaly()) {
        const char *alert = (detector.getWindowResult() == FLOW_FAST)
                            ? "FLOW_FAST" : "FLOW_SLOW";
        publishAlert(alert, iv.currentFlowRate);
        saveImageToLog();   // 시리얼 출력 없이 바로 파일 저장
      }

      // IV complete
      if (iv.finishWeight > 0 && iv.currentWeight <= iv.finishWeight) {
        iv.complete = true;
        iv.started  = false;
        Serial.println("[IV] Infusion complete.");
        StaticJsonDocument<64> doc;
        doc["type"]        = "IV_COMPLETE";
        doc["finalWeight"] = iv.currentWeight;
        char buf[64]; serializeJson(doc, buf);
        mqtt.publish(T_ALERT, buf, true);
      }

      if (detector.isWindowFull()) {
        Serial.printf("[IV] W:%.2fg  flow:%.4f/%.4fg/s  → %s (신뢰도:%.0f%%)\n",
                      iv.currentWeight,
                      iv.currentFlowRate, iv.targetFlowRate,
                      detector.getResultLabel(),
                      detector.getWindowConfidence() * 100.0f);
      } else {
        Serial.printf("[IV] W:%.2fg  flow:%.4f/%.4fg/s  [수집중 %d/64]\n",
                      iv.currentWeight,
                      iv.currentFlowRate, iv.targetFlowRate,
                      detector.getSampleCount());
      }
    }
  }

  // ── Periodic status publish (every 5 s) ─────────────────────────
  if (iv.started && !iv.complete && now - lastStatusMs >= STATUS_MS) {
    lastStatusMs = now;
    publishStatus();
  }
}
