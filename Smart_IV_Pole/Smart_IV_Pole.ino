/*
 * Smart IV Pole — 스마트 수액 폴 메인 스케치
 *
 * 필요 라이브러리 (Arduino Library Manager):
 *   PubSubClient  (Nick O'Leary)
 *   ArduinoJson   (Benoit Blanchon)
 *
 * WiFi 프로비저닝: "ESP BLE Prov" 앱 사용 (Espressif)
 *   기기 이름 : IV_POLE_BLE
 *   PoP 코드  : ivpole01
 *
 * MQTT 토픽 (client-id = iv_pole_01):
 *   구독  iv_pole/iv_pole_01/config  ← 앱에서 목표 유속 전송
 *   구독  iv_pole/iv_pole_01/cmd     ← tare / reset 명령
 *   발행  iv_pole/iv_pole_01/status  → 무게, 유속, 예상 종료 시각
 *   발행  iv_pole/iv_pole_01/alert   → 이상 감지 / 주입 완료
 *
 * 부팅 절차 (target 명령 입력 시 자동 실행):
 *   1. 영점 조정    — 빈 수액 봉투 상태로 자동 tare
 *   2. 드립 팩터 교정 — 60초간 무게 변화 측정 → g/gtt 계산
 *   3. 교정값 자동 적용 → 목표 유속(g/s) 갱신
 *   4. EMA 안정화 후 모니터링 시작
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiProv.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "ads1232.h"
#include "cnn_detector.h"

// ── 이미지 로그 ───────────────────────────────────────────────────
#define IMAGE_LOG_PATH  "/imglog.csv"

// ── 핀 정의 ───────────────────────────────────────────────────────
#define ADS_DOUT  19   // DRDY/DOUT → IO19
#define ADS_SCLK  18   // SCLK      → IO18
#define ADS_PDWN  23   // PDWN      → IO23
#define ADS_GAIN0 33   // GAIN0     → IO33
#define ADS_GAIN1 32   // GAIN1     → IO32

// ── MQTT 설정 ─────────────────────────────────────────────────────
#define MQTT_BROKER  "192.168.1.100"   // TODO: 실제 브로커 IP로 변경
#define MQTT_PORT    1883
#define MQTT_CLIENT  "iv_pole_01"

#define T_STATUS  "iv_pole/" MQTT_CLIENT "/status"
#define T_ALERT   "iv_pole/" MQTT_CLIENT "/alert"
#define T_CONFIG  "iv_pole/" MQTT_CLIENT "/config"
#define T_CMD     "iv_pole/" MQTT_CLIENT "/cmd"

// ── 타이밍 설정 ───────────────────────────────────────────────────
#define WEIGHT_MS       1000   // 무게 측정 주기 (1초)
#define STATUS_MS       5000   // MQTT 상태 발행 주기 (5초)
#define MQTT_RETRY_MS   5000   // MQTT 재연결 시도 주기

// ── 로드셀 교정값 ─────────────────────────────────────────────────
// calib_factor = (raw - tare) / known_grams
// 분동으로 1회 교정 후 이 값으로 고정
#define CALIB_FACTOR_DEFAULT  1642.8623f

// ── 드립 팩터 기본값 ──────────────────────────────────────────────
// 1 gtt당 무게(g). IV 세트 종류에 따라 다름:
//   20 gtt/mL → 0.0500 g/gtt  (기본, 일반 성인용)
//   15 gtt/mL → 0.0667 g/gtt
//   10 gtt/mL → 0.1000 g/gtt
//   60 gtt/mL → 0.0167 g/gtt  (소아용 마이크로드립)
#define DRIP_FACTOR_DEFAULT  0.0500f

// ── BLE 프로비저닝 설정 ───────────────────────────────────────────
#define BLE_NAME  "IV_POLE_BLE"
#define PROV_POP  "ivpole01"

// ── EMA 안정화 샘플 수 ────────────────────────────────────────────
// 측정 시작 전 EMA 필터가 수렴할 때까지 대기하는 샘플 수
#define WARMUP_SAMPLES  8

// ── 드립 팩터 교정 측정 시간 ──────────────────────────────────────
// 자동 교정 시 무게 변화를 측정하는 시간 (초)
#define CALIB_DURATION_MS  60000UL   // 60초

// ── 부팅 절차 단계 정의 ───────────────────────────────────────────
enum IVPhase {
  PHASE_IDLE,     // 대기 — target 명령 입력 대기
  PHASE_TARE,     // 영점 조정 중
  PHASE_CALIB,    // 드립 팩터 교정 중 (60초 측정)
  PHASE_WARMUP,   // EMA 안정화 대기
  PHASE_MONITOR,  // 주입 모니터링 중
  PHASE_DONE      // 주입 완료
};

// ── 전역 객체 ─────────────────────────────────────────────────────
ADS1232      loadCell(ADS_DOUT, ADS_SCLK, ADS_PDWN, ADS_GAIN0, ADS_GAIN1);
WiFiClient   espClient;
PubSubClient mqtt(espClient);
CNNDetector  detector;

// ── IV 상태 구조체 ────────────────────────────────────────────────
struct IVState {
  float targetFlowRate = 0;                    // 목표 유속 (g/s)
  float finishWeight   = 0;                    // 주입 종료 무게 (g)
  float currentWeight  = 0;                    // 현재 무게 (g)
  float prevWeight     = 0;                    // 이전 측정 무게 (g)
  float currentFlowRate= 0;                    // 현재 유속 (g/s)
  float dripFactor     = DRIP_FACTOR_DEFAULT;  // 드립 팩터 (g/gtt)
  bool  started        = false;
  bool  complete       = false;
  int   warmup         = 0;                    // 남은 안정화 횟수
} iv;

// ── 부팅 절차 상태 변수 ───────────────────────────────────────────
IVPhase       ivPhase         = PHASE_IDLE;
float         calibTargetGtt  = 0;     // 교정 시 목표 gtt/min
float         calibWeightStart= 0;     // 교정 시작 무게 (g)
unsigned long calibStartMs    = 0;     // 교정 시작 시각

// ── 타이밍 변수 ───────────────────────────────────────────────────
unsigned long lastWeightMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastMqttMs   = 0;

// ─────────────────────────────────────────────────────────────────
// BLE 프로비저닝 이벤트 콜백
// ─────────────────────────────────────────────────────────────────
void onProvEvent(arduino_event_t *ev) {
  switch (ev->event_id) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WiFi] 연결 끊김.");
      break;
    case ARDUINO_EVENT_PROV_START:
      Serial.printf("[Prov] BLE 준비. Device=%s  PoP=%s\n", BLE_NAME, PROV_POP);
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
      Serial.println("[Prov] 자격증명 수신.");
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("[Prov] WiFi 확인 완료.");
      break;
    case ARDUINO_EVENT_PROV_END:
      Serial.println("[Prov] 프로비저닝 완료.");
      break;
    default: break;
  }
}

// ─────────────────────────────────────────────────────────────────
// MQTT 헬퍼 함수
// ─────────────────────────────────────────────────────────────────

// 이상 감지 알림 발행
void publishAlert(const char *type, float measuredFlowRate) {
  StaticJsonDocument<128> doc;
  doc["type"]         = type;
  doc["flowMeasured"] = measuredFlowRate;
  doc["flowTarget"]   = iv.targetFlowRate;
  doc["confidence"]   = detector.getWindowConfidence();
  doc["ts"]           = millis();
  char buf[128];
  serializeJson(doc, buf);
  mqtt.publish(T_ALERT, buf, true);
  Serial.printf("[ALERT] %s  신뢰도:%.0f%%\n", type, detector.getWindowConfidence() * 100.0f);
}

// 현재 상태 발행 (5초 주기)
void publishStatus() {
  StaticJsonDocument<256> doc;
  doc["weight"]     = round(iv.currentWeight    * 100) / 100.0;
  doc["flowRate"]   = round(iv.currentFlowRate  * 10000) / 10000.0;
  doc["state"]      = detector.getLastState();
  doc["result"]     = detector.getResultLabel();
  doc["confidence"] = round(detector.getWindowConfidence() * 1000) / 1000.0;
  if (iv.targetFlowRate > 0 && iv.currentWeight > iv.finishWeight)
    doc["etaSec"] = (int)((iv.currentWeight - iv.finishWeight) / iv.targetFlowRate);
  char buf[256];
  serializeJson(doc, buf);
  mqtt.publish(T_STATUS, buf);
}

// ─────────────────────────────────────────────────────────────────
// MQTT 수신 메시지 처리
// ─────────────────────────────────────────────────────────────────
void onMqttMsg(char *topic, byte *payload, unsigned int len) {
  payload[len] = '\0';
  Serial.printf("[MQTT←] %s : %s\n", topic, (char *)payload);

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, len)) return;

  String t = topic;

  if (t == T_CONFIG) {
    // 앱에서 전송: { "targetFlowRate": 60, "finishWeight": 50 }  (gtt/min)
    if (doc.containsKey("targetFlowRate")) {
      calibTargetGtt = doc["targetFlowRate"].as<float>();
      ivPhase        = PHASE_TARE;   // 부팅 절차 시작
      Serial.printf("[BOOT] MQTT target=%.0f gtt/min — 부팅 절차 시작\n", calibTargetGtt);
    }
    if (doc.containsKey("finishWeight"))
      iv.finishWeight = doc["finishWeight"].as<float>();
  }

  if (t == T_CMD) {
    String cmd = doc["cmd"].as<String>();
    if (cmd == "tare") {
      Serial.println("[CMD] 영점 조정 중...");
      loadCell.tare(20);
      Serial.println("[CMD] 완료.");
    } else if (cmd == "reset") {
      iv      = IVState{};
      ivPhase = PHASE_IDLE;
      detector.reset();
      Serial.println("[CMD] 초기화 완료.");
    }
  }
}

bool connectMQTT() {
  if (mqtt.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  Serial.printf("[MQTT] 연결 중 %s:%d ...\n", MQTT_BROKER, MQTT_PORT);
  if (mqtt.connect(MQTT_CLIENT)) {
    mqtt.subscribe(T_CONFIG);
    mqtt.subscribe(T_CMD);
    Serial.println("[MQTT] 연결 완료.");
    return true;
  }
  Serial.printf("[MQTT] 실패, state=%d\n", mqtt.state());
  return false;
}

// ─────────────────────────────────────────────────────────────────
// CNN 이미지 시각화 & LittleFS 저장
// ─────────────────────────────────────────────────────────────────

// 현재 CNN 이미지를 시리얼로 출력 (ASCII art)
void printImage() {
  int img[CNN_DIM][CNN_DIM];
  detector.getImage(img);

  Serial.println("[CNN] ┌────┐");
  for (int r = 0; r < CNN_DIM; r++) {
    Serial.print("      │");
    for (int c = 0; c < CNN_DIM; c++) {
      if      (img[r][c] == FLOW_FAST) Serial.print('+');
      else if (img[r][c] == FLOW_SLOW) Serial.print('-');
      else                             Serial.print('.');
    }
    Serial.println("│");
  }
  Serial.println("      └────┘");
  Serial.printf("  + 빠름  · 정상  - 느림   score:%.3f\n",
                detector.getWindowConfidence());
}

// 이상 감지 시 LittleFS에 이미지 저장
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
  f.println();
  f.close();
  Serial.println("[LOG] 이미지 저장됨");
}

// 저장된 로그 전체 시리얼 출력
void dumpImageLog() {
  File f = LittleFS.open(IMAGE_LOG_PATH, "r");
  if (!f) { Serial.println("[LOG] 저장된 로그 없음"); return; }
  Serial.println("[LOG] ── imglog.csv 시작 ──────────────");
  while (f.available()) Serial.write(f.read());
  f.close();
  Serial.println("[LOG] ── 끝 ────────────────────────────");
}

void clearImageLog() {
  LittleFS.remove(IMAGE_LOG_PATH);
  Serial.println("[LOG] 로그 삭제됨");
}

// ─────────────────────────────────────────────────────────────────
// 시리얼 명령 처리
// ─────────────────────────────────────────────────────────────────
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("target ")) {
    // ── 부팅 절차 시작 ────────────────────────────────────────────
    // 순서: 영점 조정 → 드립 팩터 교정(60초) → 교정값 적용 → 측정 시작
    calibTargetGtt = line.substring(7).toFloat();
    ivPhase        = PHASE_TARE;
    Serial.printf("[BOOT] target=%.0f gtt/min — 부팅 절차 시작\n", calibTargetGtt);

  } else if (line.startsWith("finish ")) {
    iv.finishWeight = line.substring(7).toFloat();
    Serial.printf("[CMD] 종료 무게=%.2f g\n", iv.finishWeight);

  } else if (line == "tare") {
    Serial.println("[CMD] 영점 조정 중...");
    loadCell.tare(20);
    Serial.println("[CMD] 완료.");

  } else if (line == "reset") {
    iv      = IVState{};
    ivPhase = PHASE_IDLE;
    detector.reset();
    Serial.println("[CMD] 초기화 완료.");

  } else if (line == "status") {
    // 현재 상태 출력
    const char *phaseStr[] = { "대기", "영점조정", "드립팩터교정", "안정화", "모니터링", "완료" };
    float targetGtt = (iv.dripFactor > 0)
                    ? (iv.targetFlowRate / iv.dripFactor) * 60.0f : 0;
    Serial.printf("[STATUS] 단계:%s  W:%.2fg  유속:%.4fg/s  목표:%.4fg/s (%.1fgtt/min)\n"
                  "         dripFactor:%.5fg/gtt  결과:%s  신뢰도:%.0f%%  CNN:%s  샘플:%d/%d\n",
                  phaseStr[ivPhase],
                  iv.currentWeight, iv.currentFlowRate, iv.targetFlowRate, targetGtt,
                  iv.dripFactor,
                  detector.getResultLabel(),
                  detector.getWindowConfidence() * 100.0f,
                  detector.isTFLiteActive() ? "ON" : "fallback",
                  detector.getSampleCount(), CNN_WIN);

  } else if (line.startsWith("tolerance ")) {
    // 유속 허용 오차 비율 설정 (0.05 ~ 0.50)
    // 예) tolerance 0.17 → 목표 60gtt 기준 ±17% = 50~70gtt 정상
    float t = line.substring(10).toFloat();
    if (t <= 0 || t >= 1.0f) {
      Serial.println("[CMD] tolerance 범위: 0.05 ~ 0.50  예) tolerance 0.17");
    } else {
      detector.setTolerance(t);
      float targetGtt = (iv.targetFlowRate > 0 && iv.dripFactor > 0)
                      ? (iv.targetFlowRate / iv.dripFactor) * 60.0f : 0;
      Serial.printf("[CMD] tolerance=±%.0f%%", t * 100.0f);
      if (targetGtt > 0)
        Serial.printf("  → 정상 범위: %.0f ~ %.0f gtt/min",
                      targetGtt * (1.0f - t), targetGtt * (1.0f + t));
      Serial.println();
    }

  } else if (line.startsWith("dropfactor ")) {
    // 드립 팩터 수동 설정 (g/gtt)
    // 자동 교정 대신 수동으로 입력할 때 사용
    float df = line.substring(11).toFloat();
    if (df <= 0) {
      Serial.println("[CMD] 값 오류 (예: dropfactor 0.05)");
    } else {
      iv.dripFactor = df;
      Serial.printf("[CMD] dripFactor=%.5f g/gtt\n", iv.dripFactor);
      Serial.println("[CMD] ※ 'target <gtt/min>' 다시 입력해야 g/s 값이 갱신됨");
    }

  } else if (line.startsWith("alpha ")) {
    float a = constrain(line.substring(6).toFloat(), 0.05f, 0.5f);
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
    Serial.println("[CMD] 명령어 목록:");
    Serial.println("  target <gtt/min>      부팅 절차 시작 (자동 영점+교정+모니터링)");
    Serial.println("  finish <g>            주입 종료 무게 설정");
    Serial.println("  tolerance <0.05~0.5>  유속 허용 오차 (기본 0.17 = ±17%)");
    Serial.println("  dropfactor <g/gtt>    드립 팩터 수동 설정");
    Serial.println("  tare                  영점 조정");
    Serial.println("  reset                 전체 초기화");
    Serial.println("  status                현재 상태 출력");
    Serial.println("  alpha <0.05~0.5>      EMA 부드러움 조절");
    Serial.println("  image                 CNN 이미지 출력");
    Serial.println("  savelog / dumplog / clearlog");
  }
}

// ─────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Smart IV Pole ===");

  // LittleFS 마운트
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS 마운트 실패 — 로그 저장 비활성화");
  } else {
    Serial.println("[FS] LittleFS OK");
  }

  // 로드셀 초기화
  loadCell.begin(128);
  loadCell.setCalibFactor(CALIB_FACTOR_DEFAULT);
  Serial.printf("[ADS] 초기화 완료. CalibFactor=%.2f\n", loadCell.getCalibFactor());

  // BLE WiFi 프로비저닝 (논블로킹, 최대 10초 대기)
  WiFi.onEvent(onProvEvent);
  WiFiProv.beginProvision(
    NETWORK_PROV_SCHEME_BLE,
    NETWORK_PROV_SCHEME_HANDLER_FREE_BLE,
    NETWORK_PROV_SECURITY_1,
    PROV_POP,
    BLE_NAME
  );
  Serial.println("[WiFi] 프로비저닝 대기 중 (NVS 저장값 있으면 자동 연결)...");
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED)
    Serial.println("[WiFi] 미연결 — 오프라인 모드 (MQTT 비활성화).");

  // CNN 탐지기 초기화
  if (!detector.begin()) {
    Serial.println("[CNN] Fallback 모드 — train/train_cnn.py 실행 후 재컴파일 필요.");
  }

  // MQTT 설정
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMqttMsg);
  connectMQTT();

  Serial.println("[SYS] 준비 완료. 'target <gtt/min>' 입력으로 측정을 시작하세요.");
  lastWeightMs = lastStatusMs = millis();
}

// ─────────────────────────────────────────────────────────────────
// loop
// ─────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  handleSerial();

  // WiFi 재연결 감시 (논블로킹)
  if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();

  // MQTT 연결 유지 및 재연결
  if (!mqtt.connected() && now - lastMqttMs > MQTT_RETRY_MS) {
    lastMqttMs = now;
    connectMQTT();
  }
  mqtt.loop();

  // ── 부팅 절차 상태 머신 ──────────────────────────────────────────

  if (ivPhase == PHASE_TARE) {
    // 1단계: 영점 조정
    Serial.println("[BOOT] 1/4 영점 조정 중...");
    loadCell.tare(20);
    calibWeightStart = loadCell.stableRead(10);
    calibStartMs     = millis();
    ivPhase          = PHASE_CALIB;
    Serial.printf("[BOOT] 2/4 드립 팩터 교정 시작 (%lu초 측정)...\n",
                  CALIB_DURATION_MS / 1000UL);
    Serial.printf("[BOOT]     IV가 %.0f gtt/min 속도로 흐르는지 확인하세요.\n",
                  calibTargetGtt);
  }

  if (ivPhase == PHASE_CALIB && now - calibStartMs >= CALIB_DURATION_MS) {
    // 2단계: 드립 팩터 교정 완료
    float weightEnd  = loadCell.stableRead(10);
    float weightLost = calibWeightStart - weightEnd;

    if (weightLost > 0.05f && calibTargetGtt > 0) {
      // dripFactor = 60초 무게 감소(g) ÷ 60초간 방울 수(= gtt/min)
      iv.dripFactor = weightLost / calibTargetGtt;
      Serial.printf("[BOOT]     교정 완료: %.5f g/gtt  (무게 감소 %.3fg / %.0f방울)\n",
                    iv.dripFactor, weightLost, calibTargetGtt);
    } else {
      Serial.printf("[BOOT]     교정 실패 (변화량 %.3fg — 기본값 %.5f 사용)\n",
                    weightLost, iv.dripFactor);
    }

    // 3단계: 교정된 드립 팩터로 목표 유속 계산
    iv.targetFlowRate = (calibTargetGtt / 60.0f) * iv.dripFactor;
    detector.reset();
    Serial.printf("[BOOT] 3/4 교정값 적용: 목표 유속 %.4f g/s (%.0f gtt/min)\n",
                  iv.targetFlowRate, calibTargetGtt);

    // 4단계: EMA 안정화 준비
    loadCell.resetEma();
    iv.currentWeight = loadCell.readWeight();
    iv.prevWeight    = iv.currentWeight;
    iv.started       = true;
    iv.complete      = false;
    iv.warmup        = WARMUP_SAMPLES;
    ivPhase          = PHASE_WARMUP;
    Serial.printf("[BOOT] 4/4 EMA 안정화 중... (%d 샘플)\n", WARMUP_SAMPLES);
  }

  // ── 무게 측정 주기 (1초) ─────────────────────────────────────────
  if (now - lastWeightMs >= WEIGHT_MS) {
    lastWeightMs = now;

    iv.prevWeight    = iv.currentWeight;
    iv.currentWeight = loadCell.readWeight();

    if (iv.started && !iv.complete) {

      if (iv.warmup > 0) {
        // EMA 안정화 대기: 무게 차 계산 없이 prevWeight를 최신값으로 유지
        iv.prevWeight = iv.currentWeight;
        iv.warmup--;
        Serial.printf("[BOOT] 안정화 중... W:%.2fg  (%d 샘플 남음)\n",
                      iv.currentWeight, iv.warmup);
        if (iv.warmup == 0) {
          ivPhase = PHASE_MONITOR;
          Serial.println("[BOOT] 측정 시작!");
        }
      } else {
        // ── 정상 모니터링 ─────────────────────────────────────────

        // 유속(g/s) = 무게 감소량(g) ÷ 측정 주기(s)
        iv.currentFlowRate = (iv.prevWeight - iv.currentWeight)
                             / (WEIGHT_MS / 1000.0f);

        // CNN 샘플 추가 (16개 채워지면 자동 분류)
        detector.addSample(iv.currentFlowRate, iv.targetFlowRate);

        // 이상 감지 — 윈도우 완성 직후 1회만 발동
        if (detector.detectAnomaly()) {
          const char *alert = (detector.getWindowResult() == FLOW_FAST)
                              ? "FLOW_FAST" : "FLOW_SLOW";
          publishAlert(alert, iv.currentFlowRate);
          saveImageToLog();
        }

        // 주입 완료 판정
        if (iv.finishWeight > 0 && iv.currentWeight <= iv.finishWeight) {
          iv.complete = true;
          iv.started  = false;
          ivPhase     = PHASE_DONE;
          Serial.println("[IV] 주입 완료.");
          StaticJsonDocument<64> doc;
          doc["type"]        = "IV_COMPLETE";
          doc["finalWeight"] = iv.currentWeight;
          char buf[64]; serializeJson(doc, buf);
          mqtt.publish(T_ALERT, buf, true);
        }

        // 윈도우 완성 직후(count==0)에만 결과 출력,
        // 그 외에는 수집 진행률 표시
        if (detector.getSampleCount() == 0 && detector.isWindowFull()) {
          Serial.printf("[IV] W:%.2fg  유속:%.4f/%.4fg/s  → %s (신뢰도:%.0f%%)\n",
                        iv.currentWeight,
                        iv.currentFlowRate, iv.targetFlowRate,
                        detector.getResultLabel(),
                        detector.getWindowConfidence() * 100.0f);
        } else {
          Serial.printf("[IV] W:%.2fg  유속:%.4f/%.4fg/s  [수집중 %d/%d]\n",
                        iv.currentWeight,
                        iv.currentFlowRate, iv.targetFlowRate,
                        detector.getSampleCount(), CNN_WIN);
        }
      }
    }
  }

  // ── MQTT 상태 발행 (5초 주기) ────────────────────────────────────
  if (iv.started && !iv.complete && now - lastStatusMs >= STATUS_MS) {
    lastStatusMs = now;
    publishStatus();
  }
}
