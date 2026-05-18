/*
 * Smart IV Pole — 스마트 수액 폴 메인 스케치 (실무용)
 *
 * ── 자동 부팅 절차 (별도 조작 불필요) ────────────────────────────────
 *   전원 ON  →  2초 대기  →  자동 영점 조정
 *              →  수액 감지(무게 > 50g)  →  60초 드립 팩터 교정
 *              →  EMA 안정화  →  모니터링 시작
 *   수액 제거 감지 시  →  자동 재영점·재시작
 *
 * ── 목표 유속 설정 (MQTT 앱에서 입력) ───────────────────────────────
 *   기본값  : 60 gtt/min (성인 일반 세트)
 *   앱 변경 : T_CONFIG {"targetFlowRate": <gtt/min>, "finishWeight": <g>}
 *
 * ── 필요 라이브러리 (Arduino Library Manager) ───────────────────────
 *   PubSubClient  (Nick O'Leary)
 *   ArduinoJson   (Benoit Blanchon)
 *
 * ── WiFi 프로비저닝: "ESP BLE Prov" 앱 사용 (Espressif) ────────────
 *   기기 이름 : IV_POLE_BLE  /  PoP 코드 : ivpole01
 *
 * ── MQTT 토픽 (client-id = iv_pole_01) ─────────────────────────────
 *   구독  iv_pole/iv_pole_01/config  ← 목표 유속·종료 무게 설정
 *   구독  iv_pole/iv_pole_01/cmd     ← tare / reset
 *   발행  iv_pole/iv_pole_01/status  → 무게·유속·예상 종료
 *   발행  iv_pole/iv_pole_01/alert   → 이상·주입완료
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiProv.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "ads1232.h"
#include "cnn_detector.h"

// ── 이미지 로그 파일 경로 ─────────────────────────────────────────
#define IMAGE_LOG_PATH  "/imglog.csv"

// ── 핀 정의 ──────────────────────────────────────────────────────
#define ADS_DOUT  19   // DRDY/DOUT
#define ADS_SCLK  18   // SCLK
#define ADS_PDWN  23   // PDWN
#define ADS_GAIN0 33   // GAIN0
#define ADS_GAIN1 32   // GAIN1

// ── MQTT 설정 ─────────────────────────────────────────────────────
#define MQTT_BROKER  "192.168.1.100"
#define MQTT_PORT    1883
#define MQTT_CLIENT  "iv_pole_01"

#define T_STATUS  "iv_pole/" MQTT_CLIENT "/status"
#define T_ALERT   "iv_pole/" MQTT_CLIENT "/alert"
#define T_CONFIG  "iv_pole/" MQTT_CLIENT "/config"
#define T_CMD     "iv_pole/" MQTT_CLIENT "/cmd"

// ── 타이밍 설정 ───────────────────────────────────────────────────
#define WEIGHT_MS         1000     // 무게 측정 주기 (1초)
#define STATUS_MS         5000     // MQTT 상태 발행 주기 (5초)
#define MQTT_RETRY_MS     5000     // MQTT 재연결 시도 주기
#define BOOT_DELAY_MS     2000UL   // 전원 ON 후 영점 조정까지 대기 시간

// ── 드립 팩터 교정 ─────────────────────────────────────────────────
#define CALIB_DURATION_MS  60000UL   // 교정 측정 시간 (60초)

// ── 수액 자동 감지 임계값 ─────────────────────────────────────────
#define WEIGHT_HANG_G    50.0f   // 수액팩 감지: 이 무게 이상이면 수액 걸린 것으로 판단
#define WEIGHT_REMOVE_G  15.0f   // 수액팩 제거: 이 무게 미만으로 내려가면 수액 제거로 판단

// ── 기본 목표 유속 ────────────────────────────────────────────────
// MQTT 앱에서 변경 가능. 앱 미사용 시 이 값으로 교정·모니터링.
#define DEFAULT_TARGET_GTT  60.0f   // gtt/min (성인용 20gtt/mL 세트 기준)

// ── 로드셀 교정 계수 ─────────────────────────────────────────────
// 분동으로 1회 교정 후 이 값으로 고정
// calib_factor = (raw - tare) / known_grams
#define CALIB_FACTOR_DEFAULT  1642.8623f

// ── 드립 팩터 기본값 ──────────────────────────────────────────────
// 자동 교정 실패 시 사용하는 백업값 (20gtt/mL 세트 기준)
#define DRIP_FACTOR_DEFAULT  0.0500f

// ── BLE 프로비저닝 설정 ───────────────────────────────────────────
#define BLE_NAME  "IV_POLE_BLE"
#define PROV_POP  "ivpole01"

// ── EMA 안정화 샘플 수 ────────────────────────────────────────────
#define WARMUP_SAMPLES  8

// ─────────────────────────────────────────────────────────────────
// 부팅 절차 단계 정의
// ─────────────────────────────────────────────────────────────────
enum IVPhase {
  PHASE_BOOT,     // 전원 ON — 2초 대기 중
  PHASE_TARE,     // 자동 영점 조정 중
  PHASE_WAIT,     // 수액 감지 대기 (무게 임계값 감시)
  PHASE_CALIB,    // 드립 팩터 교정 중 (60초)
  PHASE_WARMUP,   // EMA 안정화 대기 (8샘플)
  PHASE_MONITOR,  // 주입 모니터링 중
  PHASE_DONE      // 주입 완료 (수액 제거 대기)
};

// ─────────────────────────────────────────────────────────────────
// 전역 객체
// ─────────────────────────────────────────────────────────────────
ADS1232      loadCell(ADS_DOUT, ADS_SCLK, ADS_PDWN, ADS_GAIN0, ADS_GAIN1);
WiFiClient   espClient;
PubSubClient mqtt(espClient);
CNNDetector  detector;

// ─────────────────────────────────────────────────────────────────
// IV 상태 구조체
// ─────────────────────────────────────────────────────────────────
struct IVState {
  float targetFlowRate = 0;                    // 목표 유속 (g/s)
  float finishWeight   = 0;                    // 주입 종료 무게 (g), 0 = 미설정
  float currentWeight  = 0;                    // 현재 무게 (g)
  float prevWeight     = 0;                    // 이전 측정 무게 (g)
  float currentFlowRate= 0;                    // 현재 유속 (g/s)
  float dripFactor     = DRIP_FACTOR_DEFAULT;  // 드립 팩터 (g/gtt)
  bool  started        = false;
  bool  complete       = false;
  int   warmup         = 0;                    // 남은 안정화 샘플 수
} iv;

// ─────────────────────────────────────────────────────────────────
// 부팅 절차 상태 변수
// ─────────────────────────────────────────────────────────────────
IVPhase       ivPhase          = PHASE_BOOT;
float         calibTargetGtt   = DEFAULT_TARGET_GTT;  // 교정 목표 유속 (gtt/min)
float         calibWeightStart = 0;                   // 교정 시작 무게 (g)
unsigned long calibStartMs     = 0;                   // 교정 시작 시각
unsigned long bootMs           = 0;                   // 전원 ON 시각

// ─────────────────────────────────────────────────────────────────
// 타이밍 변수
// ─────────────────────────────────────────────────────────────────
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
  Serial.printf("[ALERT] %s  신뢰도:%.0f%%\n",
                type, detector.getWindowConfidence() * 100.0f);
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
    // 앱에서 전송: { "targetFlowRate": 60, "finishWeight": 50 }
    if (doc.containsKey("targetFlowRate")) {
      calibTargetGtt = doc["targetFlowRate"].as<float>();
      Serial.printf("[CFG] 목표 유속 변경: %.0f gtt/min\n", calibTargetGtt);

      // 모니터링 중 유속 변경 시 목표 즉시 반영
      if (ivPhase == PHASE_MONITOR || ivPhase == PHASE_WARMUP) {
        iv.targetFlowRate = (calibTargetGtt / 60.0f) * iv.dripFactor;
        detector.reset();
        Serial.printf("[CFG] 유속 즉시 반영: %.4f g/s\n", iv.targetFlowRate);
      }
    }
    if (doc.containsKey("finishWeight")) {
      iv.finishWeight = doc["finishWeight"].as<float>();
      Serial.printf("[CFG] 종료 무게: %.1f g\n", iv.finishWeight);
    }
  }

  if (t == T_CMD) {
    String cmd = doc["cmd"].as<String>();
    if (cmd == "tare") {
      // 수동 재영점 명령
      Serial.println("[CMD] 영점 조정 중...");
      loadCell.tare(20);
      Serial.println("[CMD] 완료.");
    } else if (cmd == "reset") {
      // 전체 초기화 → PHASE_TARE로 돌아가 재시작
      iv      = IVState{};
      ivPhase = PHASE_TARE;
      detector.reset();
      Serial.println("[CMD] 초기화 완료 — 재영점 시작.");
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
// CNN 이미지 시각화 & LittleFS 로그
// ─────────────────────────────────────────────────────────────────

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
// 시리얼 디버그 명령 처리 (엔지니어·개발용, 실무 사용 불필요)
// ─────────────────────────────────────────────────────────────────
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("target ")) {
    calibTargetGtt = line.substring(7).toFloat();
    Serial.printf("[DBG] 목표 유속 변경: %.0f gtt/min\n", calibTargetGtt);
    if (ivPhase == PHASE_MONITOR || ivPhase == PHASE_WARMUP) {
      iv.targetFlowRate = (calibTargetGtt / 60.0f) * iv.dripFactor;
      detector.reset();
      Serial.printf("[DBG] 즉시 반영: %.4f g/s\n", iv.targetFlowRate);
    }

  } else if (line.startsWith("finish ")) {
    iv.finishWeight = line.substring(7).toFloat();
    Serial.printf("[DBG] 종료 무게: %.2f g\n", iv.finishWeight);

  } else if (line == "tare") {
    loadCell.tare(20);
    Serial.println("[DBG] 영점 완료.");

  } else if (line == "reset") {
    iv      = IVState{};
    ivPhase = PHASE_TARE;
    detector.reset();
    Serial.println("[DBG] 초기화 완료 — 재영점 시작.");

  } else if (line == "status") {
    const char *phaseStr[] = { "부팅대기", "영점조정", "수액대기", "드립팩터교정", "안정화", "모니터링", "완료" };
    Serial.printf("[STATUS] 단계:%s  W:%.2fg  유속:%.4f/%.4fg/s  dripF:%.5f\n"
                  "         결과:%s  신뢰도:%.0f%%  CNN:%s  샘플:%d/%d\n",
                  phaseStr[ivPhase],
                  iv.currentWeight, iv.currentFlowRate, iv.targetFlowRate,
                  iv.dripFactor,
                  detector.getResultLabel(),
                  detector.getWindowConfidence() * 100.0f,
                  detector.isTFLiteActive() ? "ON" : "fallback",
                  detector.getSampleCount(), CNN_WIN);

  } else if (line.startsWith("tolerance ")) {
    float t = line.substring(10).toFloat();
    if (t > 0 && t < 1.0f) {
      detector.setTolerance(t);
      Serial.printf("[DBG] tolerance=±%.0f%%\n", t * 100.0f);
    }

  } else if (line.startsWith("alpha ")) {
    float a = constrain(line.substring(6).toFloat(), 0.05f, 0.5f);
    loadCell.setEmaAlpha(a);
    Serial.printf("[DBG] EMA alpha=%.2f\n", a);

  } else if (line == "image")    { printImage();    }
  else if (line == "dumplog")    { dumpImageLog();  }
  else if (line == "clearlog")   { clearImageLog(); }
  else {
    Serial.println("[DBG] 명령: target / finish / tare / reset / status");
    Serial.println("           tolerance / alpha / image / dumplog / clearlog");
  }
}

// ─────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Smart IV Pole (실무용) ===");

  // LittleFS 마운트
  if (!LittleFS.begin(true))
    Serial.println("[FS] LittleFS 마운트 실패 — 로그 저장 비활성화");
  else
    Serial.println("[FS] LittleFS OK");

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
  Serial.println("[WiFi] 프로비저닝 대기 중...");
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED)
    Serial.println("[WiFi] 미연결 — 오프라인 모드.");

  // CNN 탐지기 초기화
  if (!detector.begin())
    Serial.println("[CNN] Fallback 모드.");

  // MQTT 설정
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMqttMsg);
  connectMQTT();

  // 부팅 시각 기록 → 2초 후 자동 영점 시작
  bootMs = millis();
  Serial.printf("[SYS] 준비 완료. %.1f초 후 자동 영점 조정 시작.\n",
                BOOT_DELAY_MS / 1000.0f);
  Serial.printf("[SYS] 기본 목표 유속: %.0f gtt/min (앱에서 변경 가능)\n",
                calibTargetGtt);

  lastWeightMs = lastStatusMs = millis();
}

// ─────────────────────────────────────────────────────────────────
// loop
// ─────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // 디버그 시리얼 처리
  handleSerial();

  // WiFi 재연결 감시 (논블로킹)
  if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();

  // MQTT 연결 유지 및 재연결
  if (!mqtt.connected() && now - lastMqttMs > MQTT_RETRY_MS) {
    lastMqttMs = now;
    connectMQTT();
  }
  mqtt.loop();

  // =================================================================
  // 자동 부팅 절차 상태 머신
  // =================================================================

  // ── 1단계: 부팅 대기 (2초) ────────────────────────────────────────
  if (ivPhase == PHASE_BOOT) {
    if (now - bootMs >= BOOT_DELAY_MS) {
      ivPhase = PHASE_TARE;
    }
    return;   // 2초 대기 중에는 아래 루프 실행 안 함
  }

  // ── 2단계: 자동 영점 조정 ─────────────────────────────────────────
  // PHASE_TARE는 blocking이지만 200ms 이내 완료 (tare 20 샘플)
  if (ivPhase == PHASE_TARE) {
    Serial.println("[AUTO] 영점 조정 중...");
    loadCell.tare(20);
    iv = IVState{};          // 상태 전체 초기화
    detector.reset();
    ivPhase = PHASE_WAIT;
    Serial.println("[AUTO] 영점 완료. 수액팩을 걸어주세요.");
    Serial.printf("[AUTO] 감지 임계값: %.0fg 이상\n", WEIGHT_HANG_G);
    lastWeightMs = millis();
    return;
  }

  // ── 3단계: 수액 감지 대기 ─────────────────────────────────────────
  // 1초 주기로 무게 확인 → WEIGHT_HANG_G 초과 시 교정 시작
  if (ivPhase == PHASE_WAIT && now - lastWeightMs >= WEIGHT_MS) {
    lastWeightMs = now;
    float w = loadCell.readWeight();
    iv.currentWeight = w;

    if (w > WEIGHT_HANG_G) {
      Serial.printf("[AUTO] 수액 감지 (%.1fg) — 드립 팩터 교정 시작\n", w);
      Serial.printf("[AUTO] 목표 유속: %.0f gtt/min | 교정 시간: %lu초\n",
                    calibTargetGtt, CALIB_DURATION_MS / 1000UL);
      Serial.println("[AUTO] IV가 정상 속도로 흐르는지 확인하세요.");
      // 교정 기준 무게 스냅샷 (EMA 없는 정밀 측정)
      calibWeightStart = loadCell.stableRead(10);
      calibStartMs     = millis();
      ivPhase          = PHASE_CALIB;
    }
    return;
  }

  // ── 4단계: 드립 팩터 교정 (60초 측정) ────────────────────────────
  if (ivPhase == PHASE_CALIB && now - calibStartMs >= CALIB_DURATION_MS) {
    float weightEnd  = loadCell.stableRead(10);
    float weightLost = calibWeightStart - weightEnd;

    Serial.printf("[AUTO] 교정 측정 완료: %.3fg 감소 / %.0f방울\n",
                  weightLost, calibTargetGtt);

    if (weightLost > 0.05f && calibTargetGtt > 0) {
      // dripFactor = 60초 무게 감소(g) ÷ 60초간 방울 수(= gtt/min × 1min)
      iv.dripFactor = weightLost / calibTargetGtt;
      Serial.printf("[AUTO] 드립 팩터 교정값: %.5f g/gtt\n", iv.dripFactor);
    } else {
      // 무게 변화 미미 → 기본값 사용 (IV가 흐르지 않았거나 너무 느린 경우)
      Serial.printf("[AUTO] 교정 실패 (변화량 부족) — 기본값 %.5f g/gtt 사용\n",
                    iv.dripFactor);
    }

    // 교정된 드립 팩터로 목표 유속(g/s) 계산
    iv.targetFlowRate = (calibTargetGtt / 60.0f) * iv.dripFactor;
    Serial.printf("[AUTO] 목표 유속 확정: %.4f g/s (%.0f gtt/min)\n",
                  iv.targetFlowRate, calibTargetGtt);

    // EMA 초기화 후 안정화 대기
    loadCell.resetEma();
    iv.currentWeight = loadCell.readWeight();
    iv.prevWeight    = iv.currentWeight;
    iv.started       = true;
    iv.complete      = false;
    iv.warmup        = WARMUP_SAMPLES;
    ivPhase          = PHASE_WARMUP;
    Serial.printf("[AUTO] EMA 안정화 중... (%d 샘플)\n", WARMUP_SAMPLES);
    lastWeightMs = millis();
    return;
  }

  // =================================================================
  // 무게 측정 주기 (1초)
  // 교정 중(PHASE_CALIB) / 부팅 대기(PHASE_BOOT/WAIT) 제외하고 실행
  // =================================================================
  if (ivPhase != PHASE_WAIT && now - lastWeightMs >= WEIGHT_MS) {
    lastWeightMs = now;

    iv.prevWeight    = iv.currentWeight;
    iv.currentWeight = loadCell.readWeight();

    // ── 안정화 단계 ────────────────────────────────────────────────
    if (ivPhase == PHASE_WARMUP) {
      // EMA가 수렴할 때까지 prevWeight를 계속 갱신 (유속 계산 안 함)
      iv.prevWeight = iv.currentWeight;
      iv.warmup--;
      Serial.printf("[AUTO] 안정화 중... W:%.2fg  (남은 샘플 %d개)\n",
                    iv.currentWeight, iv.warmup);
      if (iv.warmup == 0) {
        ivPhase = PHASE_MONITOR;
        Serial.println("[AUTO] ✓ 모니터링 시작!");
      }
      return;
    }

    // ── 모니터링 단계 ──────────────────────────────────────────────
    if (ivPhase == PHASE_MONITOR) {

      // 수액팩 제거 감지 → 자동 재영점
      if (iv.currentWeight < WEIGHT_REMOVE_G) {
        Serial.println("[AUTO] 수액 제거 감지 — 자동 재영점 후 재시작.");
        iv      = IVState{};
        ivPhase = PHASE_TARE;
        detector.reset();
        return;
      }

      // 유속(g/s) = 무게 감소량 ÷ 측정 주기
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

      // 주입 완료 판정 (종료 무게 설정 시)
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
        return;
      }

      // 측정 결과 출력 (윈도우 완성 직후는 결과, 그 외에는 수집 진행률)
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

    // ── 주입 완료 후 수액 제거 감지 ───────────────────────────────
    if (ivPhase == PHASE_DONE && iv.currentWeight < WEIGHT_REMOVE_G) {
      Serial.println("[AUTO] 빈 수액 제거 감지 — 자동 재영점 후 대기 중.");
      iv      = IVState{};
      ivPhase = PHASE_TARE;
      detector.reset();
    }
  }

  // ── MQTT 상태 발행 (5초 주기, 모니터링 중에만) ───────────────────
  if (ivPhase == PHASE_MONITOR && now - lastStatusMs >= STATUS_MS) {
    lastStatusMs = now;
    publishStatus();
  }
}
