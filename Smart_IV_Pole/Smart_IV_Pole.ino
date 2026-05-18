

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiProv.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "esp_system.h"
#include "esp_efuse.h"    // esp_efuse_mac_get_default()
#include "ads1232.h"
#include "cnn_detector.h"

// ── 펌웨어 버전 ───────────────────────────────────────────────────
#define FW_VERSION  "1.1.0"

// ── 이미지 로그 파일 경로 ─────────────────────────────────────────
#define IMAGE_LOG_PATH  "/imglog.csv"

// ── 핀 정의 ──────────────────────────────────────────────────────
#define ADS_DOUT  19
#define ADS_SCLK  18
#define ADS_PDWN  23
#define ADS_GAIN0 33
#define ADS_GAIN1 32

// ── MQTT 설정 ─────────────────────────────────────────────────────
#define MQTT_BROKER   "192.168.1.100"   // TODO: 실제 브로커 IP/도메인으로 변경
#define MQTT_PORT     1883
// #define MQTT_USER  "user"            // 브로커 인증 필요 시 주석 해제
// #define MQTT_PASS  "pass"

// ── 타이밍 설정 ───────────────────────────────────────────────────
#define WEIGHT_MS         1000     // 무게 측정 주기 (1초)
#define STATUS_MS         5000     // MQTT 상태 발행 주기 (5초)
#define MQTT_RETRY_MS     5000     // MQTT 재연결 시도 주기
#define BOOT_DELAY_MS     2000UL   // 전원 ON 후 영점까지 대기

// ── 드립 팩터 교정 ─────────────────────────────────────────────────
#define CALIB_DURATION_MS  60000UL   // 교정 측정 시간 (60초)

// ── 수액 자동 감지 임계값 ─────────────────────────────────────────
#define WEIGHT_HANG_G    50.0f   // 수액 감지: 이 이상이면 수액 걸린 것으로 판단
#define WEIGHT_REMOVE_G  15.0f   // 수액 제거: 이 미만이면 수액 없는 것으로 판단

// ── 기본 목표 유속 ────────────────────────────────────────────────
// 앱에서 변경 전까지 사용하는 기본값 (성인용 20gtt/mL 세트)
#define DEFAULT_TARGET_GTT  60.0f   // gtt/min

// ── 로드셀 교정 계수 ─────────────────────────────────────────────
#define CALIB_FACTOR_DEFAULT  1642.8623f

// ── 드립 팩터 기본값 (교정 실패 시 백업) ─────────────────────────
#define DRIP_FACTOR_DEFAULT  0.0500f

// ── EMA 안정화 샘플 수 ────────────────────────────────────────────
#define WARMUP_SAMPLES  8

// =================================================================
// 기기 고유 ID — MAC 주소 뒤 6자리 기반 ("IVPOLE_AABBCC")
// QR 코드 라벨에 인쇄 → 앱이 스캔하여 백엔드에서 기기 특정
// =================================================================
char deviceId[16];     // "IVPOLE_AABBCC"
char bleName[20];      // BLE 검색명 = deviceId (기기마다 다름)

// ── MQTT 토픽 (setup()에서 deviceId로 동적 생성) ──────────────────
char T_STATUS[48];   // iv_pole/<ID>/status
char T_ALERT[48];    // iv_pole/<ID>/alert
char T_CONFIG[48];   // iv_pole/<ID>/config
char T_CMD[48];      // iv_pole/<ID>/cmd
char T_INFO[48];     // iv_pole/<ID>/info  (온·오프라인 retained)

// =================================================================
// 부팅 절차 단계 정의
// =================================================================
enum IVPhase {
  PHASE_BOOT,     // 전원 ON — 2초 대기 중
  PHASE_TARE,     // 자동 영점 조정 중
  PHASE_WAIT,     // 수액 감지 대기
  PHASE_CALIB,    // 드립 팩터 교정 중 (60초)
  PHASE_WARMUP,   // EMA 안정화 대기 (8샘플)
  PHASE_MONITOR,  // 주입 모니터링 중
  PHASE_DONE      // 주입 완료
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
  float targetFlowRate  = 0;
  float finishWeight    = 0;
  float currentWeight   = 0;
  float prevWeight      = 0;
  float currentFlowRate = 0;
  float dripFactor      = DRIP_FACTOR_DEFAULT;
  bool  started         = false;
  bool  complete        = false;
  int   warmup          = 0;
} iv;

// ─────────────────────────────────────────────────────────────────
// 부팅 절차 상태 변수
// ─────────────────────────────────────────────────────────────────
IVPhase       ivPhase          = PHASE_BOOT;
float         calibTargetGtt   = DEFAULT_TARGET_GTT;
float         calibWeightStart = 0;
unsigned long calibStartMs     = 0;
unsigned long bootMs           = 0;

// ─────────────────────────────────────────────────────────────────
// 타이밍 변수
// ─────────────────────────────────────────────────────────────────
unsigned long lastWeightMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastMqttMs   = 0;

// =================================================================
// 기기 ID 생성 — ESP32 칩 고유 MAC으로 식별
// =================================================================
void buildDeviceId() {
  // ESP32 eFuse MAC 6바이트 구조:
  //   byte[0~2] = OUI (Espressif 제조사 코드, 모든 기기 동일)
  //   byte[3~5] = 기기별 고유값  →  getEfuseMac() uint64_t 기준 bit[47:24]
  // ※ mac & 0xFFFFFF 는 OUI를 가져오므로 모든 기기가 같은 ID가 됨 → 잘못된 방법
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);   // mac[0]=OUI첫번째 ... mac[5]=기기고유마지막
  snprintf(deviceId, sizeof(deviceId), "IVPOLE_%02X%02X%02X", mac[3], mac[4], mac[5]);
  snprintf(bleName,  sizeof(bleName),  "%s", deviceId);
}

// MQTT 토픽 문자열 빌드 (deviceId 확정 후 setup()에서 1회 호출)
void buildTopics() {
  snprintf(T_STATUS, sizeof(T_STATUS), "iv_pole/%s/status", deviceId);
  snprintf(T_ALERT,  sizeof(T_ALERT),  "iv_pole/%s/alert",  deviceId);
  snprintf(T_CONFIG, sizeof(T_CONFIG), "iv_pole/%s/config", deviceId);
  snprintf(T_CMD,    sizeof(T_CMD),    "iv_pole/%s/cmd",    deviceId);
  snprintf(T_INFO,   sizeof(T_INFO),   "iv_pole/%s/info",   deviceId);
}

// ─────────────────────────────────────────────────────────────────
// BLE 프로비저닝 이벤트 콜백
// ─────────────────────────────────────────────────────────────────
void onProvEvent(arduino_event_t *ev) {
  switch (ev->event_id) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WiFi] 연결됨. IP: %s\n", WiFi.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WiFi] 연결 끊김.");
      break;
    case ARDUINO_EVENT_PROV_START:
      Serial.printf("[Prov] BLE 대기 중.  기기명: %s\n", bleName);
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
      Serial.println("[Prov] WiFi 자격증명 수신.");
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("[Prov] WiFi 인증 완료.");
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
  StaticJsonDocument<160> doc;
  doc["type"]         = type;
  doc["flowMeasured"] = measuredFlowRate;
  doc["flowTarget"]   = iv.targetFlowRate;
  doc["confidence"]   = detector.getWindowConfidence();
  doc["ts"]           = millis();
  char buf[160];
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

// 온라인 상태 발행 (MQTT 연결 시마다 호출)
// retained = true → 브로커에 저장, 백엔드·앱이 언제든 최신 상태 조회 가능
void publishOnline() {
  StaticJsonDocument<128> doc;
  doc["online"]   = true;
  doc["deviceId"] = deviceId;
  doc["fw"]       = FW_VERSION;
  doc["ip"]       = WiFi.localIP().toString();
  char buf[128];
  serializeJson(doc, buf);
  mqtt.publish(T_INFO, buf, /*retained=*/true);
  Serial.printf("[MQTT] 온라인 상태 발행 → %s\n", T_INFO);
}

// ─────────────────────────────────────────────────────────────────
// MQTT 연결 (LWT 포함)
// LWT(Last Will Testament): 기기가 비정상 종료되면
// 브로커가 자동으로 T_INFO에 {"online":false} 를 발행해줌
// ─────────────────────────────────────────────────────────────────
bool connectMQTT() {
  if (mqtt.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false;

  Serial.printf("[MQTT] 연결 중 %s:%d ...\n", MQTT_BROKER, MQTT_PORT);

  // LWT 메시지 사전 등록
  StaticJsonDocument<64> lwtDoc;
  lwtDoc["online"]   = false;
  lwtDoc["deviceId"] = deviceId;
  char lwtBuf[64];
  serializeJson(lwtDoc, lwtBuf);

  bool ok;
#ifdef MQTT_USER
  ok = mqtt.connect(deviceId, MQTT_USER, MQTT_PASS,
                    T_INFO, /*qos*/0, /*retain*/true, lwtBuf);
#else
  ok = mqtt.connect(deviceId,
                    NULL, NULL,
                    T_INFO, /*qos*/0, /*retain*/true, lwtBuf);
#endif

  if (ok) {
    mqtt.subscribe(T_CONFIG);
    mqtt.subscribe(T_CMD);
    publishOnline();   // 온라인 상태 즉시 발행
    Serial.println("[MQTT] 연결 완료.");
    return true;
  }
  Serial.printf("[MQTT] 실패, state=%d\n", mqtt.state());
  return false;
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
    // 백엔드 → 앱 → MQTT: { "targetFlowRate": 60, "finishWeight": 50 }
    if (doc.containsKey("targetFlowRate")) {
      calibTargetGtt = doc["targetFlowRate"].as<float>();
      Serial.printf("[CFG] 목표 유속: %.0f gtt/min\n", calibTargetGtt);

      // 모니터링 중 변경 시 즉시 반영
      if (ivPhase == PHASE_MONITOR || ivPhase == PHASE_WARMUP) {
        iv.targetFlowRate = (calibTargetGtt / 60.0f) * iv.dripFactor;
        detector.reset();
        Serial.printf("[CFG] 즉시 반영: %.4f g/s\n", iv.targetFlowRate);
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
      Serial.println("[CMD] 영점 조정 중...");
      loadCell.tare(20);
      Serial.println("[CMD] 완료.");
    } else if (cmd == "reset") {
      iv      = IVState{};
      ivPhase = PHASE_TARE;
      detector.reset();
      Serial.println("[CMD] 초기화 완료 — 재영점 시작.");
    }
  }
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
// 시리얼 디버그 명령 (엔지니어용, 실무 사용 불필요)
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
    }
  } else if (line.startsWith("finish ")) {
    iv.finishWeight = line.substring(7).toFloat();
    Serial.printf("[DBG] 종료 무게: %.2f g\n", iv.finishWeight);
  } else if (line == "tare") {
    loadCell.tare(20);
    Serial.println("[DBG] 영점 완료.");
  } else if (line == "reset") {
    iv = IVState{};  ivPhase = PHASE_TARE;  detector.reset();
    Serial.println("[DBG] 초기화 → 재영점.");
  } else if (line == "status") {
    const char *ph[] = { "부팅대기","영점조정","수액대기","드립팩터교정","안정화","모니터링","완료" };
    Serial.printf("[STATUS] 단계:%s  W:%.2fg  유속:%.4f/%.4fg/s  dripF:%.5f\n"
                  "         결과:%s  신뢰도:%.0f%%  CNN:%s  샘플:%d/%d\n",
                  ph[ivPhase], iv.currentWeight,
                  iv.currentFlowRate, iv.targetFlowRate, iv.dripFactor,
                  detector.getResultLabel(),
                  detector.getWindowConfidence() * 100.0f,
                  detector.isTFLiteActive() ? "ON" : "fallback",
                  detector.getSampleCount(), CNN_WIN);
  } else if (line.startsWith("tolerance ")) {
    float t = line.substring(10).toFloat();
    if (t > 0 && t < 1.0f) { detector.setTolerance(t); Serial.printf("[DBG] tolerance=±%.0f%%\n", t*100); }
  } else if (line.startsWith("alpha ")) {
    float a = constrain(line.substring(6).toFloat(), 0.05f, 0.5f);
    loadCell.setEmaAlpha(a);  Serial.printf("[DBG] EMA alpha=%.2f\n", a);
  } else if (line == "image")   { printImage();   }
  else if (line == "dumplog")   { dumpImageLog(); }
  else if (line == "clearlog")  { clearImageLog();}
  else {
    Serial.println("[DBG] 명령: target/finish/tare/reset/status/tolerance/alpha/image/dumplog/clearlog");
  }
}

// ─────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Smart IV Pole ===");

  // ── 기기 ID 생성 및 MQTT 토픽 구성 ─────────────────────────────
  buildDeviceId();
  buildTopics();

  // ── 부팅 시 기기 정보 출력 (QR 라벨 제작용) ─────────────────────
  Serial.println("╔══════════════════════════════════════╗");
  Serial.printf( "║  기기 ID : %-26s║\n", deviceId);
  Serial.printf( "║  BLE 이름: %-26s║\n", bleName);
  Serial.printf( "║  FW 버전 : %-26s║\n", FW_VERSION);
  Serial.printf( "║  QR 내용 : iv-pole://connect/%s  ║\n", deviceId);
  Serial.println("╚══════════════════════════════════════╝");

  // ── LittleFS 마운트 ──────────────────────────────────────────────
  if (!LittleFS.begin(true))
    Serial.println("[FS] LittleFS 마운트 실패");
  else
    Serial.println("[FS] LittleFS OK");

  // ── 로드셀 초기화 ────────────────────────────────────────────────
  loadCell.begin(128);
  loadCell.setCalibFactor(CALIB_FACTOR_DEFAULT);
  Serial.printf("[ADS] CalibFactor=%.2f\n", loadCell.getCalibFactor());

  // ── BLE WiFi 프로비저닝 ──────────────────────────────────────────
  // NVS에 저장된 WiFi 자격증명 있으면 자동 연결,
  // 없으면 앱에서 BLE로 SSID/PW 전송 필요
  WiFi.onEvent(onProvEvent);
  WiFiProv.beginProvision(
    NETWORK_PROV_SCHEME_BLE,
    NETWORK_PROV_SCHEME_HANDLER_FREE_BLE,
    NETWORK_PROV_SECURITY_1,
    "ivpole01",    // PoP (Proof of Possession) — 앱 페어링 시 요구
    bleName        // BLE 기기명 = 기기 ID (앱이 QR 스캔 후 자동 매칭)
  );
  Serial.println("[WiFi] 프로비저닝 대기 중...");
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED)
    Serial.println("[WiFi] 미연결 — 오프라인 모드 (MQTT 비활성화).");

  // ── CNN 탐지기 초기화 ────────────────────────────────────────────
  if (!detector.begin())
    Serial.println("[CNN] Fallback 모드.");

  // ── MQTT 설정 ────────────────────────────────────────────────────
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMqttMsg);
  connectMQTT();

  // ── 부팅 타이머 시작 → 2초 후 자동 영점 ──────────────────────────
  bootMs = millis();
  Serial.printf("[SYS] %.1f초 후 자동 영점 조정 시작.\n", BOOT_DELAY_MS / 1000.0f);
  Serial.printf("[SYS] 기본 목표 유속: %.0f gtt/min (앱에서 변경 가능)\n", calibTargetGtt);

  lastWeightMs = lastStatusMs = millis();
}

// ─────────────────────────────────────────────────────────────────
// loop
// ─────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  handleSerial();

  // WiFi 재연결 감시
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
    if (now - bootMs >= BOOT_DELAY_MS) ivPhase = PHASE_TARE;
    return;
  }

  // ── 2단계: 자동 영점 조정 ─────────────────────────────────────────
  if (ivPhase == PHASE_TARE) {
    Serial.println("[AUTO] 영점 조정 중...");
    loadCell.tare(20);
    iv = IVState{};
    detector.reset();
    ivPhase = PHASE_WAIT;
    Serial.println("[AUTO] 영점 완료. 수액팩을 걸어주세요.");
    lastWeightMs = millis();
    return;
  }

  // ── 3단계: 수액 감지 대기 ─────────────────────────────────────────
  if (ivPhase == PHASE_WAIT && now - lastWeightMs >= WEIGHT_MS) {
    lastWeightMs = now;
    float w = loadCell.readWeight();
    iv.currentWeight = w;

    if (w > WEIGHT_HANG_G) {
      Serial.printf("[AUTO] 수액 감지 (%.1fg) — 드립 팩터 교정 시작\n", w);
      Serial.printf("[AUTO] 목표: %.0f gtt/min | 교정 시간: %lu초\n",
                    calibTargetGtt, CALIB_DURATION_MS / 1000UL);
      calibWeightStart = loadCell.stableRead(10);
      calibStartMs     = millis();
      ivPhase          = PHASE_CALIB;
    }
    return;
  }

  // ── 4단계: 드립 팩터 교정 (60초) ──────────────────────────────────
  if (ivPhase == PHASE_CALIB && now - calibStartMs >= CALIB_DURATION_MS) {
    float weightEnd  = loadCell.stableRead(10);
    float weightLost = calibWeightStart - weightEnd;

    if (weightLost > 0.05f && calibTargetGtt > 0) {
      iv.dripFactor = weightLost / calibTargetGtt;
      Serial.printf("[AUTO] 교정 완료: %.5f g/gtt  (감소 %.3fg / %.0f방울)\n",
                    iv.dripFactor, weightLost, calibTargetGtt);
    } else {
      Serial.printf("[AUTO] 교정 실패 — 기본값 %.5f g/gtt 사용\n", iv.dripFactor);
    }

    iv.targetFlowRate = (calibTargetGtt / 60.0f) * iv.dripFactor;
    Serial.printf("[AUTO] 목표 유속 확정: %.4f g/s (%.0f gtt/min)\n",
                  iv.targetFlowRate, calibTargetGtt);

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
  // =================================================================
  if (ivPhase != PHASE_WAIT && now - lastWeightMs >= WEIGHT_MS) {
    lastWeightMs = now;

    iv.prevWeight    = iv.currentWeight;
    iv.currentWeight = loadCell.readWeight();

    // ── EMA 안정화 단계 ────────────────────────────────────────────
    if (ivPhase == PHASE_WARMUP) {
      iv.prevWeight = iv.currentWeight;   // 유속 계산 없이 EMA 수렴 대기
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
        Serial.println("[AUTO] 수액 제거 감지 — 재영점 후 재시작.");
        iv      = IVState{};
        ivPhase = PHASE_TARE;
        detector.reset();
        return;
      }

      // 유속 계산: g/s = 무게 감소량 ÷ 측정 주기
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
        return;
      }

      // 측정 결과 출력
      if (detector.getSampleCount() == 0 && detector.isWindowFull()) {
        Serial.printf("[IV] W:%.2fg  유속:%.4f/%.4fg/s  → %s (신뢰도:%.0f%%)\n",
                      iv.currentWeight, iv.currentFlowRate, iv.targetFlowRate,
                      detector.getResultLabel(),
                      detector.getWindowConfidence() * 100.0f);
      } else {
        Serial.printf("[IV] W:%.2fg  유속:%.4f/%.4fg/s  [수집중 %d/%d]\n",
                      iv.currentWeight, iv.currentFlowRate, iv.targetFlowRate,
                      detector.getSampleCount(), CNN_WIN);
      }
    }

    // ── 주입 완료 후 수액 제거 감지 ───────────────────────────────
    if (ivPhase == PHASE_DONE && iv.currentWeight < WEIGHT_REMOVE_G) {
      Serial.println("[AUTO] 빈 수액 제거 — 재영점 후 대기 중.");
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
