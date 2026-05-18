#pragma once
/*
 * BLE WiFi 프로비저닝 — 커스텀 GATT 서버
 *
 * Flutter 앱(flutter_blue_plus 등)에서 직접 통신 가능한 BLE 서비스.
 * WiFiProv 라이브러리를 사용하지 않고 표준 BLE GATT 만 사용하므로
 * 어떤 BLE 라이브러리든 호환됨.
 *
 * ── BLE 프로토콜 ────────────────────────────────────────────────────
 *
 *  Service UUID : 4fafc201-1fb5-459e-8fcc-c5c9c331914b
 *
 *  1) WiFi 스캔 (Read/Write)
 *     UUID : a3c87500-8ed3-4bdf-8a39-a01bebede295
 *     - Write 아무 1바이트  → 백그라운드 WiFi 스캔 시작
 *     - Read              → JSON 배열 반환 (스캔 완료 후)
 *       [{"ssid":"NetworkA","rssi":-45,"enc":3},
 *        {"ssid":"NetworkB","rssi":-67,"enc":0}, ...]
 *       enc: 0=open, 2=WPA, 3=WPA2, 4=WPA2_ENT, 8=WPA3
 *
 *  2) WiFi 자격증명 (Write)
 *     UUID : a3c87501-8ed3-4bdf-8a39-a01bebede295
 *     - Write JSON : {"ssid":"<SSID>","pw":"<PASSWORD>"}
 *     - ESP32 가 즉시 연결 시도 + 성공 시 NVS 저장
 *
 *  3) 연결 상태 (Read/Notify)
 *     UUID : a3c87502-8ed3-4bdf-8a39-a01bebede295
 *     - Notify JSON : {"state":"idle|scanning|connecting|connected|failed",
 *                      "ssid":"<현재 SSID 또는 빈 문자열>",
 *                      "ip":"<IP 또는 빈 문자열>",
 *                      "deviceId":"IVPOLE_XXXXXX"}
 *     - 상태 변경 시 자동으로 notify 발송
 *
 * ── 동작 흐름 ────────────────────────────────────────────────────────
 *
 *   부팅 → NVS에 자격증명 있으면 자동 연결 시도
 *        → 실패 또는 자격증명 없으면 BLE 광고 시작
 *        → 앱 연결 → 스캔 → SSID 선택 → 비번 입력 → 연결
 *        → 연결 성공 시 NVS 저장 → 다음 부팅부터 자동 연결
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ── UUID 정의 ─────────────────────────────────────────────────────
#define BLEPROV_SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLEPROV_CHAR_SCAN_UUID     "a3c87500-8ed3-4bdf-8a39-a01bebede295"
#define BLEPROV_CHAR_CRED_UUID     "a3c87501-8ed3-4bdf-8a39-a01bebede295"
#define BLEPROV_CHAR_STATUS_UUID   "a3c87502-8ed3-4bdf-8a39-a01bebede295"

// ── NVS 네임스페이스 ──────────────────────────────────────────────
#define BLEPROV_NVS_NAMESPACE  "ble_wifi"

class BLEWiFiProv {
public:
  enum State {
    IDLE,        // 대기 상태
    SCANNING,    // WiFi 스캔 중
    CONNECTING,  // WiFi 연결 시도 중
    CONNECTED,   // WiFi 연결 완료
    FAILED       // 연결 실패
  };

  // 초기화 — BLE GATT 서버 시작, 광고 시작
  // deviceName : BLE 광고에 사용할 이름 (예: "IVPOLE_AABBCC")
  void begin(const char *deviceName) {
    strncpy(_deviceName, deviceName, sizeof(_deviceName) - 1);
    _state = IDLE;
    _scanJson[0] = '\0';

    Serial.printf("[BLEProv] BLE 초기화 중... 기기명: %s\n", _deviceName);
    BLEDevice::init(_deviceName);

    // ★ MTU 늘리기 — 기본 23바이트로는 WiFi 목록 JSON이 잘림
    //   517 요청 시 클라이언트가 MTU 협상하면 한 번에 큰 패킷 전송 가능
    BLEDevice::setMTU(517);

    // GATT 서버 생성
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks(this));

    BLEService *service = server->createService(BLEPROV_SERVICE_UUID);

    // 1) Scan characteristic
    _charScan = service->createCharacteristic(
      BLEPROV_CHAR_SCAN_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    _charScan->setCallbacks(new ScanCallbacks(this));
    _charScan->setValue("[]");

    // 2) Credentials characteristic
    _charCred = service->createCharacteristic(
      BLEPROV_CHAR_CRED_UUID,
      BLECharacteristic::PROPERTY_WRITE
    );
    _charCred->setCallbacks(new CredCallbacks(this));

    // 3) Status characteristic (Notify)
    _charStatus = service->createCharacteristic(
      BLEPROV_CHAR_STATUS_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    _charStatus->addDescriptor(new BLE2902());
    updateStatusValue();

    service->start();

    // ── 광고 패킷 구성 ──────────────────────────────────────────
    // 폰(특히 안드로이드)에서 잘 잡히게 하려면:
    //  1) 광고 패킷에 이름을 명시적으로 포함 (scan response 의존 금지)
    //  2) 광고 인터벌을 짧게 (빠른 발견)
    //  3) BLE_GAP_CONN_MODE_UND (일반 발견 가능) + Connectable 모드
    BLEAdvertising *adv = BLEDevice::getAdvertising();

    // 광고 데이터: 이름 + Flags + 서비스 UUID
    BLEAdvertisementData advData;
    advData.setFlags(0x06);   // LE General Discoverable + BR/EDR Not Supported
    advData.setName(_deviceName);
    advData.setCompleteServices(BLEUUID(BLEPROV_SERVICE_UUID));
    adv->setAdvertisementData(advData);

    // 스캔 응답에도 이름 포함 (active scan 호환성)
    BLEAdvertisementData scanRsp;
    scanRsp.setName(_deviceName);
    adv->setScanResponseData(scanRsp);

    adv->setScanResponse(true);

    // 광고 인터벌: 짧게 → 폰이 빨리 발견 (단위 0.625ms)
    //   100ms ~ 150ms 권장 (전력 vs 발견 속도 균형)
    adv->setMinInterval(0xA0);   // 160 * 0.625 = 100ms
    adv->setMaxInterval(0xF0);   // 240 * 0.625 = 150ms

    BLEDevice::startAdvertising();

    Serial.printf("[BLEProv] BLE 광고 시작 (인터벌 100~150ms)\n");
    Serial.printf("[BLEProv] 광고 이름: %s\n", _deviceName);
    Serial.println("[BLEProv] ※ 안드로이드: 위치 서비스 ON + 앱 위치권한 허용 필요");
  }

  // loop() 에서 주기적으로 호출 — WiFi 연결 상태 변화 감지
  void loop() {
    // 비동기 WiFi 스캔 완료 체크
    if (_scanInProgress) {
      int16_t n = WiFi.scanComplete();
      if (n >= 0) {
        buildScanJson(n);
        size_t len = strlen(_scanJson);
        _charScan->setValue((uint8_t*)_scanJson, len);
        WiFi.scanDelete();
        _scanInProgress = false;
        setState(IDLE);
        Serial.printf("[BLEProv] WiFi 스캔 완료: %d개, JSON %u bytes\n", n, (unsigned)len);
        Serial.printf("[BLEProv] JSON 내용: %s\n", _scanJson);
      }
    }

    // 자격증명 받았으면 연결 시도
    if (_credPending) {
      _credPending = false;
      attemptConnect();
    }
  }

  // 저장된 자격증명으로 연결 시도 (부팅 시 호출)
  // 성공 true / 실패 false 반환
  bool tryStoredCredentials(uint32_t timeoutMs = 10000) {
    Preferences prefs;
    if (!prefs.begin(BLEPROV_NVS_NAMESPACE, true)) return false;

    String ssid = prefs.getString("ssid", "");
    String pw   = prefs.getString("pw",   "");
    prefs.end();

    if (ssid.length() == 0) {
      Serial.println("[BLEProv] 저장된 자격증명 없음.");
      return false;
    }

    Serial.printf("[BLEProv] 저장된 자격증명으로 연결: %s\n", ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pw.c_str());

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
      delay(250); Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[BLEProv] 연결 성공. IP: %s\n", WiFi.localIP().toString().c_str());
      _state = CONNECTED;
      return true;
    }
    Serial.println("[BLEProv] 저장된 자격증명 연결 실패.");
    WiFi.disconnect();
    return false;
  }

  // 저장된 자격증명 삭제 (재프로비저닝 강제)
  void clearStoredCredentials() {
    Preferences prefs;
    if (prefs.begin(BLEPROV_NVS_NAMESPACE, false)) {
      prefs.clear();
      prefs.end();
      Serial.println("[BLEProv] 저장된 자격증명 삭제.");
    }
  }

  State getState() const { return _state; }

private:
  char _deviceName[24];
  State _state = IDLE;

  BLECharacteristic *_charScan   = nullptr;
  BLECharacteristic *_charCred   = nullptr;
  BLECharacteristic *_charStatus = nullptr;

  bool _scanInProgress = false;
  bool _credPending    = false;
  String _pendingSsid;
  String _pendingPw;

  char _scanJson[1024];

  // ── 상태 변경 + Notify ──────────────────────────────────────────
  void setState(State newState) {
    _state = newState;
    updateStatusValue();
    if (_charStatus) _charStatus->notify();
  }

  void updateStatusValue() {
    StaticJsonDocument<256> doc;
    const char *stateStr[] = { "idle","scanning","connecting","connected","failed" };
    doc["state"]    = stateStr[_state];
    doc["ssid"]     = (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : String("");
    doc["ip"]       = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("");
    doc["deviceId"] = _deviceName;
    char buf[256];
    size_t n = serializeJson(doc, buf);
    if (_charStatus) _charStatus->setValue((uint8_t*)buf, n);
  }

  // ── WiFi 스캔 시작 ──────────────────────────────────────────────
  void startScan() {
    if (_scanInProgress) return;
    setState(SCANNING);
    WiFi.mode(WIFI_STA);
    WiFi.scanNetworks(true, false);   // 비동기, hidden 제외
    _scanInProgress = true;
    Serial.println("[BLEProv] WiFi 스캔 시작...");
  }

  // 스캔 결과를 JSON으로 변환
  void buildScanJson(int16_t n) {
    StaticJsonDocument<1024> doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < n && i < 20; i++) {
      JsonObject o = arr.createNestedObject();
      o["ssid"] = WiFi.SSID(i);
      o["rssi"] = WiFi.RSSI(i);
      o["enc"]  = (int)WiFi.encryptionType(i);
    }
    serializeJson(doc, _scanJson, sizeof(_scanJson));
  }

  // ── 받은 자격증명으로 WiFi 연결 ────────────────────────────────
  void attemptConnect() {
    if (_pendingSsid.length() == 0) return;
    Serial.printf("[BLEProv] WiFi 연결 시도: %s\n", _pendingSsid.c_str());
    setState(CONNECTING);

    WiFi.mode(WIFI_STA);
    WiFi.begin(_pendingSsid.c_str(), _pendingPw.c_str());

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(250);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[BLEProv] 연결 성공. IP: %s\n", WiFi.localIP().toString().c_str());
      // NVS 저장
      Preferences prefs;
      if (prefs.begin(BLEPROV_NVS_NAMESPACE, false)) {
        prefs.putString("ssid", _pendingSsid);
        prefs.putString("pw",   _pendingPw);
        prefs.end();
        Serial.println("[BLEProv] 자격증명 NVS 저장 완료.");
      }
      setState(CONNECTED);
    } else {
      Serial.println("[BLEProv] 연결 실패.");
      setState(FAILED);
    }
  }

  // ── BLE 콜백 클래스들 ──────────────────────────────────────────
  class ServerCallbacks : public BLEServerCallbacks {
  public:
    ServerCallbacks(BLEWiFiProv *p) : _p(p) {}
    void onConnect(BLEServer *server) override {
      Serial.println("[BLEProv] 앱 연결됨.");
    }
    void onDisconnect(BLEServer *server) override {
      Serial.println("[BLEProv] 앱 연결 해제 — 광고 재시작.");
      BLEDevice::startAdvertising();
    }
  private:
    BLEWiFiProv *_p;
  };

  class ScanCallbacks : public BLECharacteristicCallbacks {
  public:
    ScanCallbacks(BLEWiFiProv *p) : _p(p) {}
    void onWrite(BLECharacteristic *c) override {
      Serial.println("[BLEProv] WiFi 스캔 요청 수신 (write)");
      _p->startScan();
    }
    void onRead(BLECharacteristic *c) override {
      std::string v = c->getValue();
      Serial.printf("[BLEProv] WiFi 목록 read 요청, 응답 %u bytes\n",
                    (unsigned)v.length());
    }
  private:
    BLEWiFiProv *_p;
  };

  class CredCallbacks : public BLECharacteristicCallbacks {
  public:
    CredCallbacks(BLEWiFiProv *p) : _p(p) {}
    void onWrite(BLECharacteristic *c) override {
      String val = c->getValue().c_str();
      Serial.printf("[BLEProv] 자격증명 수신: %s\n", val.c_str());

      StaticJsonDocument<256> doc;
      if (deserializeJson(doc, val)) {
        Serial.println("[BLEProv] JSON 파싱 실패.");
        return;
      }
      _p->_pendingSsid = doc["ssid"].as<String>();
      _p->_pendingPw   = doc["pw"].as<String>();
      _p->_credPending = true;
    }
  private:
    BLEWiFiProv *_p;
  };
};
