/*
 * BLE 진단용 최소 스케치
 *
 * 목적: ESP32 WROOM-32E 의 BLE 자체가 동작하는지 확인.
 *       WiFiProv 라이브러리 영향 없이 순수 BLE 광고만 테스트.
 *
 * 사용법:
 *   1. Arduino IDE → 보드: "ESP32 Dev Module"
 *   2. Tools → Partition Scheme: "Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)"
 *      또는 "Huge APP (3MB No OTA/1MB SPIFFS)"
 *   3. 이 스케치 업로드 후 시리얼 모니터(115200) 확인.
 *   4. 휴대폰 BLE 스캐너(nRF Connect, LightBlue 등)에서 "IVPOLE_TEST" 검색.
 *
 * 결과 해석:
 *   - 시리얼에 "BLE 광고 시작!" 출력 + 폰에서 발견됨
 *       → BLE 정상. 문제는 WiFiProv 쪽.
 *   - 시리얼 출력은 정상이지만 폰에서 안 보임
 *       → BLE 광고는 시작됐지만 광고 파라미터 문제 / 폰 BLE 문제.
 *   - 시리얼에 에러 또는 멈춤
 *       → 파티션 스킴 / BLE 라이브러리 미포함 문제.
 *   - BLE_NAME 길이 문제 가능성 확인.
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>

#define SERVICE_UUID         "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void setup() {
  Serial.begin(115200);
  delay(1500);   // 시리얼 모니터 안정화

  Serial.println("\n\n=== BLE 진단 테스트 ===");

  // ── MAC 주소 확인 (eFuse 직접 읽기, WiFi 초기화 무관) ──────────
  uint64_t chipid = ESP.getEfuseMac();
  uint8_t  mac[6];
  for (int i = 0; i < 6; i++) mac[i] = (chipid >> (8 * i)) & 0xFF;
  char devName[24];
  snprintf(devName, sizeof(devName), "IVPOLE_%02X%02X%02X", mac[3], mac[4], mac[5]);

  Serial.printf("[CHIP] MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("[CHIP] 모델: %s\n",         ESP.getChipModel());
  Serial.printf("[CHIP] 리비전: %d\n",       ESP.getChipRevision());
  Serial.printf("[CHIP] 코어: %d개\n",       ESP.getChipCores());
  Serial.printf("[CHIP] Flash: %u MB\n",     ESP.getFlashChipSize() / (1024*1024));
  Serial.printf("[CHIP] Free heap: %u\n",    ESP.getFreeHeap());

  // ── BLE 초기화 ─────────────────────────────────────────────────
  Serial.println("\n[BLE] 1/4 BLEDevice::init() 호출 중...");
  BLEDevice::init(devName);
  Serial.printf("[BLE]      OK. 기기명: %s (길이: %d)\n", devName, strlen(devName));

  Serial.println("[BLE] 2/4 BLE 서버 생성 중...");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Hello from ESP32");
  pService->start();
  Serial.println("[BLE]      OK.");

  Serial.println("[BLE] 3/4 광고 설정 중...");
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);   // iPhone 호환성
  pAdvertising->setMinPreferred(0x12);
  Serial.println("[BLE]      OK.");

  Serial.println("[BLE] 4/4 광고 시작!");
  BLEDevice::startAdvertising();

  Serial.println();
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.printf( "║  BLE 광고 중 — 기기명: %-18s║\n", devName);
  Serial.println("║  휴대폰 BLE 스캐너로 검색해보세요.       ║");
  Serial.println("║  - 안드로이드: nRF Connect              ║");
  Serial.println("║  - 아이폰    : LightBlue                ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.printf("[MEM] Free heap 광고 후: %u bytes\n", ESP.getFreeHeap());
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last >= 3000) {
    last = millis();
    Serial.printf("[BLE] 광고 중... (Free heap: %u)\n", ESP.getFreeHeap());
  }
  delay(50);
}
