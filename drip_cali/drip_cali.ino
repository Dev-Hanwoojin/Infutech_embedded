// ─────────────────────────────────────────────────────────────────
//  드립 팩터 측정 스케치  (drip_cali/drip_cali.ino)
//
//  목적: IV 세트별로 실제 1방울(gtt)의 무게(g)를 실측 교정
//         → 메인 스케치 Smart_IV_Pole.ino 의 DRIP_FACTOR_DEFAULT 에 복사
//
//  측정 원리:
//    dripFactor (g/gtt) = 무게 감소량(g) / 실제 방울 수(gtt)
//    실제 방울 수 = 설정 유속(gtt/min) × 경과 시간(min)
//
//  사용 순서:
//    1. tare            — 빈 상태 영점 (수액팩 + 폴대 포함)
//    2. start <gtt/min> — IV 컨트롤러에서 설정한 유속 입력 후 수액 흐름 시작
//    3. (일정 시간 경과 — 최소 2분 이상 권장)
//    4. stop            — 측정 종료, dripFactor 자동 계산 출력
//    5. 출력된 dripFactor 값을 Smart_IV_Pole.ino 의 DRIP_FACTOR_DEFAULT 에 복사
//
//  빠른 방법 (자동 타이머):
//    measure <gtt/min> <초> — 지정 시간 동안 자동 측정 후 결과 출력
//
//  커맨드 목록:
//    tare                  — 영점 조정
//    start <gtt/min>       — 수동 측정 시작
//    stop                  — 수동 측정 종료 + 결과
//    measure <gtt/min> <s> — 자동 측정 (예: measure 60 120)
//    status                — 현재 측정 진행 상황
//    alpha <0.05~0.5>      — EMA 부드러움 조절
//    raw                   — raw ADC 출력 토글
//    help                  — 커맨드 목록
//
//  핀 설정: Smart_IV_Pole 과 동일
//    DOUT=19  SCLK=18  PDWN=23  GAIN0=33  GAIN1=32
// ─────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include "../Smart_IV_Pole/ads1232.h"

#define ADS_DOUT  19
#define ADS_SCLK  18
#define ADS_PDWN  23
#define ADS_GAIN0 33
#define ADS_GAIN1 32

// ── 캘리브레이션 팩터 (Smart_IV_Pole.ino 와 동일한 값) ────────────
#define CALIB_FACTOR_DEFAULT  1642.8623f

#define PRINT_INTERVAL_MS  1000   // 실시간 출력 주기
#define MIN_MEASURE_SEC    30     // 최소 측정 시간 (정확도 보장)
#define STABLE_SAMPLES     10     // 시작/종료 무게 안정화 샘플 수

ADS1232 scale(ADS_DOUT, ADS_SCLK, ADS_PDWN, ADS_GAIN0, ADS_GAIN1);

// ── EMA 우회, raw 다중 평균으로 안정적인 무게 측정 ─────────────────
// EMA는 alpha=0.15로 느리게 추종 → 시작/종료 스냅샷에 부적합
// raw 여러 개 평균을 사용해 드리프트/진동 노이즈 억제
float stableWeight(int n = STABLE_SAMPLES) {
  Serial.printf("  [무게 측정중 %d샘플...", n);
  long sum = 0;
  for (int i = 0; i < n; i++) {
    sum += scale.readRaw();
    delay(80);
  }
  float w = (float)(sum / n - scale.getTareOffset()) / scale.getCalibFactor();
  Serial.printf(" %.4f g]\n", w);
  return w;
}

// ── 측정 상태 ────────────────────────────────────────────────────
struct MeasureState {
  bool     running    = false;
  float    gttPerMin  = 0;      // 설정 유속 (gtt/min)
  float    startWeight= 0;      // 측정 시작 시 무게 (g)
  unsigned long startMs = 0;    // 측정 시작 시각 (ms)
} meas;

bool showRaw = false;
unsigned long lastPrint = 0;

// ── 결과 계산 및 출력 ─────────────────────────────────────────────
void printResult(float endWeight, unsigned long elapsedMs) {
  float weightDrop  = meas.startWeight - endWeight;
  float elapsedMin  = elapsedMs / 60000.0f;
  float totalDrops  = meas.gttPerMin * elapsedMin;
  float dripFactor  = (totalDrops > 0) ? (weightDrop / totalDrops) : 0;

  Serial.println("─────────────────────────────────────────");
  Serial.printf("[결과] 경과 시간  : %.1f 초 (%.2f 분)\n",
                elapsedMs / 1000.0f, elapsedMin);
  Serial.printf("[결과] 설정 유속  : %.1f gtt/min\n", meas.gttPerMin);
  Serial.printf("[결과] 총 방울 수 : %.1f gtt\n", totalDrops);
  Serial.printf("[결과] 무게 감소  : %.4f g\n", weightDrop);
  Serial.println("─────────────────────────────────────────");

  if (weightDrop <= 0) {
    Serial.println("[경고] 무게가 감소하지 않았습니다.");
    Serial.println("       수액이 실제로 흘렀는지 확인하세요.");
    return;
  }
  if (elapsedMs < (MIN_MEASURE_SEC * 1000UL)) {
    Serial.printf("[경고] 측정 시간이 %d초 미만입니다 — 정확도가 낮을 수 있습니다.\n",
                  MIN_MEASURE_SEC);
  }

  Serial.printf("[결과] ★ dripFactor = %.5f g/gtt\n", dripFactor);
  Serial.println("─────────────────────────────────────────");

  // 참고: 표준 IV 세트별 이론값
  float theoretical = 1000.0f / (meas.gttPerMin > 0 ? (meas.gttPerMin / (meas.gttPerMin)) : 1);
  Serial.println("[참고] 표준 IV 세트별 이론값:");
  Serial.println("       20 gtt/mL (성인 표준)    = 0.05000 g/gtt");
  Serial.println("       15 gtt/mL               = 0.06667 g/gtt");
  Serial.println("       10 gtt/mL               = 0.10000 g/gtt");
  Serial.println("       60 gtt/mL (소아 마이크로드립) = 0.01667 g/gtt");
  Serial.println("─────────────────────────────────────────");
  Serial.println("[적용] Smart_IV_Pole.ino 의 DRIP_FACTOR_DEFAULT 값을 위 결과로 교체:");
  Serial.printf("       #define DRIP_FACTOR_DEFAULT  %.5ff\n", dripFactor);
  Serial.println("─────────────────────────────────────────");
}

// ── 자동 측정 (블로킹) ────────────────────────────────────────────
void autoMeasure(float gttPerMin, int seconds) {
  Serial.println("─────────────────────────────────────────");
  Serial.printf("[AUTO] 유속=%.1f gtt/min  시간=%d초 자동 측정 시작\n",
                gttPerMin, seconds);
  Serial.println("[AUTO] 지금 수액 흐름을 시작하세요!");
  Serial.println("[AUTO] 3초 후 측정 시작...");
  for (int i = 3; i > 0; i--) {
    Serial.printf("[AUTO] %d...\n", i);
    delay(1000);
  }

  Serial.println("[AUTO] 시작 무게 측정 중...");
  float startW = stableWeight();
  unsigned long startMs = millis();
  Serial.printf("[AUTO] 시작 무게: %.4f g\n", startW);
  Serial.println("[AUTO] 측정 중... (시리얼 입력으로 중단 불가)");

  unsigned long targetMs = (unsigned long)seconds * 1000UL;
  unsigned long nextPrint = millis() + 5000;

  while (millis() - startMs < targetMs) {
    if (millis() > nextPrint) {
      float elapsed = (millis() - startMs) / 1000.0f;
      float remain  = seconds - elapsed;
      float curW    = scale.readWeight();
      Serial.printf("[AUTO] 경과: %.0f초  남은 시간: %.0f초  현재 무게: %.4f g  감소: %.4f g\n",
                    elapsed, remain, curW, startW - curW);
      nextPrint = millis() + 5000;
    }
    delay(100);
  }

  unsigned long elapsedMs = millis() - startMs;
  Serial.println("[AUTO] 종료 무게 측정 중...");
  float endW = stableWeight();

  meas.gttPerMin   = gttPerMin;
  meas.startWeight = startW;
  meas.startMs     = startMs;

  Serial.printf("[AUTO] 종료 무게: %.4f g\n", endW);
  printResult(endW, elapsedMs);
}

// ── 시리얼 커맨드 처리 ────────────────────────────────────────────
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // ── tare ──────────────────────────────────────────────────────
  if (line == "tare") {
    if (meas.running) {
      Serial.println("[ERR] 측정 중에는 tare 불가. stop 먼저 입력.");
      return;
    }
    Serial.println("[TARE] 영점 조정 중... (20샘플)");
    scale.tare(20);
    Serial.printf("[TARE] 완료. tare offset = %ld\n", scale.getTareOffset());

  // ── start <gtt/min> ───────────────────────────────────────────
  } else if (line.startsWith("start ")) {
    if (meas.running) {
      Serial.println("[ERR] 이미 측정 중입니다. stop 먼저 입력.");
      return;
    }
    float rate = line.substring(6).toFloat();
    if (rate <= 0) { Serial.println("[ERR] 유속 값이 잘못됨 (예: start 60)"); return; }

    meas.gttPerMin   = rate;
    Serial.println("[START] 시작 무게 측정 중...");
    meas.startWeight = stableWeight();
    meas.startMs     = millis();
    meas.running     = true;

    Serial.println("─────────────────────────────────────────");
    Serial.printf("[START] 유속=%.1f gtt/min  시작 무게=%.4f g\n",
                  meas.gttPerMin, meas.startWeight);
    Serial.println("[START] 수액 흐름을 시작하세요.");
    Serial.println("[START] 측정 종료 시 'stop' 입력");
    Serial.println("─────────────────────────────────────────");

  // ── stop ──────────────────────────────────────────────────────
  } else if (line == "stop") {
    if (!meas.running) { Serial.println("[ERR] 측정 중이 아닙니다."); return; }
    meas.running = false;
    unsigned long elapsedMs = millis() - meas.startMs;
    Serial.println("[STOP] 종료 무게 측정 중...");
    float endWeight = stableWeight();
    Serial.printf("[STOP] 종료 무게: %.4f g\n", endWeight);
    printResult(endWeight, elapsedMs);

  // ── measure <gtt/min> <초> ────────────────────────────────────
  } else if (line.startsWith("measure ")) {
    if (meas.running) {
      Serial.println("[ERR] 이미 측정 중입니다. stop 먼저 입력.");
      return;
    }
    // 파싱: "measure 60 120"
    String args = line.substring(8);
    args.trim();
    int spaceIdx = args.indexOf(' ');
    if (spaceIdx < 0) {
      Serial.println("[ERR] 사용법: measure <gtt/min> <초>  예) measure 60 120");
      return;
    }
    float rate  = args.substring(0, spaceIdx).toFloat();
    int   secs  = args.substring(spaceIdx + 1).toInt();
    if (rate <= 0 || secs <= 0) {
      Serial.println("[ERR] 값이 잘못됨. 예) measure 60 120");
      return;
    }
    autoMeasure(rate, secs);

  // ── status ────────────────────────────────────────────────────
  } else if (line == "status") {
    float curW = scale.readWeight();
    Serial.println("─────────────────────────────────────────");
    if (meas.running) {
      unsigned long elapsedMs = millis() - meas.startMs;
      float drops   = meas.gttPerMin * (elapsedMs / 60000.0f);
      float dropW   = meas.startWeight - curW;
      Serial.printf("[STATUS] 측정 중  |  유속: %.1f gtt/min\n", meas.gttPerMin);
      Serial.printf("[STATUS] 경과: %.0f초  |  현재 무게: %.4f g\n",
                    elapsedMs / 1000.0f, curW);
      Serial.printf("[STATUS] 누적 방울: %.1f gtt  |  무게 감소: %.4f g\n",
                    drops, dropW);
      if (drops > 1)
        Serial.printf("[STATUS] 현재 예상 dripFactor: %.5f g/gtt\n", dropW / drops);
    } else {
      Serial.printf("[STATUS] 대기 중  |  현재 무게: %.4f g\n", curW);
      Serial.printf("[STATUS] tare offset: %ld  |  factor: %.4f\n",
                    scale.getTareOffset(), scale.getCalibFactor());
    }
    Serial.println("─────────────────────────────────────────");

  // ── alpha ────────────────────────────────────────────────────
  } else if (line.startsWith("alpha ")) {
    float a = constrain(line.substring(6).toFloat(), 0.05f, 0.5f);
    scale.setEmaAlpha(a);
    Serial.printf("[SET] EMA alpha=%.2f\n", a);

  // ── raw ───────────────────────────────────────────────────────
  } else if (line == "raw") {
    showRaw = !showRaw;
    Serial.printf("[SET] raw 출력 %s\n", showRaw ? "ON" : "OFF");

  // ── help ──────────────────────────────────────────────────────
  } else if (line == "help") {
    Serial.println("─────────────────────────────────────────");
    Serial.println(" tare                    영점 조정");
    Serial.println(" start <gtt/min>         수동 측정 시작  예) start 60");
    Serial.println(" stop                    수동 측정 종료 + 결과 출력");
    Serial.println(" measure <gtt/min> <초>  자동 측정      예) measure 60 120");
    Serial.println(" status                  현재 진행 상황");
    Serial.println(" alpha <0.05~0.5>        EMA 부드러움 조절");
    Serial.println(" raw                     raw ADC 출력 토글");
    Serial.println(" help                    이 목록");
    Serial.println("─────────────────────────────────────────");

  } else {
    Serial.println("[ERR] 모르는 커맨드. help 입력하면 목록 나옴");
  }
}

// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== 드립 팩터 측정 (Drip Factor Calibration) ===");

  scale.begin(128);
  scale.setCalibFactor(CALIB_FACTOR_DEFAULT);
  Serial.println("[ADS] 초기화 완료. 열 안정화 대기 중...");
  Serial.printf("[ADS] 캘리 팩터: %.4f\n", scale.getCalibFactor());
  for (int i = 5; i > 0; i--) {
    Serial.printf("  tare까지 %d초...\r", i);
    delay(1000);
  }
  Serial.println();
  scale.tare(20);
  Serial.printf("[ADS] tare offset = %ld\n", scale.getTareOffset());

  Serial.println("\n[사용법]");
  Serial.println("  ① 수액팩과 폴대를 로드셀에 올린 상태에서 tare");
  Serial.println("  ② IV 컨트롤러에서 유속 설정 (예: 60 gtt/min)");
  Serial.println("  ③-A 자동: measure 60 120  (60gtt/min으로 120초 측정)");
  Serial.println("  ③-B 수동: start 60  →  (흐름 시작)  →  stop");
  Serial.println("  ④ 출력된 dripFactor를 DRIP_FACTOR_DEFAULT에 복사\n");
}

void loop() {
  handleSerial();

  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();
    float w = scale.readWeight();

    if (meas.running) {
      unsigned long elapsedMs = millis() - meas.startMs;
      float drops  = meas.gttPerMin * (elapsedMs / 60000.0f);
      float dropW  = meas.startWeight - w;
      float curFactor = (drops > 1) ? (dropW / drops) : 0;
      Serial.printf("[측정중] %.0f초  무게: %.4f g  감소: %.4f g  예상factor: %.5f\n",
                    elapsedMs / 1000.0f, w, dropW, curFactor);
    } else if (showRaw) {
      long raw = scale.readRaw();
      Serial.printf("무게: %.4f g  raw: %ld\n", w, raw);
    } else {
      Serial.printf("무게: %.4f g\n", w);
    }
  }
}
