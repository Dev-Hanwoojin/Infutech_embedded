// ─────────────────────────────────────────────────────────────────
//  ADS1232 Load Cell Calibration Sketch
//  폴더 위치: ESP32_embedded/loadcell_cali/
//
//  시리얼 모니터 설정: 115200 baud, 줄바꿈(newline) 포함
//
//  사용 순서:
//    1. 아무것도 올리지 않은 상태에서  → tare
//    2. 알고 있는 무게를 올리고        → calib <그램>
//    3. 출력되는 factor 값을
//       Smart_IV_Pole.ino 의 CALIB_FACTOR_DEFAULT 에 복사
//
//  커맨드:
//    tare              — 영점 조정 (아무것도 없는 상태)
//    calib <g>         — 알려진 무게로 자동 계산  예) calib 200
//    factor <value>    — 캘리 팩터 직접 입력      예) factor 420.5
//    alpha <0.05~0.5>  — EMA 부드러움 조절        예) alpha 0.1
//    raw               — raw/filtered 출력 토글
//    help              — 커맨드 목록
// ─────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include "../Smart_IV_Pole/ads1232.h"

// ── 핀 설정 (Smart_IV_Pole 과 동일) ───────────────────────────────
#define ADS_DOUT  19
#define ADS_SCLK  18
#define ADS_PDWN  23
#define ADS_GAIN0 33
#define ADS_GAIN1 32

#define PRINT_INTERVAL_MS  500   // 출력 주기

ADS1232 scale(ADS_DOUT, ADS_SCLK, ADS_PDWN, ADS_GAIN0, ADS_GAIN1);

bool showRaw = false;   // true: raw ADC값도 같이 출력

// ─────────────────────────────────────────────────────────────────
void printHelp() {
  Serial.println("─────────────────────────────────────────");
  Serial.println(" tare                영점 조정");
  Serial.println(" autocal <g>         자동 수렴 캘리  예) autocal 1000");
  Serial.println(" calib <g>           단발성 캘리 계산");
  Serial.println(" factor <value>      캘리 팩터 직접 설정");
  Serial.println(" alpha <0.05~0.5>    EMA 부드러움 (낮을수록 부드러움)");
  Serial.println(" raw                 raw ADC 출력 토글");
  Serial.println(" help                이 목록");
  Serial.println("─────────────────────────────────────────");
}

// ── 자동 캘리브레이션 ─────────────────────────────────────────────
// factor_new = factor_old × (reading / target)
// 오차가 목표값의 0.1% 이내로 들어오면 수렴으로 판단.
void autoCalibrate(float targetG) {
  const float CONVERGE_G  = targetG * 0.001f;  // 수렴 기준: ±0.1%
  const float DAMPING     = 0.4f;              // 댐핑 (0<d<1, 낮을수록 천천히)
  const int   MAX_ITER    = 80;
  const int   SAMPLE_N    = 5;                 // 반복당 평균 샘플 수

  Serial.println("─────────────────────────────────────────");
  Serial.printf("[AutoCal] 목표: %.2f g   수렴기준: ±%.3f g\n", targetG, CONVERGE_G);
  Serial.println("[AutoCal] iter |  reading(g)  |  error(g)  |  factor");
  Serial.println("─────────────────────────────────────────");

  for (int iter = 1; iter <= MAX_ITER; iter++) {
    // 여러 샘플 평균 (노이즈 억제)
    float sum = 0;
    for (int s = 0; s < SAMPLE_N; s++) {
      sum += (float)(scale.readRaw() - scale.getTareOffset()) / scale.getCalibFactor();
      delay(50);
    }
    float reading = sum / SAMPLE_N;
    float error   = reading - targetG;

    Serial.printf("[AutoCal] %4d  |  %10.3f  |  %+8.3f  |  %.6f\n",
                  iter, reading, error, scale.getCalibFactor());

    // 수렴 확인
    if (fabsf(error) <= CONVERGE_G) {
      scale.resetEma();   // EMA 묵은 상태 제거 → 이후 readWeight()가 즉시 정확한 값 출력
      Serial.println("─────────────────────────────────────────");
      Serial.printf("[AutoCal] ✔ 수렴 완료! factor = %.6f\n", scale.getCalibFactor());
      Serial.printf("[AutoCal]   최종 reading = %.3f g  (오차 %+.3f g)\n", reading, error);
      Serial.println("[AutoCal] → CALIB_FACTOR_DEFAULT 에 위 factor 값을 복사하세요");
      Serial.println("─────────────────────────────────────────");
      return;
    }

    // factor 업데이트: 댐핑 적용한 비례 조정
    // reading/target 비율만큼 factor를 줄이면 reading이 target으로 올라감
    float idealFactor = scale.getCalibFactor() * (reading / targetG);
    float newFactor   = scale.getCalibFactor() + DAMPING * (idealFactor - scale.getCalibFactor());
    scale.setCalibFactor(newFactor);

    delay(500);
  }

  scale.resetEma();
  Serial.println("─────────────────────────────────────────");
  Serial.printf("[AutoCal] 최대 반복 도달. 현재 factor = %.6f\n", scale.getCalibFactor());
  Serial.println("[AutoCal] 노이즈가 크면 alpha 값을 낮추고 다시 시도하세요");
  Serial.println("─────────────────────────────────────────");
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line == "tare") {
    Serial.println("[CAL] 영점 조정 중... (20샘플)");
    long offset = scale.tare(20);
    Serial.printf("[CAL] 완료. tare offset = %ld\n", offset);

  } else if (line.startsWith("calib ")) {
    float known = line.substring(6).toFloat();
    if (known <= 0) { Serial.println("[CAL] 무게 값이 잘못됨"); return; }
    Serial.printf("[CAL] %.1fg 기준으로 캘리 계산 중...\n", known);
    scale.calibrate(known);
    Serial.printf("[CAL] ✔ factor = %.4f\n", scale.getCalibFactor());
    Serial.println("[CAL] → Smart_IV_Pole.ino 의 CALIB_FACTOR_DEFAULT 에 이 값을 복사하세요");

  } else if (line.startsWith("factor ")) {
    float f = line.substring(7).toFloat();
    if (f == 0) { Serial.println("[CAL] 값이 잘못됨"); return; }
    scale.setCalibFactor(f);
    Serial.printf("[CAL] factor = %.4f 으로 설정\n", f);

  } else if (line.startsWith("alpha ")) {
    float a = constrain(line.substring(6).toFloat(), 0.05f, 0.5f);
    scale.setEmaAlpha(a);
    Serial.printf("[CAL] EMA alpha = %.2f\n", a);

  } else if (line.startsWith("autocal ")) {
    float target = line.substring(8).toFloat();
    if (target <= 0) { Serial.println("[CAL] 무게 값이 잘못됨"); return; }
    autoCalibrate(target);

  } else if (line == "raw") {
    showRaw = !showRaw;
    Serial.printf("[CAL] raw 출력 %s\n", showRaw ? "ON" : "OFF");

  } else if (line == "help") {
    printHelp();

  } else {
    Serial.println("[CAL] 모르는 커맨드. help 입력하면 목록 나옴");
  }
}

// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ADS1232 Load Cell Calibration ===");

  scale.begin(128);
  Serial.println("[ADS] 초기화 완료. 열 안정화 대기 중...");
  for (int i = 10; i > 0; i--) {
    Serial.printf("  tare까지 %d초...\r", i);
    delay(1000);
  }
  Serial.println();
  scale.tare(20);
  Serial.printf("[ADS] tare offset = %ld\n", scale.getTareOffset());
  Serial.printf("[ADS] 기본 factor = %.4f  (아직 캘리 안 됨)\n", scale.getCalibFactor());

  printHelp();
  Serial.println("\n[출력 형식]  weight: <g>   (raw: <ADC값>)");
  Serial.println("────────────────────────────────────────\n");
}

unsigned long lastPrint = 0;

void loop() {
  handleSerial();

  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();

    float w = scale.readWeight();

    if (showRaw) {
      // raw는 필터 없이 직접 읽기
      long raw = scale.readRaw();
      Serial.printf("weight: %8.2f g    raw: %ld    tare: %ld    factor: %.4f\n",
                    w, raw, scale.getTareOffset(), scale.getCalibFactor());
    } else {
      Serial.printf("weight: %8.2f g    factor: %.4f\n",
                    w, scale.getCalibFactor());
    }
  }
}
