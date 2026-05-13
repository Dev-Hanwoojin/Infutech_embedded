#pragma once
#include <Arduino.h>

/*
 * ADS1232  24-bit 차동 ADC 드라이버 (소프트웨어 SPI)
 *
 * GAIN0/GAIN1 이득 설정표:
 *   0,0 → ×1  |  1,0 → ×2  |  0,1 → ×64  |  1,1 → ×128
 *
 * 핀 연결: DOUT(DRDY), SCLK, PDWN, GAIN0, GAIN1
 */
class ADS1232 {
public:
  ADS1232(uint8_t dout, uint8_t sclk, uint8_t pdwn, uint8_t gain0, uint8_t gain1)
    : _dout(dout), _sclk(sclk), _pdwn(pdwn), _gain0(gain0), _gain1(gain1),
      _calibFactor(1.0f), _tareOffset(0),
      _emaAlpha(0.15f), _emaValue(0.0f), _emaInit(false) {}

  // ── 초기화 ────────────────────────────────────────────────────────

  void begin(uint8_t gainSetting = 128) {
    pinMode(_dout,  INPUT);
    pinMode(_sclk,  OUTPUT);
    pinMode(_pdwn,  OUTPUT);
    pinMode(_gain0, OUTPUT);
    pinMode(_gain1, OUTPUT);
    digitalWrite(_sclk, LOW);
    powerUp();
    setGain(gainSetting);
    delay(400);   // 첫 번째 변환 안정화 대기
  }

  void powerDown() { digitalWrite(_pdwn, LOW);  }
  void powerUp()   { digitalWrite(_pdwn, HIGH); }

  // 이득 설정 (1 / 2 / 64 / 128)
  void setGain(uint8_t gain) {
    switch (gain) {
      case 1:   digitalWrite(_gain0, LOW);  digitalWrite(_gain1, LOW);  break;
      case 2:   digitalWrite(_gain0, HIGH); digitalWrite(_gain1, LOW);  break;
      case 64:  digitalWrite(_gain0, LOW);  digitalWrite(_gain1, HIGH); break;
      case 128: // fall-through
      default:  digitalWrite(_gain0, HIGH); digitalWrite(_gain1, HIGH); break;
    }
  }

  // DRDY 신호 확인 (LOW = 변환 완료)
  bool isReady() { return digitalRead(_dout) == LOW; }

  // ── ADC 읽기 ──────────────────────────────────────────────────────

  // DRDY가 LOW가 될 때까지 대기 후 24비트 2의 보수 값을 읽어 반환.
  // 1초 이상 응답 없으면 0 반환 (타임아웃).
  long readRaw() {
    unsigned long deadline = millis() + 1000;
    while (digitalRead(_dout) == HIGH) {
      if (millis() > deadline) {
        Serial.println("[ADS] DRDY 타임아웃");
        return 0;
      }
      yield();
    }

    long value = 0;
    for (int i = 0; i < 24; i++) {
      digitalWrite(_sclk, HIGH); delayMicroseconds(1);
      value = (value << 1) | digitalRead(_dout);
      digitalWrite(_sclk, LOW);  delayMicroseconds(1);
    }
    // 25번째 클럭: 고속 출력 모드 선택
    digitalWrite(_sclk, HIGH); delayMicroseconds(1);
    digitalWrite(_sclk, LOW);  delayMicroseconds(1);

    // 24비트 → 32비트 부호 확장
    if (value & 0x800000) value |= 0xFF000000;
    return value;
  }

  // 필터 없이 원시 무게(g) 반환
  float readWeightRaw() {
    return (float)(readRaw() - _tareOffset) / _calibFactor;
  }

  // 중앙값 필터(3회) + EMA 저역통과 필터 적용 후 무게(g) 반환.
  // alpha: 0.05(매우 부드러움) ~ 0.3(빠른 반응), 기본값 0.15
  float readWeight() {
    // 스파이크 제거: 3회 측정 후 중간값 선택
    float a = readWeightRaw(), b = readWeightRaw(), c = readWeightRaw();
    float median;
    if      ((a <= b && b <= c) || (c <= b && b <= a)) median = b;
    else if ((b <= a && a <= c) || (c <= a && a <= b)) median = a;
    else                                               median = c;

    // EMA 저역통과 필터 (진동·잡음 평탄화)
    if (!_emaInit) { _emaValue = median; _emaInit = true; }
    else _emaValue = _emaAlpha * median + (1.0f - _emaAlpha) * _emaValue;

    return _emaValue;
  }

  // n회 원시 측정값 평균 → g 변환 (EMA 없는 정밀 스냅샷용).
  // 드립 팩터 교정처럼 정확한 순간 무게가 필요할 때 사용.
  float stableRead(int n = 10) {
    long sum = 0;
    for (int i = 0; i < n; i++) { sum += readRaw(); delay(10); }
    return (float)(sum / n - _tareOffset) / _calibFactor;
  }

  // ── EMA 설정 ──────────────────────────────────────────────────────

  // alpha 조정 (0.05 ~ 0.5): 낮을수록 부드럽지만 반응 느림
  void  setEmaAlpha(float alpha) { _emaAlpha = alpha; }
  float getEmaAlpha()      const { return _emaAlpha;  }

  // EMA 초기화 — 다음 readWeight() 호출 시 현재 값으로 즉시 시작
  void resetEma() { _emaInit = false; }

  // ── 영점 · 교정 ────────────────────────────────────────────────────

  // samples회 측정 평균으로 영점 설정. EMA도 함께 초기화.
  long tare(int samples = 10) {
    long sum = 0;
    for (int i = 0; i < samples; i++) { sum += readRaw(); delay(10); }
    _tareOffset = sum / samples;
    _emaInit    = false;
    return _tareOffset;
  }

  // knownWeight_g의 분동을 올린 상태에서 호출 → calibFactor 계산
  void calibrate(float knownWeight_g) {
    long raw = 0;
    for (int i = 0; i < 10; i++) { raw += readRaw(); delay(10); }
    raw /= 10;
    _calibFactor = (float)(raw - _tareOffset) / knownWeight_g;
  }

  void  setCalibFactor(float f) { _calibFactor = f; }
  float getCalibFactor()  const { return _calibFactor; }
  long  getTareOffset()   const { return _tareOffset;  }

private:
  uint8_t _dout, _sclk, _pdwn, _gain0, _gain1;
  float   _calibFactor;
  long    _tareOffset;
  float   _emaAlpha, _emaValue;
  bool    _emaInit;
};
