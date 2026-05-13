#pragma once
#include <Arduino.h>

// ADS1232 24-bit differential ADC driver (bit-bang)
// GAIN0/GAIN1 truth table:
//   0,0 → ×1  |  1,0 → ×2  |  0,1 → ×64  |  1,1 → ×128
class ADS1232 {
public:
  ADS1232(uint8_t dout, uint8_t sclk, uint8_t pdwn, uint8_t gain0, uint8_t gain1)
    : _dout(dout), _sclk(sclk), _pdwn(pdwn), _gain0(gain0), _gain1(gain1),
      _calibFactor(1.0f), _tareOffset(0),
      _emaAlpha(0.15f), _emaValue(0.0f), _emaInit(false) {}

  void begin(uint8_t gainSetting = 128) {
    pinMode(_dout,  INPUT);
    pinMode(_sclk,  OUTPUT);
    pinMode(_pdwn,  OUTPUT);
    pinMode(_gain0, OUTPUT);
    pinMode(_gain1, OUTPUT);
    digitalWrite(_sclk, LOW);
    powerUp();
    setGain(gainSetting);
    delay(400); // first conversion settle
  }

  void powerDown() { digitalWrite(_pdwn, LOW); }
  void powerUp()   { digitalWrite(_pdwn, HIGH); }

  void setGain(uint8_t gain) {
    switch (gain) {
      case 1:   digitalWrite(_gain0, LOW);  digitalWrite(_gain1, LOW);  break;
      case 2:   digitalWrite(_gain0, HIGH); digitalWrite(_gain1, LOW);  break;
      case 64:  digitalWrite(_gain0, LOW);  digitalWrite(_gain1, HIGH); break;
      case 128: // fall-through
      default:  digitalWrite(_gain0, HIGH); digitalWrite(_gain1, HIGH); break;
    }
  }

  bool isReady() { return digitalRead(_dout) == LOW; }

  // Blocks until DRDY goes LOW, then clocks out 24-bit two's-complement value.
  // Returns 0 on 1-second timeout.
  long readRaw() {
    unsigned long deadline = millis() + 1000;
    while (digitalRead(_dout) == HIGH) {
      if (millis() > deadline) {
        Serial.println("[ADS] DRDY timeout");
        return 0;
      }
      yield();
    }

    long value = 0;
    for (int i = 0; i < 24; i++) {
      digitalWrite(_sclk, HIGH);
      delayMicroseconds(1);
      value = (value << 1) | digitalRead(_dout);
      digitalWrite(_sclk, LOW);
      delayMicroseconds(1);
    }
    // 25th pulse selects high-speed output rate
    digitalWrite(_sclk, HIGH); delayMicroseconds(1);
    digitalWrite(_sclk, LOW);  delayMicroseconds(1);

    // Sign-extend 24-bit → 32-bit
    if (value & 0x800000) value |= 0xFF000000;
    return value;
  }

  // Raw → grams, no filtering
  float readWeightRaw() {
    return (float)(readRaw() - _tareOffset) / _calibFactor;
  }

  // Median-of-3 spike rejection + EMA smoothing
  // alpha: 0.05(매우 부드럽) ~ 0.3(빠른 반응), 기본 0.15
  float readWeight() {
    // 1) 중앙값 필터: 3회 측정 후 중간값 선택 (스파이크 제거)
    float a = readWeightRaw();
    float b = readWeightRaw();
    float c = readWeightRaw();
    float median;
    if ((a <= b && b <= c) || (c <= b && b <= a)) median = b;
    else if ((b <= a && a <= c) || (c <= a && a <= b)) median = a;
    else median = c;

    // 2) EMA 저역통과 필터 (진동·잡음 평탄화)
    if (!_emaInit) { _emaValue = median; _emaInit = true; }
    else _emaValue = _emaAlpha * median + (1.0f - _emaAlpha) * _emaValue;

    return _emaValue;
  }

  // EMA 계수 조정 (0.05 ~ 0.5)
  // 낮을수록 부드럽지만 반응 느림, 높을수록 빠르지만 노이즈 많음
  void setEmaAlpha(float alpha) { _emaAlpha = alpha; }
  float getEmaAlpha() const { return _emaAlpha; }

  // EMA 상태 초기화 — 다음 readWeight() 호출 시 현재 값으로 즉시 시작
  void resetEma() { _emaInit = false; }

  // Average `samples` readings to set the zero offset.
  // EMA 상태도 함께 초기화.
  long tare(int samples = 10) {
    long sum = 0;
    for (int i = 0; i < samples; i++) { sum += readRaw(); delay(10); }
    _tareOffset = sum / samples;
    _emaInit = false;   // EMA 초기화 (다음 읽기에서 새 기준으로 시작)
    return _tareOffset;
  }

  // Place a `knownWeight_g` load on the scale, then call this.
  void calibrate(float knownWeight_g) {
    long raw = 0;
    for (int i = 0; i < 10; i++) { raw += readRaw(); delay(10); }
    raw /= 10;
    _calibFactor = (float)(raw - _tareOffset) / knownWeight_g;
  }

  void  setCalibFactor(float f) { _calibFactor = f; }
  float getCalibFactor() const  { return _calibFactor; }
  long  getTareOffset()  const  { return _tareOffset; }

private:
  uint8_t _dout, _sclk, _pdwn, _gain0, _gain1;
  float   _calibFactor;
  long    _tareOffset;
  float   _emaAlpha;
  float   _emaValue;
  bool    _emaInit;
};
