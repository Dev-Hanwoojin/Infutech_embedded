#pragma once
#include <Arduino.h>
#include <string.h>

// TFLite Micro — ESP32 코어 3.3.7 (espressif__esp-tflite-micro) 내장
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

// ── 유속 상태 정의 ─────────────────────────────────────────────────
#define FLOW_FAST    1    // 유속 빠름
#define FLOW_NORMAL  0    // 정상
#define FLOW_SLOW   -1    // 유속 느림

// ── CNN 윈도우 크기 ────────────────────────────────────────────────
// 4×4 = 16샘플 = 16초 윈도우 (측정 주기 1초 기준)
#define CNN_DIM  4
#define CNN_WIN  (CNN_DIM * CNN_DIM)   // 16샘플

// ── TFLite Micro 설정 ──────────────────────────────────────────────
#define TENSOR_ARENA_KB  16
#define TENSOR_ARENA_SZ  (TENSOR_ARENA_KB * 1024)

// ── 판정 임계값 ────────────────────────────────────────────────────
// 허용 오차 ±17%: 임상 중력식 점적 허용 범위 ±15~20% 기준
//   예) 목표 60 gtt/min → 50~70 gtt/min 정상 판정
#define DEFAULT_TOLERANCE    0.17f
// CNN 신뢰도 임계값: 이 이상일 때만 CNN 결과를 최종 판정에 반영
#define DEFAULT_CNN_THRESH   0.75f
// Fallback 비율 임계값: 이상 픽셀 비율이 이 이상이면 비정상 판정
#define DEFAULT_RATIO_THRESH 0.40f

/*
 * CNNDetector — 3클래스 유속 분류 (SLOW / NORMAL / FAST)
 *
 * CNN 출력 : [P(slow), P(normal), P(fast)] → argmax로 상태 결정
 * Fallback : CNN 모델이 없을 때 +1/-1 픽셀 비율로 판단
 *
 * 16샘플(= 16초)이 쌓인 뒤부터 getWindowResult() 유효
 */
class CNNDetector {
public:
  CNNDetector()
    : _head(0), _count(0), _lastState(FLOW_NORMAL),
      _windowResult(FLOW_NORMAL), _windowConfidence(0.0f),
      _hasResult(false), _alertPending(false),
      _tolerance(DEFAULT_TOLERANCE),
      _cnnThresh(DEFAULT_CNN_THRESH),
      _ratioThresh(DEFAULT_RATIO_THRESH),
      _model(nullptr), _interpreter(nullptr),
      _tfliteReady(false), _useFallback(false),
      _verbose(false), _inferCount(0)
  {
    memset(_buf,   0, sizeof(_buf));
    memset(_image, 0, sizeof(_image));
    memset(_arena, 0, sizeof(_arena));
    _lastProb[0] = _lastProb[1] = _lastProb[2] = 0.0f;
  }

  // TFLite 모델 초기화. 모델 파일이 없으면 Fallback 모드로 동작.
  bool begin() {
#ifndef CNN_MODEL_AVAILABLE
    Serial.println("[CNN] 모델 없음 — Fallback 모드 (train/train_cnn.py 실행 후 재컴파일).");
    _useFallback = true;
    return false;
#else
    _model = tflite::GetModel(cnn_model_data);
    if (_model->version() != TFLITE_SCHEMA_VERSION) {
      Serial.printf("[CNN] 스키마 버전 불일치: model=%u, lib=%u\n",
                    _model->version(), TFLITE_SCHEMA_VERSION);
      _useFallback = true;
      return false;
    }

    static tflite::MicroMutableOpResolver<5> resolver;
    static bool resolverReady = false;
    if (!resolverReady) {
      resolver.AddConv2D();
      resolver.AddMaxPool2D();
      resolver.AddReshape();
      resolver.AddFullyConnected();
      resolver.AddSoftmax();
      resolverReady = true;
    }

    _interpreter = new tflite::MicroInterpreter(
      _model, resolver, _arena, TENSOR_ARENA_SZ
    );

    if (_interpreter->AllocateTensors() != kTfLiteOk) {
      Serial.println("[CNN] 텐서 할당 실패 — TENSOR_ARENA_KB 값을 늘려주세요.");
      _useFallback = true;
      return false;
    }

    Serial.printf("[CNN] TFLite 준비 완료. 아레나 사용: %u / %u bytes\n",
                  (unsigned)_interpreter->arena_used_bytes(), TENSOR_ARENA_SZ);

    // 입력/출력 텐서 형상 검증 (4×4×1 입력, 3클래스 출력)
    TfLiteTensor *in  = _interpreter->input(0);
    TfLiteTensor *out = _interpreter->output(0);
    bool shapeOK = (in->dims->size == 4)
                && (in->dims->data[1] == CNN_DIM)
                && (in->dims->data[2] == CNN_DIM)
                && (in->dims->data[3] == 1)
                && (out->dims->data[1] == 3);
    if (!shapeOK) {
      Serial.println("[CNN] 텐서 형상 불일치 — 모델을 확인하세요.");
      _useFallback = true;
      return false;
    }

    _tfliteReady = true;
    return true;
#endif
  }

  // ── 파라미터 설정 ──────────────────────────────────────────────────
  void setTolerance(float t)   { _tolerance   = t; }
  void setCNNThresh(float t)   { _cnnThresh   = t; }
  void setRatioThresh(float t) { _ratioThresh = t; }

  // ── 샘플 추가 ─────────────────────────────────────────────────────
  // measured: 측정 유속(g/s), target: 목표 유속(g/s)
  // CNN_WIN(16)개 채워지면 자동으로 분류 실행 후 버퍼 초기화
  void addSample(float measured, float target) {
    _lastState  = encodeState(measured, target);
    _buf[_head] = _lastState;
    _head       = (_head + 1) % CNN_WIN;
    _count++;

    if (_count >= CNN_WIN) {
      buildImage();
      runClassify();
      _hasResult = true;
      if (_windowResult != FLOW_NORMAL) _alertPending = true;
      // 버퍼 초기화 — 다음 16샘플을 새로 수집
      memset(_buf, 0, sizeof(_buf));
      _head  = 0;
      _count = 0;
    }
  }

  // 단일 샘플을 SLOW / NORMAL / FAST 중 하나로 분류
  int encodeState(float measured, float target) {
    if (target <= 0) return FLOW_NORMAL;
    float ratio = (measured - target) / target;
    if (ratio >  _tolerance) return FLOW_FAST;
    if (ratio < -_tolerance) return FLOW_SLOW;
    return FLOW_NORMAL;
  }

  // ── 결과 조회 ─────────────────────────────────────────────────────
  int   getWindowResult()     const { return _windowResult;     }  // 윈도우 분류 결과
  float getWindowConfidence() const { return _windowConfidence; }  // 결과 신뢰도 (0~1)
  bool  isWindowFull()        const { return _hasResult;        }  // 첫 윈도우 완성 여부
  int   getSampleCount()      const { return _count;            }  // 현재 수집된 샘플 수
  int   getLastState()        const { return _lastState;        }  // 최근 단일 샘플 상태
  bool  isTFLiteActive()      const { return _tfliteReady;      }  // TFLite 활성화 여부

  // 이상 감지 플래그 — 윈도우 완성 직후 1회만 true 반환, 이후 자동 클리어
  bool detectAnomaly() {
    if (_alertPending) { _alertPending = false; return true; }
    return false;
  }

  // 결과를 한국어 문자열로 반환
  // 빠름/느림 구분 없이 통합 "수액 이상 발생" 표시 (임상 단순화)
  const char* getResultLabel() const {
    if (!isWindowFull()) return "수집중";
    switch (_windowResult) {
      case FLOW_FAST:
      case FLOW_SLOW: return "수액 이상 발생";
      default:        return "정상";
    }
  }

  // 디버그용 — 내부 빠름/느림 구분 (필요 시 사용)
  const char* getResultDetail() const {
    if (!isWindowFull()) return "collecting";
    switch (_windowResult) {
      case FLOW_FAST: return "fast";
      case FLOW_SLOW: return "slow";
      default:        return "normal";
    }
  }

  void getImage(int out[CNN_DIM][CNN_DIM]) {
    memcpy(out, _image, sizeof(_image));
  }

  // ── 디버그 기능 ───────────────────────────────────────────────────
  // verbose: true → 매 추론마다 입력 이미지 + 출력 확률/비율 시리얼 출력
  void setVerbose(bool v) { _verbose = v; }
  bool getVerbose() const { return _verbose; }

  // 현재 CNN 엔진 상태를 시리얼로 상세 출력 (시리얼 'cnn' 명령에서 호출)
  void printDebugInfo() {
    Serial.println("───── CNN 진단 정보 ─────");
    Serial.printf("  추론 엔진   : %s\n",
                  _tfliteReady ? "TFLite Micro (학습된 모델)" : "Fallback (픽셀 비율)");
    Serial.printf("  모델 컴파일 : %s\n",
#ifdef CNN_MODEL_AVAILABLE
                  "CNN_MODEL_AVAILABLE 정의됨 (모델 헤더 포함)"
#else
                  "정의 안 됨 → cnn_model.h 비어있음 (Fallback 강제)"
#endif
    );
    Serial.printf("  윈도우 크기 : %dx%d = %d 샘플 (%d초)\n",
                  CNN_DIM, CNN_DIM, CNN_WIN, CNN_WIN);
    Serial.printf("  누적 추론 수: %lu 회\n", _inferCount);
    Serial.printf("  현재 수집   : %d / %d 샘플\n", _count, CNN_WIN);
    Serial.printf("  허용오차    : ±%.0f%%  | CNN임계:%.2f | 비율임계:%.2f\n",
                  _tolerance * 100.0f, _cnnThresh, _ratioThresh);
#ifdef CNN_MODEL_AVAILABLE
    if (_tfliteReady && _interpreter) {
      Serial.printf("  Arena 사용  : %u / %u bytes\n",
                    (unsigned)_interpreter->arena_used_bytes(), TENSOR_ARENA_SZ);
      TfLiteTensor *in  = _interpreter->input(0);
      TfLiteTensor *out = _interpreter->output(0);
      Serial.printf("  입력 텐서   : [%d,%d,%d,%d]\n",
                    in->dims->data[0], in->dims->data[1],
                    in->dims->data[2], in->dims->data[3]);
      Serial.printf("  출력 텐서   : [%d,%d]\n",
                    out->dims->data[0], out->dims->data[1]);
    }
#endif
    if (_hasResult) {
      Serial.printf("  최근 결과   : %s (신뢰도 %.1f%%)\n",
                    getResultDetail(), _windowConfidence * 100.0f);
      Serial.printf("  최근 확률   : slow=%.3f normal=%.3f fast=%.3f\n",
                    _lastProb[0], _lastProb[1], _lastProb[2]);
    } else {
      Serial.println("  최근 결과   : 아직 윈도우 미완성");
    }
    Serial.printf("  verbose     : %s\n", _verbose ? "ON" : "OFF");
    Serial.println("─────────────────────────");
  }

  // 전체 초기화
  void reset() {
    memset(_buf,   0, sizeof(_buf));
    memset(_image, 0, sizeof(_image));
    _head             = 0;
    _count            = 0;
    _lastState        = FLOW_NORMAL;
    _windowResult     = FLOW_NORMAL;
    _windowConfidence = 0.0f;
    _hasResult        = false;
    _alertPending     = false;
  }

private:
  int   _buf[CNN_WIN];
  int   _image[CNN_DIM][CNN_DIM];
  int   _head, _count, _lastState;
  int   _windowResult;
  float _windowConfidence;
  bool  _hasResult, _alertPending;

  float _tolerance, _cnnThresh, _ratioThresh;

  const tflite::Model      *_model;
  tflite::MicroInterpreter *_interpreter;
  alignas(16) uint8_t       _arena[TENSOR_ARENA_SZ];
  bool _tfliteReady, _useFallback;

  // ── 디버그 상태 ──
  bool          _verbose;        // 추론 상세 로그 on/off
  unsigned long _inferCount;     // 누적 추론 횟수
  float         _lastProb[3];    // 최근 [slow, normal, fast] 확률/비율

  // verbose 모드일 때 입력 이미지를 시리얼에 출력
  void printVerboseImage() {
    Serial.printf("[CNN#%lu] %s 추론 입력 이미지:\n",
                  _inferCount, _tfliteReady ? "TFLite" : "Fallback");
    for (int r = 0; r < CNN_DIM; r++) {
      Serial.print("        ");
      for (int c = 0; c < CNN_DIM; c++) {
        if      (_image[r][c] == FLOW_FAST) Serial.print('+');
        else if (_image[r][c] == FLOW_SLOW) Serial.print('-');
        else                                Serial.print('.');
      }
      Serial.println();
    }
  }

  // 1차원 버퍼 → 2차원 이미지 변환
  void buildImage() {
    for (int r = 0; r < CNN_DIM; r++)
      for (int c = 0; c < CNN_DIM; c++)
        _image[r][c] = _buf[(_head + r * CNN_DIM + c) % CNN_WIN];
  }

  // 추론 실행 → _windowResult, _windowConfidence 갱신
  void runClassify() {
    _inferCount++;
    if (_verbose) printVerboseImage();
    _tfliteReady ? classifyCNN() : classifyFallback();
    if (_verbose) {
      Serial.printf("[CNN#%lu] 결과: %s | 확률 slow=%.3f normal=%.3f fast=%.3f\n",
                    _inferCount, getResultDetail(),
                    _lastProb[0], _lastProb[1], _lastProb[2]);
    }
  }

  // TFLite 모델로 3클래스 분류
  void classifyCNN() {
#ifdef CNN_MODEL_AVAILABLE
    TfLiteTensor *input = _interpreter->input(0);
    for (int r = 0; r < CNN_DIM; r++)
      for (int c = 0; c < CNN_DIM; c++)
        input->data.f[r * CNN_DIM + c] = (float)_image[r][c];

    if (_interpreter->Invoke() != kTfLiteOk) {
      Serial.println("[CNN] Invoke 실패 — Fallback으로 전환.");
      classifyFallback();
      return;
    }

    // 출력: [P(slow), P(normal), P(fast)]
    float pSlow   = _interpreter->output(0)->data.f[0];
    float pNormal = _interpreter->output(0)->data.f[1];
    float pFast   = _interpreter->output(0)->data.f[2];
    _lastProb[0] = pSlow; _lastProb[1] = pNormal; _lastProb[2] = pFast;

    if (pNormal >= _cnnThresh) {
      _windowResult     = FLOW_NORMAL;
      _windowConfidence = pNormal;
    } else if (pFast > pSlow) {
      _windowResult     = FLOW_FAST;
      _windowConfidence = pFast;
    } else {
      _windowResult     = FLOW_SLOW;
      _windowConfidence = pSlow;
    }
#endif
  }

  // Fallback: +1/-1 픽셀 비율로 판단 (모델 없을 때 사용)
  void classifyFallback() {
    int fast = 0, slow = 0;
    for (int r = 0; r < CNN_DIM; r++)
      for (int c = 0; c < CNN_DIM; c++) {
        if (_image[r][c] == FLOW_FAST) fast++;
        if (_image[r][c] == FLOW_SLOW) slow++;
      }

    float ratioFast = (float)fast / CNN_WIN;
    float ratioSlow = (float)slow / CNN_WIN;
    // Fallback 은 확률이 아니라 픽셀 비율 — 디버그 표시용으로 저장
    _lastProb[0] = ratioSlow;
    _lastProb[1] = 1.0f - ratioFast - ratioSlow;
    _lastProb[2] = ratioFast;

    if      (ratioFast >= _ratioThresh && ratioFast > ratioSlow) {
      _windowResult     = FLOW_FAST;
      _windowConfidence = ratioFast;
    } else if (ratioSlow >= _ratioThresh && ratioSlow > ratioFast) {
      _windowResult     = FLOW_SLOW;
      _windowConfidence = ratioSlow;
    } else {
      _windowResult     = FLOW_NORMAL;
      _windowConfidence = 1.0f - max(ratioFast, ratioSlow);
    }
  }
};
