"""
Smart IV Pole — 실측 데이터 CNN 학습 스크립트

합성 데이터(train_cnn.py) 대신 ESP32 에서 수집한 실제 측정 데이터로 학습한다.

────────────────────────────────────────────────────────────────────
1. 데이터 수집 (ESP32)
   - 시리얼 모니터에서 'log on' 입력
   - 측정이 진행되면 매 샘플마다 다음 형식의 줄이 출력됨:
       LOG,<ms>,<flow_gs>,<target_gs>,<state>
   - 시리얼 출력을 텍스트로 캡처/저장 (예: PuTTY 로그, Arduino IDE 복사)
   - 'log off' 로 종료

2. 라벨링 (PC, 수동)
   - 캡처한 텍스트에서 "LOG," 로 시작하는 줄만 모아 CSV 로 저장
   - 맨 끝에 label 컬럼을 직접 추가:  slow / normal / fast
       ms,flow_gs,target_gs,state,label
       2000,0.0510,0.0500,0,normal
       4000,0.0300,0.0500,-1,slow
       ...
   - 파일을 train/data/ 폴더에 저장 (여러 개 가능)

3. 학습 (이 스크립트)
       python train/train_real.py
   - train/data/*.csv 를 모두 읽어 16샘플 슬라이딩 윈도우로 자름
   - 각 윈도우 → 4x4 이미지 (state -1/0/+1)
   - 윈도우 라벨 = 윈도우 내 다수결 label
   - 학습 후 ../cnn_model.h 생성 (train_cnn.py 와 동일 구조)
────────────────────────────────────────────────────────────────────

주의:
  - state 컬럼이 이미 -1/0/+1 이면 그대로 이미지로 사용.
  - 데이터가 부족하면(클래스당 수백 윈도우 미만) 과적합/저정확도 가능.
    실측이 적으면 train_cnn.py 합성 데이터와 섞는 것도 방법.
"""

import os
import glob
import csv
import numpy as np
import tensorflow as tf
from tensorflow import keras

np.random.seed(42)
tf.random.set_seed(42)

CNN_DIM   = 4
WIN_SIZE  = CNN_DIM * CNN_DIM          # 16
STRIDE    = 1                          # 슬라이딩 윈도우 간격 (1=최대 데이터)
DATA_DIR  = os.path.join(os.path.dirname(__file__), 'data')

# 라벨 문자열 → 정수 (cnn_detector.h FLOW_* 와 동일: 0=slow 1=normal 2=fast)
LABEL_MAP = {
    'slow':   0, 'feel_slow': 0, '-1': 0,
    'normal': 1, 'ok': 1,        '0':  1,
    'fast':   2, 'feel_fast': 2, '1':  2,
}


def load_csv_files(data_dir):
    """data/*.csv 를 읽어 (states, labels) 시퀀스 리스트 반환.
       각 파일은 시간순 정렬되어 있다고 가정."""
    files = sorted(glob.glob(os.path.join(data_dir, '*.csv')))
    if not files:
        raise SystemExit(
            f"[오류] {data_dir} 에 CSV 가 없습니다.\n"
            f"      ESP32 'log on' 으로 수집 → label 컬럼 추가 → 이 폴더에 저장하세요."
        )

    sessions = []   # [(states[], labels[]), ...] 파일(=세션) 단위
    for path in files:
        states, labels = [], []
        with open(path, newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            # 컬럼명 정규화 (공백/대문자 무시)
            for row in reader:
                row = { (k or '').strip().lower(): (v or '').strip()
                        for k, v in row.items() }
                if 'state' not in row or 'label' not in row:
                    continue
                try:
                    st = int(float(row['state']))
                except ValueError:
                    continue
                st = max(-1, min(1, st))            # -1/0/+1 클램프
                lab = LABEL_MAP.get(row['label'].lower())
                if lab is None:
                    continue
                states.append(float(st))
                labels.append(lab)
        if len(states) >= WIN_SIZE:
            sessions.append((np.array(states, dtype=np.float32),
                             np.array(labels, dtype=np.int32)))
            print(f"  로드: {os.path.basename(path)}  ({len(states)} 샘플)")
        else:
            print(f"  건너뜀: {os.path.basename(path)}  (16샘플 미만)")
    return sessions


def make_windows(sessions):
    """세션별로 16샘플 슬라이딩 윈도우 생성.
       윈도우 라벨 = 윈도우 내 다수결."""
    X, y = [], []
    for states, labels in sessions:
        n = len(states)
        for start in range(0, n - WIN_SIZE + 1, STRIDE):
            win_s = states[start:start + WIN_SIZE]
            win_l = labels[start:start + WIN_SIZE]
            # 다수결 라벨
            major = np.bincount(win_l, minlength=3).argmax()
            X.append(win_s.reshape(CNN_DIM, CNN_DIM, 1))
            y.append(major)
    return np.array(X, dtype=np.float32), np.array(y, dtype=np.int32)


# ── 데이터 로드 ────────────────────────────────────────────────────
print("실측 CSV 로드 중...")
sessions = load_csv_files(DATA_DIR)
X, y = make_windows(sessions)
print(f"\n총 윈도우: {len(X)}개")
for cls, name in [(0, 'slow'), (1, 'normal'), (2, 'fast')]:
    print(f"  {name:6s}: {int(np.sum(y == cls))}개")

if len(X) < 30:
    raise SystemExit("[오류] 윈도우가 너무 적습니다(<30). 데이터를 더 수집하세요.")

# 셔플 + 85/15 분할
idx = np.random.permutation(len(X))
X, y = X[idx], y[idx]
y_oh = keras.utils.to_categorical(y, 3)
split = int(len(X) * 0.85)
X_tr, X_val = X[:split], X[split:]
y_tr, y_val = y_oh[:split], y_oh[split:]
print(f"학습 {len(X_tr)} / 검증 {len(X_val)}")


# ── 모델 (train_cnn.py 와 동일 구조 — TFLite Micro 호환) ───────────
_POOLED = (CNN_DIM // 2) * (CNN_DIM // 2) * 8   # 32

model = keras.Sequential([
    keras.layers.Input(shape=(CNN_DIM, CNN_DIM, 1), batch_size=1),
    keras.layers.Conv2D(8, (3, 3), padding='same', activation='relu'),
    keras.layers.MaxPooling2D((2, 2)),
    keras.layers.Reshape((_POOLED,)),
    keras.layers.Dense(16, activation='relu'),
    keras.layers.Dense(3,  activation='softmax'),
], name='iv_cnn_4x4_real')
model.summary()

model.compile(optimizer=keras.optimizers.Adam(1e-3),
              loss='categorical_crossentropy', metrics=['accuracy'])

callbacks = [
    keras.callbacks.EarlyStopping(patience=10, restore_best_weights=True),
    keras.callbacks.ReduceLROnPlateau(factor=0.5, patience=5, verbose=1),
]
model.fit(X_tr, y_tr, validation_data=(X_val, y_val),
          epochs=100, batch_size=32, callbacks=callbacks, verbose=1)

val_loss, val_acc = model.evaluate(X_val, y_val, verbose=0)
print(f"\n검증 정확도: {val_acc*100:.1f}%")
if val_acc < 0.85:
    print("[경고] 정확도 85% 미만 — 데이터 수집량/라벨 품질을 점검하세요.")


# ── TFLite 변환 (순수 float32) ─────────────────────────────────────
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()
size_kb = len(tflite_model) / 1024
print(f"TFLite 모델 크기: {size_kb:.1f} KB")


# ── cnn_model.h 생성 ───────────────────────────────────────────────
OUT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'cnn_model.h'))
b = list(tflite_model); n = len(b)
lines = [
    "// Auto-generated by train/train_real.py (실측 데이터) — DO NOT EDIT",
    f"// 모델 크기: {size_kb:.1f} KB ({n} bytes) | 검증 정확도: {val_acc*100:.1f}%",
    f"// 학습 윈도우: {len(X)}개 (실측)",
    "#pragma once", "",
    "#define CNN_MODEL_AVAILABLE", "",
    "alignas(8) const unsigned char cnn_model_data[] = {",
]
for i in range(0, n, 12):
    lines.append("  " + ", ".join(f"0x{x:02x}" for x in b[i:i+12]) + ",")
lines.append("};")
lines.append(f"const unsigned int cnn_model_data_len = {n};")
lines.append("")

with open(OUT, 'w', encoding='utf-8') as f:
    f.write('\n'.join(lines))
print(f"생성 완료: {OUT}")
print("Arduino 스케치를 재컴파일하세요.")
