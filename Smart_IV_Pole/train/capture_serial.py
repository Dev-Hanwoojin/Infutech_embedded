"""
시리얼 캡처 → 학습용 CSV 자동 저장

ESP32 가 'log <label>' 로 출력하는 "LOG," 줄을 실시간으로 받아
train/data/ 폴더에 CSV 로 저장한다. (수동 복사/편집 불필요)

사용법:
    pip install pyserial
    python train/capture_serial.py --port COM15 --out session_001

    그 후 ESP32 시리얼에 직접 'log slow' 등을 입력하거나,
    이 스크립트가 열어둔 동안 Arduino 시리얼 모니터는 닫아야 함
    (포트는 한 프로그램만 점유 가능).

    Ctrl+C 로 종료 → data/<out>.csv 저장 완료.

동작:
    - "LOG_HEADER," 줄을 만나면 헤더로 사용
    - "LOG," 로 시작하는 줄을 데이터 행으로 저장 (LOG, 접두어 제거)
    - 라벨은 ESP32 가 이미 찍어주므로 그대로 저장됨
"""

import argparse
import os
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial 이 필요합니다:  pip install pyserial")

DATA_DIR = os.path.join(os.path.dirname(__file__), 'data')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True, help='시리얼 포트 (예: COM15, /dev/ttyUSB0)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--out',  default='session', help='출력 파일명 (확장자 제외)')
    args = ap.parse_args()

    os.makedirs(DATA_DIR, exist_ok=True)
    out_path = os.path.join(DATA_DIR, f"{args.out}.csv")

    # 중복 방지: 이미 있으면 _2, _3 ...
    base, n = out_path, 2
    while os.path.exists(out_path):
        out_path = base.replace('.csv', f'_{n}.csv'); n += 1

    print(f"포트 {args.port} @ {args.baud} 열기...")
    print(f"저장: {out_path}")
    print("ESP32 시리얼에 'log slow/normal/fast' 입력 후 측정하세요. Ctrl+C 로 종료.\n")

    header = "ms,flow_gs,target_gs,state,label"   # 기본값 (LOG_HEADER 받으면 갱신)
    rows = []
    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            while True:
                raw = ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw:
                    continue
                if raw.startswith('LOG_HEADER,'):
                    header = raw[len('LOG_HEADER,'):]
                    print(f"[헤더] {header}")
                elif raw.startswith('LOG,'):
                    row = raw[len('LOG,'):]
                    rows.append(row)
                    print(f"[{len(rows):4d}] {row}")
                # 그 외 일반 로그는 무시
    except KeyboardInterrupt:
        print("\n종료 — 저장 중...")
    except serial.SerialException as e:
        sys.exit(f"시리얼 오류: {e}")

    if not rows:
        print("수집된 LOG 행이 없습니다. ESP32 에서 'log <label>' 했는지 확인하세요.")
        return

    with open(out_path, 'w', encoding='utf-8', newline='') as f:
        f.write(header + '\n')
        f.write('\n'.join(rows) + '\n')
    print(f"완료: {out_path}  ({len(rows)} 행)")
    print("이제 train/train_real.py 로 학습하세요.")


if __name__ == '__main__':
    main()
