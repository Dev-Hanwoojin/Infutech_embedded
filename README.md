# Smart IV Pole — ESP32 펌웨어 연구개발 노트

> 인퓨테크 미니 스마트 링거 폴대 (Infutech mini Smart IV Pole) 임베디드 시스템

| 항목 | 내용 |
|------|------|
| **칩** | ESP32-WROOM-32E (4MB Flash, 듀얼코어, WiFi + BLE) |
| **ADC** | ADS1232 (24-bit, 로드셀 무게 측정) |
| **작성** | 한우진 |

---

## 0. 연구 개요

수액(IV) 주입 상태를 로드셀로 무게 변화를 측정하여 실시간 유속을 산출하고,
CNN(TFLite Micro) 기반으로 유속 이상(빠름/느림)을 탐지하는 임베디드 시스템.
모바일(Flutter) 앱에서 BLE로 WiFi를 설정하고, 백엔드와 MQTT로 연동한다.

**핵심 설계 목표**

- 실무(병동)에서 시리얼/PC 조작 없이 전원만 켜면 자동 동작
- IV 세트 종류(drip factor)에 무관한 범용 유속 교정
- 저사양 MCU 위에서 BLE + WiFi + CNN 추론 동시 구동

---

## 1. 로드셀 무게 측정 & 캘리브레이션 팩터

### ADS1232 드라이버

- 소프트웨어 SPI(bit-bang)로 24비트 차동 ADC 읽기
- 핀: `DOUT(19)`, `SCLK(18)`, `PDWN(23)`, `GAIN0(33)`, `GAIN1(32)`
- 게인 128 사용 (로드셀 미세 신호 증폭)

### 무게 환산식

```
weight(g) = (raw_adc - tareOffset) / calibFactor
```

- `tareOffset` : 영점(빈 상태) 측정값. `tare()`로 N회 평균
- `calibFactor` : 단위 무게당 ADC 증분. 분동 교정으로 결정

### 캘리브레이션 팩터 자동 탐색

**기존 방식 문제** — `calibFactor`를 시리얼로 사용자가 직접 입력해가며 무게값이 정확하게 나올 때까지 수정하는 방식을 사용했었음.

**해결** — `calibrate(knownWeight_g)` 메서드

- 영점(tare) 후 무게가 정확한 분동(예: 500g)을 올린 상태에서 호출
- `calibFactor = (raw - tareOffset) / knownWeight`
- 즉, 알려진 무게 1점으로 선형 계수를 역산하여 자동 결정

탐색 과정에서 확정한 실측 캘리값:

```cpp
#define CALIB_FACTOR_DEFAULT 1642.8623
```

→ 이후 이 값을 코드에 고정(하드코딩)하여 매 부팅 시 분동 교정 생략

### 측정 필터링

- **스파이크 제거** : 3회 측정 후 중앙값(median) 선택
- **EMA 저역통과 필터** : `alpha=0.15` 기본 (진동/잡음 평탄화)
- **`stableRead(n)`** : EMA 없이 n회 원시 평균 → 교정처럼 정밀 스냅샷이 필요할 때 사용 (드립팩터 교정 시작/종료 무게 측정)

---

## 2. 드립 팩터(drip factor) 자동 교정

### 배경

`drip factor(f)` = 수액 1방울(gtt)의 질량 `[g/gtt]`

| IV 세트 | 이론값 (g/gtt) | 용도 |
|---------|---------------|------|
| 20 gtt/mL | 0.0500 | 성인 일반 |
| 15 gtt/mL | 0.0667 | |
| 10 gtt/mL | 0.1000 | |
| 60 gtt/mL | 0.0167 | 소아 마이크로드립 |

유속 환산: `v[g/s] = (목표유속[gtt/min] / 60) × f`

> **문제** : 고정값(0.05) 사용 시 세트 종류가 다르면 목표 유속 오산출, 이상 감지 오경보 발생.

### 자동 교정 알고리즘

목표 유속으로 수액을 60초간 흘린 뒤 무게 감소량(ΔW) 측정 →

```
drip_factor = ΔW[g] / N_drops     (N_drops = 목표 gtt/min × 1min)
```

- 세트 종류에 무관하게 실제 흐름에서 직접 측정 → 이론값 오차(방울 크기 편차, 점도 차이)까지 자동 반영
- "목표 gtt/min" 1회 입력만으로 어떤 세트든 교정 완료

### 수액팩 초기 무게가 정확도에 영향 없는 이유

유속 계산은 `v(t) = [W(t-1) - W(t)] / dt`로, 연속된 두 측정치의 **차분(상대 변화율)** 으로 산출된다. 즉 수액팩 절대 무게(500/250mL 등)는 식에 포함되지 않으며, 영점(tare) 이후 상대 무게 변화만 사용하므로 시작 충전량과 무관하다.

> 단, 영점은 폴대/행거 등 부속 포함 상태에서 수행해야 정확.

---

## 3. 자동 부팅 절차 (상태 머신)

> **실무 요구** : 병동에서 시리얼 입력 없이 전원만 켜면 전 과정 자동 진행.

| 상태 (IVPhase) | 동작 |
|----------------|------|
| `PHASE_BOOT` | 전원 ON → 2초 대기 |
| `PHASE_TARE` | 자동 영점 조정 |
| `PHASE_WAIT` | 수액 감지 대기 (무게 > `WEIGHT_HANG_G`) |
| `PHASE_STABILIZE` | 흔들림 안정화 (교정 직전) |
| `PHASE_CALIB` | 드립 팩터 60초 교정 |
| `PHASE_WARMUP` | EMA 8샘플 안정화 |
| `PHASE_MONITOR` | 주입 모니터링 (CNN 이상 감지) |
| `PHASE_DONE` | 주입 완료 / 수액 제거 시 자동 재시작 |

### 흔들림 안정화 단계 추가 배경

수액을 폴대에 걸 때 ~30초간 흔들려 초기 무게가 출렁임 → 그 상태로 교정 시작하면 드립팩터가 부정확.

**해결: 교정 직전 안정화 단계 삽입**

- 매 측정값을 ring buffer(최근 5개)에 저장
- 최근 5개의 (최대-최소) < 0.5g 이면 "안정" 판정 → 교정 시작
- 30초 타임아웃: 계속 흔들리면 강제 진입 (무한 대기 방지)
- 안정화 중 수액 제거되면 `WAIT`로 복귀

### 자동 감지 임계값

- `WEIGHT_HANG_G` : 수액 걸림 판정 (수액팩 무게 이상)
- `WEIGHT_REMOVE_G` : 수액 제거 판정 (이 미만이면 제거로 간주 → 재영점)
- `WEIGHT_MS` : 측정 주기 2초 (1초에서 조정)

---

## 4. CNN(TFLite Micro) 이상 탐지 모델 탑재

### 설계

- **입력** : 4×4 = 16샘플 윈도우 (측정 2초 주기 기준 약 32초 관측)
  - 각 칸: 단일 샘플 상태 `-1`(느림) / `0`(정상) / `+1`(빠름)
  - 단일 샘플 상태는 유속 편차 ±tolerance(기본 17%)로 인코딩
- **출력** : 3클래스 softmax `[P(slow), P(normal), P(fast)]`
- **구조** : `Conv2D(8,3×3) → MaxPool(2×2) → Reshape → Dense(16) → Dense(3)`
- **연산** : `CONV_2D / MAX_POOL_2D / RESHAPE / FULLY_CONNECTED / SOFTMAX`

### 왜 CNN인가 — 단순 임계값과의 차이

단일 샘플 임계값(Fallback)은 "한 순간이라도 벗어나면 이상" → 오경보. CNN은 16샘플 시간 패턴을 보고 **"잠깐 튐 = 정상, 지속 이탈 = 이상"** 을 학습 → 시간적 맥락 기반 판정이 핵심 가치.

### Fallback 모드

모델이 없으면(`cnn_model.h` 비어있음) `+1`/`-1` 픽셀 비율로 판정하는 Fallback으로 자동 동작. 모델 학습 전에도 시스템이 돌아가게 보장.

### 모델이 계속 Fallback으로 떨어지던 문제 (디버깅 기록)

**증상** : `cnn_model.h`가 포함(`CNN_MODEL_AVAILABLE` 정의)됐는데도 추론 엔진이 Fallback. 즉 `begin()`의 `AllocateTensors()` 실패.

**원인** : 모델에 TFLite Micro resolver(5개 op)에 없는 op 3개 포함 — `SHAPE` / `STRIDED_SLICE` / `PACK`

1. Keras `Flatten` 레이어가 동적 shape 처리용으로 생성
2. 입력 batch 차원이 `None`(동적)이라 `Reshape`도 같은 op 생성

**해결**

- `Flatten` → 고정 크기 `Reshape((32,))`로 교체
- `Input(batch_size=1)`로 batch 고정 → 그래프 전체 정적 shape
- `Optimize.DEFAULT`(양자화) 제거 → 순수 float32 (Quantize op 회피)
- 결과 op가 resolver 5개와 정확히 일치 → TFLite 정상 활성화

### 진짜 TFLite 추론인지 검증 — 셀프 테스트

`cnntest` 명령: 정답을 아는 패턴(느림/정상/빠름)을 모델에 직접 주입.

- Fallback이면 확률이 픽셀비율(1/16 = 0.0625 배수)로만 나옴
- TFLite면 softmax 임의 실수 (예: `0.9987`)

→ 확률값 형태로 엔진 구별. 학습 데이터가 명확해 1.0/0.0 포화 출력(overconfident)이지만 비율계산으론 불가능한 값 → TFLite 확정.

---

## 5. BLE + WiFi 프로비저닝 (가장 난항)

### 목표

모바일 Flutter 앱에서 BLE로 기기 검색 → WiFi 스캔 → SSID 선택 → 비번 입력 → ESP32가 WiFi 연결 + NVS 저장. 이후 백엔드와 MQTT 연동.

### 1차 시도: WiFiProv 라이브러리 — 실패

- Espressif 전용 프로비저닝 프로토콜. ESP BLE Prov 앱 강제.
- 일반 폰 블루투스 / BLE 스캐너에 안 뜸.
- **핵심 함정** : `beginProvision()`은 NVS에 자격증명이 남아있으면 연결 실패 상황에서도 BLE 광고를 안 켜고 WiFi 재시도만 함.
- 8번째 인자 `reset_provisioned=true`로 강제했지만 여전히 한계.
- → **WiFiProv 폐기 결정.**

### 2차: 커스텀 BLE GATT 서버 (`ble_wifi_prov.h`) — 채택

표준 BLE GATT만 사용 → `flutter_blue_plus` 등 어떤 라이브러리든 호환.

**Service UUID** : `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

| Characteristic | UUID | 역할 |
|----------------|------|------|
| WiFi 스캔 | `a3c87500-...` | Write 트리거 + Notify 결과 |
| 자격증명 | `a3c87501-...` | Write `{"ssid","pw"}` |
| 상태 | `a3c87502-...` | Notify `{state, ssid, ip, deviceId}` |

- Preferences(NVS)로 자격증명 자동 저장/로드.
- 정책: BLE 항상 광고 → WiFi 연결 후에도 언제든 재설정 가능.

### BLE 디버깅 여정 — 연속 난관과 해결

<details>
<summary><b>(a) 폰에서 BLE 아예 안 보임</b></summary>

- 기기 ID가 `IVPOLE_000000` (MAC 전부 0)
- 원인: `WiFi.macAddress()`는 WiFi 스택 초기화 후에만 동작. `setup()` 초반 호출 시 0 반환.
- 해결: `ESP.getEfuseMac()`으로 eFuse 직접 읽기. MAC 뒤 3바이트(기기 고유)만 사용 (앞 3바이트는 제조사 OUI 공통)
</details>

<details>
<summary><b>(b) PC에선 보이는데 폰에서만 안 보임</b></summary>

- BLE는 폰 일반 블루투스 설정엔 원래 안 뜸(정상). BLE 스캐너 필요.
- 광고 패킷에 이름 명시(`setName`) + 인터벌 단축(100~150ms)으로 폰 발견율 개선. 안드로이드는 위치 서비스 ON + 권한 필수.
</details>

<details>
<summary><b>(c) Flutter 앱 스캔 안 됨</b></summary>

- `flutter_blue_plus` 1.36 API: `device.platformName`은 OS 캐시 기준이라 처음 보는 기기엔 빈 문자열. → `advertisementData.advName` 우선 사용.
- `scanResults` → `onScanResults` 권장 스트림으로 변경.
</details>

<details>
<summary><b>(d) Android "scanning too frequently" 차단</b></summary>

- 30초 내 5회 이상 스캔 시작 시 OS가 BLE 스캔 차단.
- 불필요한 `stopScan`/`startScan` 반복 제거, 자동 재스캔 제거.
</details>

<details>
<summary><b>(e) 권한 거부로 스캔 0개</b></summary>

- Android 12+는 `BLUETOOTH_SCAN`/`CONNECT`만 필요(`neverForLocation`). 위치 권한을 필수로 요구하던 로직이 막고 있었음 → BLE 권한만 요구.
</details>

<details>
<summary><b>(f) WiFi 목록 JSON이 1바이트만 수신 (FormatException)</b></summary>

- BLE 기본 ATT MTU 23바이트(데이터 20). 큰 JSON이 잘림.
- MTU 협상(`requestMtu`/`setMTU`) 시도했으나 Bluedroid에서 불안정.
- **최종 해결: read 폐기, "chunked notify" 방식.** ESP32가 JSON을 20바이트 청크로 잘라 순차 notify, 끝에 `\n` 마커. Flutter가 청크 누적 후 마커에서 파싱. MTU 협상 무관하게 안정.
</details>

<details>
<summary><b>(g) JSON 앞에 0x01이 붙어 파싱 실패</b></summary>

- `flutter_blue_plus`의 `lastValueStream`은 write/read/notify 모두 발행. 스캔 트리거로 보낸 `write([0x01])`가 stream에 들어와 버퍼 오염.
- `onValueReceived`(notify 전용) 사용 + `0x01` 단일패킷 무시.
</details>

<details>
<summary><b>(h) 끝 마커 \n이 데이터에 섞임</b></summary>

- 마지막 청크 + `\n`이 한 notify 패킷으로 묶여 옴.
- 마지막 바이트가 `\n`이면 끝 마커로 인식 + trailing 정리.
</details>

> **결과** : 위 8단계를 거쳐 BLE WiFi 프로비저닝 완전 동작. 실험실 AP 51~52개 스캔 → 선택 → 연결 → NVS 저장 확인.

---

## 6. 메모리 관리 (CNN 탑재로 인한 세밀 조정)

### 문제

CNN(TFLite) 모델을 실제로 탑재한 직후 WiFi 스캔이 0개(빈 `[]`)로 반환. 이전(Fallback, arena 미사용)엔 52개 잡히던 것이 안 됨.

### 원인 분석

ESP32 RAM(~320KB) 위에서 동시 점유:

```
BLE 스택 ~50KB + WiFi 스택 ~40KB + TFLite arena + 기타
```

`TENSOR_ARENA_KB`를 24KB로 잡았더니 `WiFi.scanNetworks()`의 스캔 버퍼 할당이 실패 → 0개 반환.

### 해결

- TFLite arena **24KB → 10KB** 축소 (4×4 소형 모델 실사용 arena는 5~8KB라 10KB면 충분)
- 확보 후 free heap ≈ 32KB → WiFi 스캔 정상 (실측 확인)
- WiFi 스캔 전후 free heap 시리얼 출력으로 메모리 진단 로그 추가

### 교훈 / 향후 여유 확보책

- free heap 32KB는 빠듯한 편. 측정 중 랜덤 리부팅 시: WiFi 연결 완료 후 BLE 광고 중단(필요 시 BOOT 버튼 재시작) = BLE 스택 ~50KB 회수 가능
- `cnn` 진단 명령의 `Arena 사용 X/10240 bytes`로 실사용량 모니터링

---

## 7. 알림 정책 / MQTT

### 이상 알림 통합

의료진 관점에서 "빠름/느림" 구분은 즉시 조치에 불필요 → 사용자 표시는 **"수액 이상 발생"** 단일 메시지로 통합. 단, MQTT JSON에는 `detail`(`"fast"`/`"slow"`) 필드 유지 → 백엔드 분석용.

```json
{"type":"IV_ANOMALY","detail":"fast","flowMeasured":0.06,"flowTarget":0.05}
```

### MQTT 토픽 (기기 ID 기반 격리)

| 토픽 | 방향 | 내용 |
|------|------|------|
| `iv_pole/<deviceId>/status` | 발행 | 무게/유속/예상종료 (5초) |
| `iv_pole/<deviceId>/alert` | 발행 | 이상/완료 |
| `iv_pole/<deviceId>/config` | 구독 | 목표유속/종료무게 |
| `iv_pole/<deviceId>/cmd` | 구독 | tare/reset |
| `iv_pole/<deviceId>/info` | 발행 | 온·오프라인 (LWT, retained) |

> **LWT(Last Will)** : 비정상 종료 시 브로커가 offline 자동 발행

### MQTT 비활성화 옵션

브로커 없이 개발 시 5초마다 연결 실패 로그 도배 → `MQTT_BROKER` 빈 문자열이면 시도 자체를 안 하도록 `mqttEnabled` 플래그.

---

## 8. 기기 식별 / 백엔드 연동 구조

- **기기 고유 ID** : `IVPOLE_XXXXXX` (MAC 뒤 3바이트). 다중 기기 충돌 없음.
- **BLE 광고명 = 기기 ID** → 앱이 QR 스캔으로 특정 기기 매칭
- **흐름** : 앱 BLE 연결 → WiFi 설정 → 앱이 기기 ID로 백엔드 등록 → 백엔드가 MQTT로 해당 기기 토픽에 명령 전달
- 펌웨어 버전 문자열(`FW_VERSION`) info 토픽에 포함

---

## 9. 실측 데이터 학습 파이프라인

### 배경

초기 모델은 합성 데이터(`train_cnn.py`)로 학습 → 파이프라인 검증용. "연속 블록=이상, 산발=정상" 규칙만 학습한 상태. 실제 임상 패턴 학습을 위해 실측 데이터 수집 체계 구축.

### 수집 — 세션 단위 ground-truth 자동 라벨

ESP32 시리얼 명령: `log normal` / `log slow` / `log fast` / `log off`

- 실험자가 물리적으로 상황을 만들고(예: 클램프 조여 느리게) 명령으로 라벨 지정 → 그 구간 데이터에 자동 라벨
- 출력: `LOG,<ms>,<flow>,<target>,<state>,<label>`

> ⚠️ 유속 편차 기반 자동 라벨은 의도적으로 배제. 그것은 `encodeState` 임계값 규칙을 CNN이 베끼게 만들어 Fallback과 차별성이 사라짐. **물리적 ground-truth가 정답.**

### 캡처 (PC)

`train/capture_serial.py` : pyserial로 `LOG` 줄만 추출하여 `data/*.csv` 자동 저장.

### 학습

`train/train_real.py` : `data/*.csv`를 16샘플 슬라이딩 윈도우로 자르고(다수결 라벨) `train_cnn.py`와 동일 구조로 학습 → `cnn_model.h` 생성. 모델 구조/op은 TFLite Micro 호환 형태 유지(`batch_size=1`, `Reshape`, float32).

### 전체 워크플로

```
log slow → (느린 상황 측정) → log off → train_real.py → ESP32 재컴파일
```

---

## 10. 향후 과제

- [ ] **drip factor 메모리 기능** : 교정 완료된 drip_factor를 세트 식별자와 함께 NVS/LittleFS에 저장 → 동일 세트 재사용 시 60초 교정 생략, 즉시 측정
- [ ] 실측 데이터 충분히 수집 후 CNN 재학습 (현재 합성→실측 전환)
- [ ] BLE 메모리 회수 정책 적용 검토 (연결 후 광고 중단)
- [ ] 막힘/누출 등 추가 이상 유형 분류 확장
