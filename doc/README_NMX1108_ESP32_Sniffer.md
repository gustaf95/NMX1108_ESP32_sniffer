# NMX-1108 / NMX-1106 RS-232C 프로토콜 스니퍼 개발 요구사항

## 1. 목적

보유 장비인 **NTU NMX-1108 8채널 영상믹서**의 본체와 전용 컨트롤러 사이에 오가는 RS-232C 프로토콜을 관찰하여, 향후 PC 또는 ESP32 기반 대체 컨트롤러로 제어할 수 있도록 한다.

현재 NMX-1108의 공식 프로토콜 자료는 없으나, 6채널 모델인 **NMX-1106 프로토콜 문서**가 확보되어 있다. 이 문서의 프레임 구조를 기준으로 NMX-1108에서 확장된 채널 7/8 관련 명령과 상태 응답을 분석한다.

본 프로젝트의 1차 목표는 **제어 신호를 가로채서 해석하는 수동 모니터링 장치**, 즉 RS-232C sniffer/logger를 만드는 것이다.

---

## 2. 개발 환경

- 개발 보드: ESP32 38-pin 개발보드
- 개발 IDE: Visual Studio Code
- 빌드 환경: PlatformIO
- Framework: Arduino
- USB Serial: `Serial` / UART0
- PC 디버깅용 포트: ESP32 기본 USB 포트, `Serial0`
- 본체/컨트롤러 감청용 UART:
  - `Serial1`: 컨트롤러 → 본체 방향 감청
  - `Serial2`: 본체 → 컨트롤러 방향 감청
- RS-232C 레벨 변환 IC: MAX3232 모듈 2개
- 통신 조건: NMX-1106 문서 기준 `9600 bps`, `1 stop bit`
- 데이터 비트/패리티: 문서에 명시 없음. 우선 `8N1`로 구현

---

## 3. 하드웨어 연결 개념

### 3.1 기본 구조

NMX 본체와 전용 컨트롤러 사이의 RS-232C 라인을 분리하거나 중간에서 브리지하지 않고, 우선은 **수동 감청 방식**으로 연결한다.

```text
[NMX 컨트롤러] -------- RS-232C -------- [NMX 본체]
       |                                  |
       |                                  |
   MAX3232 #1                         MAX3232 #2
       |                                  |
   ESP32 Serial1 RX                  ESP32 Serial2 RX
```

ESP32는 양쪽 방향의 신호를 각각 읽어서 PC의 USB 시리얼 모니터로 출력한다.

주의: 1차 버전에서는 ESP32가 본체와 컨트롤러 사이의 통신을 중계하지 않는다. 즉, TX를 이용해 신호를 주입하지 않고 RX 감청만 한다.

---

## 4. NMX-1106 핀어사인 참고

첨부된 NMX-1106 핀어사인 자료 기준 RJ45 RS-232C 포트는 다음과 같다.

| RJ45 핀 | RS-232C 신호 |
|---:|---|
| 4번 | GND |
| 5번 | RxD |
| 6번 | TxD |

주의: 이 표기는 장비 포트 기준일 가능성이 높다. 실제 감청 시에는 오실로스코프 또는 로직 분석기로 어느 라인이 송신 방향인지 확인한다.

---

## 5. MAX3232 연결 원칙

RS-232C는 ±전압 레벨을 사용하므로 ESP32 UART GPIO에 직접 연결하면 안 된다. 반드시 MAX3232를 통해 3.3V TTL UART 레벨로 변환한다.

### 5.1 MAX3232 #1

목적: 컨트롤러 → 본체 방향 데이터 감청

- RS-232C 입력: 컨트롤러가 송신하는 TX 라인에 병렬 연결
- TTL 출력: ESP32 `Serial1 RX`에 연결
- GND: NMX 장비 GND, MAX3232 GND, ESP32 GND 공통 연결

### 5.2 MAX3232 #2

목적: 본체 → 컨트롤러 방향 데이터 감청

- RS-232C 입력: 본체가 송신하는 TX 라인에 병렬 연결
- TTL 출력: ESP32 `Serial2 RX`에 연결
- GND: NMX 장비 GND, MAX3232 GND, ESP32 GND 공통 연결

### 5.3 ESP32 UART 핀 예시

아래 핀은 예시이며, 실제 보드에서 사용 가능한 핀으로 조정 가능하게 코드 상단에서 `#define` 또는 `constexpr`로 분리한다.

| 용도 | ESP32 UART | 예시 GPIO |
|---|---|---:|
| PC 디버깅 | Serial / UART0 | USB 기본 |
| 컨트롤러 → 본체 감청 RX | Serial1 RX | GPIO 16 |
| 컨트롤러 → 본체 감청 TX | Serial1 TX | GPIO 17, 미사용 가능 |
| 본체 → 컨트롤러 감청 RX | Serial2 RX | GPIO 26 |
| 본체 → 컨트롤러 감청 TX | Serial2 TX | GPIO 27, 미사용 가능 |

TX 핀은 1차 감청 모드에서는 사용하지 않는다. 다만 Arduino `HardwareSerial.begin()` 호출을 위해 핀 번호를 지정할 수는 있다.

---

## 6. 프로토콜 요약

NMX-1106 문서 기준 프레임은 6바이트 고정 길이로 보인다.

### 6.1 제어/요청 프레임

```text
AB CMD D1 D2 D3 CHK
```

예:

```text
AB 09 00 00 01 4A  // PGM CH CAM1
AB 0B 00 00 02 47  // NEXT CH CAM2
AB D2 00 00 01 81  // PGM/PVW 상태 요청
AB D3 00 00 01 80  // EFFECT/CKEY/PIP/DSK/AUDIO 상태 요청
```

### 6.2 응답 프레임

```text
AA CMD D1 D2 D3 CHK
```

예:

```text
AA D2 55 01 02 2A  // PGM CH1, PVW CH2
AA D3 55 xx xx 80  // EFFECT/CKEY/PIP/DSK/AUDIO 상태
```

### 6.3 체크섬 추정식

문서의 예제와 대조하면 체크섬은 다음과 같이 추정된다.

```cpp
uint8_t checksum(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3) {
    return (uint8_t)(0x54 - cmd - d1 - d2 - d3);
}
```

프레임 전체 기준으로는 header를 제외한 2~5번째 바이트를 사용한다.

```text
CHK = (0x54 - CMD - D1 - D2 - D3) & 0xFF
```

---

## 7. 주요 NMX-1106 명령표

### 7.1 PGM 채널 선택

```text
AB 09 00 00 00 4B  PGM CH OFF
AB 09 00 00 01 4A  PGM CH CAM1
AB 09 00 00 02 49  PGM CH CAM2
AB 09 00 00 03 48  PGM CH CAM3
AB 09 00 00 04 47  PGM CH CAM4
AB 09 00 00 05 46  PGM CH CAM5
AB 09 00 00 06 45  PGM CH CAM6
```

NMX-1108에서는 다음 확장 가능성을 실측으로 확인한다.

```text
AB 09 00 00 07 44  PGM CH CAM7 추정
AB 09 00 00 08 43  PGM CH CAM8 추정
```

### 7.2 NEXT/PVW 채널 선택

```text
AB 0B 00 00 00 49  NEXT CH OFF
AB 0B 00 00 01 48  NEXT CH CAM1
AB 0B 00 00 02 47  NEXT CH CAM2
AB 0B 00 00 03 46  NEXT CH CAM3
AB 0B 00 00 04 45  NEXT CH CAM4
AB 0B 00 00 05 44  NEXT CH CAM5
AB 0B 00 00 06 43  NEXT CH CAM6
```

NMX-1108에서는 다음 확장 가능성을 실측으로 확인한다.

```text
AB 0B 00 00 07 42  NEXT CH CAM7 추정
AB 0B 00 00 08 41  NEXT CH CAM8 추정
```

### 7.3 상태 조회

```text
AB D2 00 00 01 81  // PGM CH, PVW CH 상태 요청
AB D3 00 00 01 80  // EFFECT, CKEY/PIP/DSK/AUDIO 상태 요청
```

D2 응답 예:

```text
AA D2 55 01 02 2A
```

의미:

```text
PGM CH = 1CH
PVW CH = 2CH
```

NMX-1106의 D2 응답 비트 정의:

| 값 | 의미 |
|---:|---|
| 0x40 | OFF |
| 0x01 | CH1 |
| 0x02 | CH2 |
| 0x04 | CH3 |
| 0x08 | CH4 |
| 0x10 | CH5 |
| 0x20 | CH6 |

NMX-1108에서는 CH7/CH8이 `0x40`, `0x80`인지, 또는 OFF 코드가 변경되는지 반드시 실측해야 한다.

---

## 8. 펌웨어 기능 요구사항

### 8.1 기본 기능

1. ESP32 부팅 시 PC 시리얼 모니터에 시작 메시지를 출력한다.
2. UART0/Serial은 PC 디버깅 전용으로 사용한다.
3. Serial1은 컨트롤러 → 본체 방향 감청용으로 사용한다.
4. Serial2는 본체 → 컨트롤러 방향 감청용으로 사용한다.
5. 두 UART 모두 `9600, 8N1`로 초기화한다.
6. 각 UART에서 수신된 바이트를 실시간으로 버퍼링한다.
7. `0xAB` 또는 `0xAA`를 프레임 시작 바이트로 보고 6바이트 프레임을 우선 파싱한다.
8. 완성된 프레임은 PC Serial로 사람이 읽기 쉬운 형태로 출력한다.
9. 체크섬을 계산하여 `OK` 또는 `BAD`를 표시한다.
10. 알려진 명령은 간단한 의미를 함께 출력한다.
11. 알 수 없는 명령도 원시 HEX 프레임은 반드시 출력한다.

---

## 9. 출력 형식 요구사항

시리얼 모니터 출력은 분석하기 쉽게 한 줄에 한 프레임씩 출력한다.

예:

```text
[CTRL->MAIN] 12:34:56.789  AB 09 00 00 01 4A  CHK=OK  PGM CH 1
[CTRL->MAIN] 12:34:57.102  AB 0B 00 00 02 47  CHK=OK  NEXT CH 2
[MAIN->CTRL] 12:34:57.210  AA D2 55 01 02 2A  CHK=OK  STATUS PGM=CH1 PVW=CH2
[MAIN->CTRL] 12:34:57.300  AA D3 55 02 04 7C  CHK=BAD  STATUS RAW
```

타임스탬프는 `millis()` 기반이어도 된다. 가능하면 `HH:MM:SS.mmm` 형태로 포맷한다.

---

## 10. 파서 설계 요구사항

### 10.1 1차 파서

- `0xAB` 또는 `0xAA`가 들어올 때 프레임 수집을 시작한다.
- 이후 5바이트를 더 받아 총 6바이트가 되면 프레임으로 처리한다.
- 프레임 길이는 우선 6바이트 고정으로 가정한다.
- 중간에 새로운 `0xAB` 또는 `0xAA`가 들어오면 동기 재획득을 고려한다.
- 잘못된 체크섬이어도 버리지 말고 출력한다.

### 10.2 Raw dump 모드

프로토콜이 예상과 다를 경우를 대비해, 모든 수신 바이트를 raw hex로 출력하는 모드를 제공한다.

시리얼 명령 예:

```text
raw on
raw off
```

### 10.3 프레임 파싱 모드

기본 모드는 frame parsing 모드이다.

시리얼 명령 예:

```text
frame on
frame off
```

---

## 11. PC에서 보내는 디버그 명령 요구사항

PC 시리얼 모니터에서 간단한 명령을 입력하면 동작 모드를 바꿀 수 있게 한다.

최소 구현 명령:

| 명령 | 기능 |
|---|---|
| `help` | 사용 가능한 명령 출력 |
| `raw on` | raw byte 출력 활성화 |
| `raw off` | raw byte 출력 비활성화 |
| `frame on` | frame parser 출력 활성화 |
| `frame off` | frame parser 출력 비활성화 |
| `stats` | 수신 프레임 수, 체크섬 오류 수 출력 |
| `clear` | 통계 초기화 |

선택 구현 명령:

| 명령 | 기능 |
|---|---|
| `inject off` | 기본값. 절대 송신하지 않음 |
| `send1 AB 09 00 00 01 4A` | Serial1 TX로 송신. 2차 실험용 |
| `send2 AB D2 00 00 01 81` | Serial2 TX로 송신. 2차 실험용 |

주의: 1차 버전에서는 안전을 위해 송신 기능을 구현하지 않거나, 컴파일 타임 매크로 `ENABLE_INJECTION`이 켜졌을 때만 활성화한다.

---

## 12. PlatformIO 프로젝트 구조 요구사항

권장 구조:

```text
NMX1108_ESP32_Sniffer/
├── platformio.ini
├── README.md
├── src/
│   └── main.cpp
└── include/
    └── nmx_protocol.h
```

### 12.1 platformio.ini 예시

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_speed = 921600
```

필요하면 실제 보드에 맞춰 `board`를 변경한다.

---

## 13. 코드 구현 세부 요구사항

### 13.1 UART 초기화

```cpp
Serial.begin(115200);
Serial1.begin(9600, SERIAL_8N1, RX1_PIN, TX1_PIN);
Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
```

### 13.2 주요 상수

```cpp
constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t NMX_BAUD = 9600;
constexpr uint8_t HEADER_COMMAND = 0xAB;
constexpr uint8_t HEADER_RESPONSE = 0xAA;
constexpr size_t FRAME_LEN = 6;
```

### 13.3 체크섬 함수

```cpp
uint8_t nmxChecksum(const uint8_t frame[6]) {
    return static_cast<uint8_t>(0x54 - frame[1] - frame[2] - frame[3] - frame[4]);
}

bool isChecksumOk(const uint8_t frame[6]) {
    return nmxChecksum(frame) == frame[5];
}
```

### 13.4 채널 명령 디코딩

최소한 다음 명령은 디코딩한다.

| CMD | 의미 |
|---:|---|
| 0x01 | SETUP key |
| 0x02 | Audio Select EXT |
| 0x05 | Effect Number / Effect INV |
| 0x06 | Fader Mode CUT/TAKE |
| 0x07 | Manual Fader value |
| 0x08 | PGM overlay PIP/DSK/CKEY |
| 0x09 | PGM CH |
| 0x0B | NEXT/PVW CH |
| 0xD0 | Default |
| 0xD2 | PGM/PVW status request/response |
| 0xD3 | EFFECT/CKEY/PIP/DSK/AUDIO status request/response |

---

## 14. NMX-1108 분석 포인트

NMX-1108에서 반드시 확인해야 할 항목:

1. PGM CH7 버튼을 눌렀을 때 나오는 프레임
2. PGM CH8 버튼을 눌렀을 때 나오는 프레임
3. NEXT/PVW CH7 버튼을 눌렀을 때 나오는 프레임
4. NEXT/PVW CH8 버튼을 눌렀을 때 나오는 프레임
5. TAKE/CUT/FADER 동작 시 프레임이 NMX-1106과 같은지 여부
6. `AB D2 00 00 01 81` 요청에 대해 본체가 응답하는지 여부
7. D2 응답에서 CH7/CH8이 어떤 비트 또는 값으로 표현되는지
8. OFF 값이 NMX-1106처럼 `0x40`인지, 8채널 확장 때문에 달라졌는지
9. D3 응답의 EFFECT/PIP/DSK/AUDIO 비트 구조가 같은지
10. 컨트롤러가 주기적으로 상태 조회를 보내는지 여부

---

## 15. 안전 요구사항

1. ESP32 GPIO를 RS-232C 라인에 직접 연결하지 않는다.
2. MAX3232를 반드시 사용한다.
3. GND는 장비, MAX3232, ESP32 사이에 공통 연결한다.
4. 1차 실험에서는 ESP32 TX를 장비 라인에 연결하지 않는다.
5. 감청 중에는 ESP32가 본체/컨트롤러 통신을 방해하지 않도록 고임피던스 수신 형태로 연결한다.
6. 송신 실험은 감청 로그로 프로토콜을 확인한 뒤 별도 단계에서 진행한다.
7. 송신 기능은 기본 비활성화한다.
8. NMX 본체와 컨트롤러의 RxD/TxD 표기 방향이 혼동될 수 있으므로, 실제 연결 전에 멀티미터/오실로스코프/로직애널라이저로 방향을 확인한다.

---

## 16. 1차 구현 완료 기준

다음 조건을 만족하면 1차 구현 완료로 본다.

1. PlatformIO에서 빌드 및 업로드가 성공한다.
2. ESP32 USB Serial Monitor에 시작 메시지가 출력된다.
3. 컨트롤러 버튼을 누르면 `CTRL->MAIN` 방향 프레임이 표시된다.
4. 본체가 응답하면 `MAIN->CTRL` 방향 프레임이 표시된다.
5. 6바이트 프레임이 HEX로 표시된다.
6. 체크섬 OK/BAD가 표시된다.
7. PGM CH, NEXT CH, D2, D3는 의미가 디코딩된다.
8. raw 모드와 frame 모드를 PC 명령으로 켜고 끌 수 있다.
9. `stats` 명령으로 방향별 프레임 수와 체크섬 오류 수가 표시된다.

---

## 17. 2차 확장 아이디어

1차 감청이 안정화된 후 다음 기능을 추가한다.

1. CSV 로그 출력
2. 버튼 이벤트별 자동 라벨링
3. NMX-1108 CH7/CH8 명령 자동 추정
4. PC에서 명령을 입력해 본체 직접 제어
5. ESP32가 컨트롤러를 대체하는 standalone controller 모드
6. Wi-Fi 웹 UI를 통한 영상믹서 제어
7. WebSocket 기반 실시간 상태 모니터링
8. SD 카드 로그 저장

---

## 18. Claude Code에게 요청할 작업

Claude Code는 위 요구사항을 기준으로 다음을 수행한다.

1. PlatformIO Arduino 프로젝트를 생성한다.
2. `platformio.ini`를 작성한다.
3. `src/main.cpp`에 ESP32 UART sniffer 펌웨어를 구현한다.
4. 필요한 경우 `include/nmx_protocol.h`에 프로토콜 관련 상수와 함수 선언을 분리한다.
5. 빌드 가능한 상태로 코드를 작성한다.
6. 코드 상단에 UART 핀 설정을 쉽게 바꿀 수 있도록 한다.
7. 기본 모드는 수동 감청 전용으로 하며, 송신 기능은 구현하지 않거나 `ENABLE_INJECTION` 매크로가 켜져 있을 때만 활성화한다.
8. README의 명령표와 출력 예시를 코드 동작에 반영한다.

---

## 19. 참고: NMX-1106 공개 프로토콜 원문 중 핵심 예시

```text
9600BPS,1STOP

AB 09 00 00 01 4A  PGM CH (CAM1)
AB 09 00 00 02 49  PGM CH (CAM2)
AB 09 00 00 03 48  PGM CH (CAM3)
AB 09 00 00 04 47  PGM CH (CAM4)
AB 09 00 00 05 46  PGM CH (CAM5)
AB 09 00 00 06 45  PGM CH (CAM6)

AB 0B 00 00 01 48  NEXT CH (CAM1)
AB 0B 00 00 02 47  NEXT CH (CAM2)
AB 0B 00 00 03 46  NEXT CH (CAM3)
AB 0B 00 00 04 45  NEXT CH (CAM4)
AB 0B 00 00 05 44  NEXT CH (CAM5)
AB 0B 00 00 06 43  NEXT CH (CAM6)

AB D2 00 00 01 81  // PGM CH, PVW CH 상태 요청
AA D2 55 01 02 2A  // PGM CH=1, PVW CH=2 응답 예

AB D3 00 00 01 80  // EFFECT, CKEY/PIP/DSK/AUDIO 상태 요청
AA D3 55 xx xx 80  // 상태 응답 예
```
