// NMX-1106 / NMX-1108 RS-232C 프로토콜 상수 및 디코더
//
// 프레임 구조 (6바이트 고정):
//   요청/제어 : AB CMD D1 D2 D3 CHK
//   응답      : AA CMD D1 D2 D3 CHK
//   CHK       = (0x54 - CMD - D1 - D2 - D3) & 0xFF
//
// 디코딩 근거는 NMX-1106 공개 프로토콜 문서(doc/NMX-1106_통신 프로토콜.txt).
// NMX-1108 확장분(CH7/CH8 등)은 미확인이므로 출력에 '?'를 붙여 구분한다.

#pragma once

#include <Arduino.h>

namespace nmx {

// ---------------------------------------------------------------- 통신 상수
constexpr uint32_t DEBUG_BAUD = 115200;  // PC 디버깅용 UART0
constexpr uint32_t NMX_BAUD = 9600;      // NMX-1106 문서 기준: 9600bps, 1 stop

constexpr uint8_t HEADER_COMMAND = 0xAB;   // 컨트롤러 → 본체
constexpr uint8_t HEADER_RESPONSE = 0xAA;  // 본체 → 컨트롤러
constexpr size_t FRAME_LEN = 6;
constexpr uint8_t CHECKSUM_BASE = 0x54;

// ---------------------------------------------------------------- CMD 코드
enum : uint8_t {
    CMD_SETUP = 0x01,           // SETUP key
    CMD_AUDIO_EXT = 0x02,       // Audio Select EXT
    CMD_EFFECT = 0x05,          // Effect Number / Effect INV
    CMD_FADER_MODE = 0x06,      // Fader Mode CUT/TAKE
    CMD_FADER_VALUE = 0x07,     // Manual Fader value
    CMD_PGM_OVERLAY = 0x08,     // PGM overlay PIP/DSK/CKEY
    CMD_PGM_CH = 0x09,          // PGM CH
    CMD_NEXT_CH = 0x0B,         // NEXT/PVW CH
    CMD_DEFAULT = 0xD0,         // Default
    CMD_STATUS_PGM_PVW = 0xD2,  // PGM/PVW status
    CMD_STATUS_EFFECT = 0xD3,   // EFFECT/CKEY/PIP/DSK/AUDIO status
};

// D2 응답의 채널 비트마스크 (BYTE_3 = PGM, BYTE_4 = PVW)
constexpr uint8_t CH_MASK_OFF = 0x40;  // NMX-1106 기준 OFF. 1108에서는 CH7일 가능성 있음

// D3 응답 BYTE_3: EFFECT 비트
constexpr uint8_t D3_EFFECT_INV = 0x01;  // bit0

// D3 응답 BYTE_4: 오버레이/오디오 비트
constexpr uint8_t D3_EFFECT_8 = 0x01;   // bit0
constexpr uint8_t D3_CG = 0x02;         // bit1
constexpr uint8_t D3_PIP = 0x04;        // bit2
constexpr uint8_t D3_DSK = 0x08;        // bit3
constexpr uint8_t D3_EXT_AUDIO = 0x40;  // bit6

// ---------------------------------------------------------------- 헬퍼
inline bool isHeader(uint8_t b) {
    return b == HEADER_COMMAND || b == HEADER_RESPONSE;
}

inline uint8_t checksum(const uint8_t frame[FRAME_LEN]) {
    return static_cast<uint8_t>(CHECKSUM_BASE - frame[1] - frame[2] - frame[3] - frame[4]);
}

inline bool isChecksumOk(const uint8_t frame[FRAME_LEN]) {
    return checksum(frame) == frame[FRAME_LEN - 1];
}

// ---------------------------------------------------------------- 디코더
// 알려진 CMD인지 여부. stats의 unknown 집계에 사용한다.
bool isKnownCmd(uint8_t cmd);

// CMD의 짧은 이름. 알 수 없으면 nullptr.
const char* cmdName(uint8_t cmd);

// 6바이트 프레임을 사람이 읽는 설명 문자열로 변환한다.
// 알 수 없는 CMD여도 최소한 "UNKNOWN CMD 0xNN" 형태를 채운다.
void describeFrame(const uint8_t frame[FRAME_LEN], char* out, size_t outLen);

// D2 응답의 채널 비트마스크를 문자열로 변환한다. (예: "CH1", "OFF|CH7?", "CH1+CH3")
void formatChannelMask(uint8_t value, char* out, size_t outLen);

// 바이트열을 "AB 09 00 00 01 4A" 형태로 변환한다.
void formatHex(const uint8_t* data, size_t len, char* out, size_t outLen);

}  // namespace nmx
