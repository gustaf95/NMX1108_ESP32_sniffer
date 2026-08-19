// NMX-1108 / NMX-1106 RS-232C 패시브 탭 스니퍼 (ESP32 / Arduino / PlatformIO)
//
// 컨트롤러-본체 케이블은 그대로 둔 채, 두 신호선에 MAX3232 리시버를 병렬로 얹어
// 오가는 프레임을 엿듣고 해석해 PC로 기록한다.
//
//   [NMX 컨트롤러] ═══════ 원래 케이블 (유지) ═══════ [NMX 본체]
//                      │                      │
//                    R1IN                   R2IN
//                   ┌──────── MAX3232 ────────┐
//                   │ R1OUT             R2OUT │
//                   └───┬──────────────────┬──┘
//                    GPIO16             GPIO26
//                   (Serial1 RX)      (Serial2 RX)
//
// ESP32에서 바깥으로 나가는 선은 없다. TX 핀은 UART에 할당조차 하지 않으므로
// (begin()의 txPin 인자에 -1) 이 펌웨어는 하드웨어적으로 송신할 수 없다.
// 따라서 ESP32가 꺼지거나 리셋돼도 컨트롤러-본체 통신은 영향을 받지 않는다.
//
// 배선만으로는 어느 포트가 컨트롤러 송신인지 알 수 없으므로, 프로토콜 헤더로 판정한다.
// 요청은 0xAB, 응답은 0xAA 로 시작하므로 0xAB 가 우세한 포트가 컨트롤러측이다.
// 확정 전에는 로그에 [P1?] / [P2?] 로 찍힌다.
//
// 탭은 놓친 바이트를 되돌려 받을 방법이 없다. RX 버퍼 포화를 OVERFLOW 로 남기는 이유다.

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "nmx_protocol.h"
#include "nmx_selftest.h"

// ---------------------------------------------------------------- 핀 설정
// build_flags 에서 -DTAP1_RX_PIN=... 형태로 덮어쓸 수 있다.
// TX 핀은 존재하지 않는다. 패시브 탭이므로 어떤 핀도 출력으로 잡지 않는다.
#ifndef TAP1_RX_PIN
#define TAP1_RX_PIN 16  // Serial1 RX  <- MAX3232 R1OUT
#endif
#ifndef TAP2_RX_PIN
#define TAP2_RX_PIN 26  // Serial2 RX  <- MAX3232 R2OUT
#endif

// ---------------------------------------------------------------- 동작 상수
namespace {

constexpr char FIRMWARE_VERSION[] = "2.1.0-tap";

// 미완성 프레임을 접고 다시 동기화하기까지의 유휴 시간.
// 9600bps에서 1바이트는 약 1.04ms이므로 프레임 내 간격보다 충분히 길다.
constexpr uint32_t FRAME_TIMEOUT_MS = 50;

// raw 모드에서 한 줄을 끊는 유휴 시간 / 최대 바이트 수.
constexpr uint32_t RAW_GAP_MS = 8;
constexpr size_t RAW_CHUNK = 16;

// 헤더(0xAB/0xAA) 이전에 흘러 들어온 바이트를 모아 두는 버퍼.
constexpr size_t SKIP_MAX = 16;

constexpr size_t CMD_LINE_MAX = 96;
constexpr size_t DESC_MAX = 160;
constexpr size_t HEX_MAX = 3 * RAW_CHUNK + 1;
constexpr size_t LOG_LINE_MAX = 256;

// 장비 UART는 넉넉한 RX 버퍼를 잡고, PC 로그는 큰 TX 버퍼로 순간 폭주를 흡수한다.
constexpr size_t NMX_RX_BUFFER = 512;
constexpr size_t LOG_TX_BUFFER = 4096;

// RX 버퍼가 이만큼 차 있으면 포화로 본다. 드라이버 링버퍼가 가득 차면
// 그 뒤에 도착한 바이트는 조용히 버려지므로, 그 직전 상태를 유실 신호로 삼는다.
constexpr size_t RX_HIGH_WATER = NMX_RX_BUFFER - 32;

// 방향 자동 판정: 체크섬 OK 프레임이 이만큼 모이고 한쪽 헤더가 이 비율 이상이면 확정.
constexpr uint32_t DIR_MIN_FRAMES = 20;
constexpr uint32_t DIR_RATIO_PCT = 80;

// scan 기본 지속시간 (후보당)
constexpr uint32_t SCAN_DEFAULT_SEC = 3;
constexpr uint32_t SCAN_MAX_SEC = 30;

// ---------------------------------------------------------------- 보레이트 / 프레임 형식

constexpr uint32_t BAUD_TABLE[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};

struct FormatEntry {
    const char* name;
    uint32_t cfg;
};
const FormatEntry FORMAT_TABLE[] = {
    {"8N1", SERIAL_8N1}, {"8E1", SERIAL_8E1}, {"8O1", SERIAL_8O1},
    {"7E1", SERIAL_7E1}, {"7O1", SERIAL_7O1}, {"8N2", SERIAL_8N2},
};

const char* formatName(uint32_t cfg) {
    for (const FormatEntry& f : FORMAT_TABLE) {
        if (f.cfg == cfg) {
            return f.name;
        }
    }
    return "?";
}

// ---------------------------------------------------------------- 상태
struct Stats {
    uint32_t bytes;
    uint32_t overflow;  // RX 버퍼 포화 감지 횟수. 브리지의 relayed 자리를 대신한다.
    uint32_t frames;
    uint32_t chkOk;
    uint32_t chkBad;
    uint32_t unknownCmd;
    uint32_t resync;
    uint32_t incomplete;
    uint32_t skipped;
};

// 포트가 어느 방향을 듣고 있는지. 배선이 아니라 프로토콜로 판정한다.
enum class Role : uint8_t { Unknown, Ctrl, Main };

struct Port {
    HardwareSerial* uart;
    uint8_t index;  // 0 = port1, 1 = port2
    int8_t rxPin;
    uint32_t baud;
    uint32_t format;

    // 프레임 파서
    uint8_t frame[nmx::FRAME_LEN];
    size_t frameLen;
    uint32_t lastByteMs;

    // 헤더 이전 잡음 바이트
    uint8_t skip[SKIP_MAX];
    size_t skipLen;
    uint32_t skipFirstMs;

    // raw 덤프
    uint8_t raw[RAW_CHUNK];
    size_t rawLen;
    uint32_t rawFirstMs;

    // 방향 판정 근거 (체크섬 OK 프레임의 첫 바이트만 센다)
    uint32_t hdrCommand;   // 0xAB
    uint32_t hdrResponse;  // 0xAA

    bool rxSaturated;
    Stats stats;
};

Port g_port[2] = {
    {&Serial1, 0, TAP1_RX_PIN, nmx::NMX_BAUD, SERIAL_8N1},
    {&Serial2, 1, TAP2_RX_PIN, nmx::NMX_BAUD, SERIAL_8N1},
};

Role g_role[2] = {Role::Unknown, Role::Unknown};
bool g_dirAuto = true;  // 헤더 바이트로 자동 판정

bool g_frameEnabled = true;
bool g_rawEnabled = false;

uint32_t g_logDropped = 0;  // 수집 보호를 위해 버린 로그 줄 수

char g_cmdLine[CMD_LINE_MAX];
size_t g_cmdLen = 0;

// ---------------------------------------------------------------- 로그 출력
//
// USB 로그 때문에 수집이 밀리면 안 되므로, 출력 버퍼에 여유가 없으면
// 로그 줄을 통째로 버리고 카운트만 올린다. 바이트를 받는 일이 항상 우선이다.
void emitLine(const char* line) {
    const size_t len = strlen(line);
    const int room = Serial.availableForWrite();
    if (room < 0 || static_cast<size_t>(room) < len + 1) {
        g_logDropped++;
        return;
    }
    Serial.write(reinterpret_cast<const uint8_t*>(line), len);
    Serial.write('\n');
}

void formatTime(uint32_t ms, char* out, size_t outLen) {
    const uint32_t totalSec = ms / 1000;
    snprintf(out, outLen, "%02u:%02u:%02u.%03u", static_cast<unsigned>((totalSec / 3600) % 100),
             static_cast<unsigned>((totalSec / 60) % 60), static_cast<unsigned>(totalSec % 60),
             static_cast<unsigned>(ms % 1000));
}

// 방향 태그. 확정 전에는 포트 번호를 그대로 보여준다. 항상 3글자라
// 줄 맨 앞에서 세로로 정렬되고 grep "\[C>M\]" 로 한 방향만 걸러낼 수 있다.
const char* portTag(const Port& p) {
    switch (g_role[p.index]) {
        case Role::Ctrl:
            return "C>M";
        case Role::Main:
            return "M>C";
        default:
            return (p.index == 0) ? "P1?" : "P2?";
    }
}

// "[C>M] 00:01:23.456  " 를 채우고 그 길이를 반환한다.
size_t lineHead(char* out, size_t outLen, const Port& p, uint32_t ms) {
    char ts[16];
    formatTime(ms, ts, sizeof(ts));
    const int n = snprintf(out, outLen, "[%s] %s  ", portTag(p), ts);
    return (n < 0) ? 0 : static_cast<size_t>(n);
}

// ---------------------------------------------------------------- 방향 자동 판정

const char* roleName(Role r) {
    switch (r) {
        case Role::Ctrl:
            return "CTRL";
        case Role::Main:
            return "MAIN";
        default:
            return "?";
    }
}

void assignRoles(uint8_t port, Role r) {
    g_role[port] = r;
    g_role[1 - port] = (r == Role::Ctrl) ? Role::Main : Role::Ctrl;
}

void announceRoles(const char* how) {
    char line[LOG_LINE_MAX];
    snprintf(line, sizeof(line), "-- direction %s: P1=%s, P2=%s --", how, roleName(g_role[0]),
             roleName(g_role[1]));
    emitLine(line);
}

// 체크섬 OK 프레임만 센다. 보레이트가 틀린 상태의 쓰레기 바이트에 낚이지 않기 위해서다.
void dirTryResolve() {
    if (!g_dirAuto || g_role[0] != Role::Unknown) {
        return;
    }
    for (const Port& p : g_port) {
        const uint32_t total = p.hdrCommand + p.hdrResponse;
        if (total < DIR_MIN_FRAMES) {
            continue;
        }
        Role r = Role::Unknown;
        if (static_cast<uint64_t>(p.hdrCommand) * 100 >= static_cast<uint64_t>(total) * DIR_RATIO_PCT) {
            r = Role::Ctrl;
        } else if (static_cast<uint64_t>(p.hdrResponse) * 100 >=
                   static_cast<uint64_t>(total) * DIR_RATIO_PCT) {
            r = Role::Main;
        }
        if (r == Role::Unknown) {
            continue;
        }
        assignRoles(p.index, r);
        announceRoles("resolved");
        return;
    }
}

void dirClearEvidence() {
    for (Port& p : g_port) {
        p.hdrCommand = 0;
        p.hdrResponse = 0;
    }
}

// ---------------------------------------------------------------- raw 모드

void rawFlush(Port& p) {
    if (p.rawLen == 0) {
        return;
    }
    char hex[HEX_MAX];
    char line[LOG_LINE_MAX];
    nmx::formatHex(p.raw, p.rawLen, hex, sizeof(hex));
    const size_t n = lineHead(line, sizeof(line), p, p.rawFirstMs);
    snprintf(line + n, sizeof(line) - n, "RAW  %s", hex);
    emitLine(line);
    p.rawLen = 0;
}

void rawFeed(Port& p, uint8_t b, uint32_t now) {
    if (p.rawLen == 0) {
        p.rawFirstMs = now;
    }
    p.raw[p.rawLen++] = b;
    if (p.rawLen >= RAW_CHUNK) {
        rawFlush(p);
    }
}

// ---------------------------------------------------------------- 프레임 파서

void skipFlush(Port& p) {
    if (p.skipLen == 0) {
        return;
    }
    if (g_frameEnabled) {
        char hex[HEX_MAX];
        char line[LOG_LINE_MAX];
        nmx::formatHex(p.skip, p.skipLen, hex, sizeof(hex));
        const size_t n = lineHead(line, sizeof(line), p, p.skipFirstMs);
        snprintf(line + n, sizeof(line) - n, "SKIP %u byte(s): %s",
                 static_cast<unsigned>(p.skipLen), hex);
        emitLine(line);
    }
    p.skipLen = 0;
}

void skipPush(Port& p, uint8_t b, uint32_t now) {
    if (p.skipLen == 0) {
        p.skipFirstMs = now;
    }
    if (p.skipLen < SKIP_MAX) {
        p.skip[p.skipLen++] = b;
    }
    p.stats.skipped++;
    if (p.skipLen >= SKIP_MAX) {
        skipFlush(p);
    }
}

void frameFeed(Port& p, uint8_t b, uint32_t now);

// 6바이트가 모였을 때 호출한다. 체크섬이 틀리면 프레임 내부의 헤더 바이트를
// 기준으로 동기를 다시 잡는다. (정상 프레임은 데이터에 0xAA/0xAB가 있어도 건드리지 않는다)
void frameComplete(Port& p, uint32_t now) {
    const bool ok = nmx::isChecksumOk(p.frame);

    p.stats.frames++;
    if (ok) {
        p.stats.chkOk++;
        // 방향 판정 근거는 체크섬이 맞은 프레임에서만 모은다.
        if (p.frame[0] == nmx::HEADER_COMMAND) {
            p.hdrCommand++;
        } else if (p.frame[0] == nmx::HEADER_RESPONSE) {
            p.hdrResponse++;
        }
    } else {
        p.stats.chkBad++;
    }
    if (!nmx::isKnownCmd(p.frame[1])) {
        p.stats.unknownCmd++;
    }

    if (g_frameEnabled) {
        char hex[HEX_MAX];
        char desc[DESC_MAX];
        char line[LOG_LINE_MAX];
        nmx::formatHex(p.frame, nmx::FRAME_LEN, hex, sizeof(hex));
        nmx::describeFrame(p.frame, desc, sizeof(desc));

        size_t n = lineHead(line, sizeof(line), p, now);
        n += snprintf(line + n, sizeof(line) - n, "%s  CHK=%-3s  %s", hex, ok ? "OK" : "BAD", desc);
        if (!ok && n < sizeof(line)) {
            snprintf(line + n, sizeof(line) - n, "  [expected CHK=%02X]",
                     static_cast<unsigned>(nmx::checksum(p.frame)));
        }
        emitLine(line);
    }

    p.frameLen = 0;

    if (ok) {
        dirTryResolve();
        return;
    }

    // 체크섬 불일치 → 프레임 중간의 헤더 바이트부터 재동기 시도
    size_t resyncAt = 0;
    for (size_t i = 1; i < nmx::FRAME_LEN; ++i) {
        if (nmx::isHeader(p.frame[i])) {
            resyncAt = i;
            break;
        }
    }
    if (resyncAt == 0) {
        return;
    }

    p.stats.resync++;
    if (g_frameEnabled) {
        char line[LOG_LINE_MAX];
        const size_t n = lineHead(line, sizeof(line), p, now);
        snprintf(line + n, sizeof(line) - n, "RESYNC from offset %u",
                 static_cast<unsigned>(resyncAt));
        emitLine(line);
    }

    uint8_t tail[nmx::FRAME_LEN];
    const size_t tailLen = nmx::FRAME_LEN - resyncAt;
    memcpy(tail, p.frame + resyncAt, tailLen);
    // tailLen <= 5 이므로 여기서 frameComplete()가 다시 불리는 일은 없다.
    for (size_t i = 0; i < tailLen; ++i) {
        frameFeed(p, tail[i], now);
    }
}

void frameFeed(Port& p, uint8_t b, uint32_t now) {
    p.lastByteMs = now;

    if (p.frameLen == 0) {
        if (!nmx::isHeader(b)) {
            skipPush(p, b, now);
            return;
        }
        skipFlush(p);
        p.frame[0] = b;
        p.frameLen = 1;
        return;
    }

    p.frame[p.frameLen++] = b;
    if (p.frameLen == nmx::FRAME_LEN) {
        frameComplete(p, now);
    }
}

// 프레임이 도중에 끊긴 채 유휴 상태가 되면 버리지 않고 그대로 출력한다.
void frameTimeout(Port& p, uint32_t now) {
    if (p.frameLen == 0) {
        return;
    }
    p.stats.incomplete++;
    if (g_frameEnabled) {
        char hex[HEX_MAX];
        char line[LOG_LINE_MAX];
        nmx::formatHex(p.frame, p.frameLen, hex, sizeof(hex));
        const size_t n = lineHead(line, sizeof(line), p, now);
        snprintf(line + n, sizeof(line) - n, "%s  INCOMPLETE (%u/%u bytes, timeout)", hex,
                 static_cast<unsigned>(p.frameLen), static_cast<unsigned>(nmx::FRAME_LEN));
        emitLine(line);
    }
    p.frameLen = 0;
}

void resetParser(Port& p) {
    p.frameLen = 0;
    p.skipLen = 0;
    p.rawLen = 0;
    p.rxSaturated = false;
}

// ---------------------------------------------------------------- 수집

// RX 링버퍼가 가득 차면 그 뒤 바이트는 드라이버가 조용히 버린다. 탭에서는 되돌려
// 받을 수 없으므로, 포화에 근접한 순간을 잡아 유실 가능 구간으로 남긴다.
// 몇 바이트를 잃었는지는 알 수 없다. 잃었을 수 있다는 사실만 기록한다.
void checkOverflow(Port& p, int avail, uint32_t now) {
    if (avail >= static_cast<int>(RX_HIGH_WATER)) {
        if (!p.rxSaturated) {
            p.rxSaturated = true;
            p.stats.overflow++;
            char line[LOG_LINE_MAX];
            const size_t n = lineHead(line, sizeof(line), p, now);
            snprintf(line + n, sizeof(line) - n,
                     "OVERFLOW: rx buffer %d/%u — 바이트 유실 가능", avail,
                     static_cast<unsigned>(NMX_RX_BUFFER));
            emitLine(line);
        }
    } else if (avail < static_cast<int>(RX_HIGH_WATER) / 2) {
        p.rxSaturated = false;
    }
}

void pumpPort(Port& p) {
    const uint32_t now = millis();
    bool got = false;

    checkOverflow(p, p.uart->available(), now);

    while (p.uart->available() > 0) {
        const int v = p.uart->read();
        if (v < 0) {
            break;
        }
        const uint8_t b = static_cast<uint8_t>(v);

        p.stats.bytes++;
        got = true;
        if (g_rawEnabled) {
            rawFeed(p, b, now);
        }
        frameFeed(p, b, now);
    }

    if (got) {
        return;
    }
    // 유휴 구간: 미완성 프레임 / 잡음 / raw 줄을 마감한다.
    if (p.frameLen > 0 && (now - p.lastByteMs) > FRAME_TIMEOUT_MS) {
        frameTimeout(p, now);
    }
    if (p.skipLen > 0 && (now - p.lastByteMs) > FRAME_TIMEOUT_MS) {
        skipFlush(p);
    }
    if (p.rawLen > 0 && (now - p.lastByteMs) > RAW_GAP_MS) {
        rawFlush(p);
    }
}

// ---------------------------------------------------------------- 포트 재설정

// end() → begin() 사이에 수신이 끊기므로, 그 순간 진행 중이던 프레임은 유실된다.
void applyPortSettings(Port& p, uint32_t baud, uint32_t fmt, bool announce) {
    const bool hadPending = (p.frameLen > 0 || p.skipLen > 0 || p.uart->available() > 0);

    p.uart->end();
    p.uart->setRxBufferSize(NMX_RX_BUFFER);
    p.uart->begin(baud, fmt, p.rxPin, -1);  // txPin = -1 → 출력 핀 미할당
    p.baud = baud;
    p.format = fmt;
    resetParser(p);

    if (announce && hadPending) {
        Serial.printf("  port%u: 진행 중이던 수신 데이터는 유실됐다\n",
                      static_cast<unsigned>(p.index + 1));
    }
}

void clearStats() {
    for (Port& p : g_port) {
        p.stats = Stats{};
    }
    g_logDropped = 0;
}

// ---------------------------------------------------------------- 명령 처리

void printHelp() {
    Serial.println(F("--- NMX passive tap sniffer commands ---"));
    Serial.println(F("  help                 이 도움말"));
    Serial.println(F("  raw on|off           수신 바이트 raw hex 출력"));
    Serial.println(F("  frame on|off         6바이트 프레임 파서 출력"));
    Serial.println(F("  dir                  포트-방향 매핑 및 판정 근거"));
    Serial.println(F("  dir auto             헤더 바이트로 자동 판정 (기본)"));
    Serial.println(F("  dir swap             두 포트의 방향 라벨을 뒤집는다"));
    Serial.println(F("  dir p1=ctrl|main     수동 고정"));
    Serial.println(F("  baud <n>             양쪽 포트 보레이트 변경"));
    Serial.println(F("  baud1|baud2 <n>      포트별 보레이트 변경"));
    Serial.println(F("  format <fmt>         양쪽 포트 프레임 형식 변경"));
    Serial.println(F("  format1|format2 <f>  포트별 프레임 형식 변경"));
    Serial.println(F("  scan [초]            보레이트 자동 탐색"));
    Serial.println(F("  scan full [초]       보레이트 x 형식 자동 탐색"));
    Serial.println(F("  stats                방향별 수신/체크섬/오버플로 통계"));
    Serial.println(F("  clear                통계 초기화"));
    Serial.println(F("  pins                 현재 RX 핀 / 보레이트 / 형식"));
    Serial.println(F("  selftest             문서 예제로 디코더 검증"));
    Serial.println();
    Serial.println(F("  송신 명령은 없다. RX 전용이라 하드웨어적으로 송신할 수 없다."));
}

void printPins() {
    for (const Port& p : g_port) {
        Serial.printf("port%u  RX=GPIO%-2d  TX=없음  %u %s  role=%s\n",
                      static_cast<unsigned>(p.index + 1), static_cast<int>(p.rxPin),
                      static_cast<unsigned>(p.baud), formatName(p.format),
                      roleName(g_role[p.index]));
    }
    Serial.printf("debug baud=%u (UART0)\n", static_cast<unsigned>(nmx::DEBUG_BAUD));
}

void printDir() {
    Serial.printf("dir mode = %s\n", g_dirAuto ? "auto" : "manual");
    for (const Port& p : g_port) {
        Serial.printf("  port%u  role=%-4s  tag=[%s]  chk-ok 프레임 중 AB=%u AA=%u\n",
                      static_cast<unsigned>(p.index + 1), roleName(g_role[p.index]), portTag(p),
                      static_cast<unsigned>(p.hdrCommand), static_cast<unsigned>(p.hdrResponse));
    }
    if (g_role[0] == Role::Unknown) {
        Serial.printf("미확정. 체크섬 OK 프레임 %u개 이상 + 한쪽 헤더 %u%% 이상에서 확정된다.\n",
                      static_cast<unsigned>(DIR_MIN_FRAMES), static_cast<unsigned>(DIR_RATIO_PCT));
        Serial.println(F("컨트롤러 버튼을 눌러 트래픽을 만들거나, 'dir p1=ctrl' 로 직접 지정한다."));
    }
}

void printStats() {
    char ts[16];
    formatTime(millis(), ts, sizeof(ts));
    Serial.printf("--- STATS (uptime %s) ---\n", ts);

    char head[2][20];
    for (const Port& p : g_port) {
        snprintf(head[p.index], sizeof(head[0]), "%s (port%u)", portTag(p),
                 static_cast<unsigned>(p.index + 1));
    }
    Serial.printf("%-14s %14s %14s\n", "", head[0], head[1]);

    struct Row {
        const char* label;
        uint32_t Stats::*field;
    };
    static const Row rows[] = {
        {"bytes", &Stats::bytes},
        {"overflow", &Stats::overflow},
        {"frames", &Stats::frames},
        {"chk OK", &Stats::chkOk},
        {"chk BAD", &Stats::chkBad},
        {"unknown cmd", &Stats::unknownCmd},
        {"resync", &Stats::resync},
        {"incomplete", &Stats::incomplete},
        {"skipped bytes", &Stats::skipped},
    };
    for (const Row& r : rows) {
        Serial.printf("%-14s %14u %14u\n", r.label,
                      static_cast<unsigned>(g_port[0].stats.*(r.field)),
                      static_cast<unsigned>(g_port[1].stats.*(r.field)));
    }
    Serial.printf("port1 %u %s   port2 %u %s\n", static_cast<unsigned>(g_port[0].baud),
                  formatName(g_port[0].format), static_cast<unsigned>(g_port[1].baud),
                  formatName(g_port[1].format));
    Serial.printf("raw=%s frame=%s dir=%s\n", g_rawEnabled ? "on" : "off",
                  g_frameEnabled ? "on" : "off", g_dirAuto ? "auto" : "manual");
    Serial.printf("dropped log lines (화면 출력 포기): %u\n", static_cast<unsigned>(g_logDropped));
    if (g_port[0].stats.overflow > 0 || g_port[1].stats.overflow > 0) {
        Serial.println(F("!! overflow > 0 — 바이트를 놓쳤다. 통계 신뢰도가 떨어진다."));
    }
}

// "on"/"off" 인자를 해석한다.
bool parseOnOff(const char* arg, bool& value) {
    if (arg == nullptr) {
        return false;
    }
    if (strcmp(arg, "on") == 0) {
        value = true;
        return true;
    }
    if (strcmp(arg, "off") == 0) {
        value = false;
        return true;
    }
    return false;
}

bool parseBaud(const char* s, uint32_t& out) {
    if (s == nullptr) {
        return false;
    }
    char* end = nullptr;
    const long v = strtol(s, &end, 10);
    if (end == s || *end != '\0' || v <= 0) {
        return false;
    }
    for (uint32_t b : BAUD_TABLE) {
        if (static_cast<uint32_t>(v) == b) {
            out = b;
            return true;
        }
    }
    return false;
}

bool parseFormat(const char* s, uint32_t& out) {
    if (s == nullptr) {
        return false;
    }
    char up[8];
    size_t i = 0;
    for (; s[i] != '\0' && i + 1 < sizeof(up); ++i) {
        up[i] = static_cast<char>(toupper(static_cast<unsigned char>(s[i])));
    }
    up[i] = '\0';
    for (const FormatEntry& f : FORMAT_TABLE) {
        if (strcmp(f.name, up) == 0) {
            out = f.cfg;
            return true;
        }
    }
    return false;
}

void printBaudList() {
    Serial.print(F("지원 보레이트:"));
    for (uint32_t b : BAUD_TABLE) {
        Serial.printf(" %u", static_cast<unsigned>(b));
    }
    Serial.println();
}

void printFormatList() {
    Serial.print(F("지원 형식:"));
    for (const FormatEntry& f : FORMAT_TABLE) {
        Serial.printf(" %s", f.name);
    }
    Serial.println();
}

// 보레이트/형식을 바꾸면 이전 설정에서 모은 통계는 의미가 없다.
// 파서 상태와 방향 판정 근거를 함께 비운다. 확정된 role 은 배선 정보라 유지한다.
void afterPortChange() {
    dirClearEvidence();
    clearStats();
    Serial.println(F("(파서 상태 / 통계 / 방향 판정 근거 초기화)"));
}

void handleBaud(char* save, int which) {
    char* arg = strtok_r(nullptr, " \t", &save);
    uint32_t baud = 0;
    if (!parseBaud(arg, baud)) {
        Serial.println(F("usage: baud|baud1|baud2 <bps>"));
        printBaudList();
        return;
    }
    for (Port& p : g_port) {
        if (which >= 0 && p.index != static_cast<uint8_t>(which)) {
            continue;
        }
        Serial.printf("port%u %u %s -> %u %s\n", static_cast<unsigned>(p.index + 1),
                      static_cast<unsigned>(p.baud), formatName(p.format),
                      static_cast<unsigned>(baud), formatName(p.format));
        applyPortSettings(p, baud, p.format, true);
    }
    afterPortChange();
}

void handleFormat(char* save, int which) {
    char* arg = strtok_r(nullptr, " \t", &save);
    uint32_t fmt = 0;
    if (!parseFormat(arg, fmt)) {
        Serial.println(F("usage: format|format1|format2 <8N1|8E1|8O1|7E1|7O1|8N2>"));
        printFormatList();
        return;
    }
    for (Port& p : g_port) {
        if (which >= 0 && p.index != static_cast<uint8_t>(which)) {
            continue;
        }
        Serial.printf("port%u %u %s -> %u %s\n", static_cast<unsigned>(p.index + 1),
                      static_cast<unsigned>(p.baud), formatName(p.format),
                      static_cast<unsigned>(p.baud), formatName(fmt));
        applyPortSettings(p, p.baud, fmt, true);
    }
    afterPortChange();
}

void handleDir(char* save) {
    char* arg = strtok_r(nullptr, " \t", &save);
    if (arg == nullptr) {
        printDir();
        return;
    }
    for (char* q = arg; *q != '\0'; ++q) {
        *q = static_cast<char>(tolower(static_cast<unsigned char>(*q)));
    }

    if (strcmp(arg, "auto") == 0) {
        g_dirAuto = true;
        g_role[0] = Role::Unknown;
        g_role[1] = Role::Unknown;
        dirClearEvidence();
        Serial.println(F("dir = auto. 근거를 비우고 다시 판정한다."));
        return;
    }
    if (strcmp(arg, "swap") == 0) {
        if (g_role[0] == Role::Unknown) {
            Serial.println(F("ERR: 아직 미확정이라 뒤집을 것이 없다. 'dir p1=ctrl' 로 지정한다."));
            return;
        }
        assignRoles(0, (g_role[0] == Role::Ctrl) ? Role::Main : Role::Ctrl);
        g_dirAuto = false;  // 사용자가 덮어썼으므로 자동 판정이 되돌리지 않게 한다
        announceRoles("swapped");
        Serial.println(F("dir mode = manual ('dir auto' 로 되돌린다)"));
        return;
    }
    if (strcmp(arg, "p1=ctrl") == 0 || strcmp(arg, "p1=main") == 0) {
        assignRoles(0, (strcmp(arg, "p1=ctrl") == 0) ? Role::Ctrl : Role::Main);
        g_dirAuto = false;
        announceRoles("set");
        Serial.println(F("dir mode = manual ('dir auto' 로 되돌린다)"));
        return;
    }
    Serial.println(F("usage: dir | dir auto | dir swap | dir p1=ctrl | dir p1=main"));
}

// ---------------------------------------------------------------- 보레이트 자동 탐색

struct ScanCount {
    uint32_t frames[2];
    uint32_t chkOk[2];
};

// 한 후보 설정으로 durMs 동안 듣고 프레임 수를 센다.
// 점수는 체크섬이 맞는 프레임 수로만 낸다. 하드웨어 프레이밍 에러 플래그는
// 코어 버전에 따라 신뢰도가 달라 쓰지 않는다. 우리는 체크섬 식을 이미 알고 있다.
// Serial 입력이 들어오면 중단하고 false 를 반환한다.
bool scanMeasure(uint32_t baud, uint32_t fmt, uint32_t durMs, ScanCount& out) {
    for (Port& p : g_port) {
        applyPortSettings(p, baud, fmt, false);
    }
    delay(30);
    for (Port& p : g_port) {
        while (p.uart->available() > 0) {
            p.uart->read();
        }
    }

    uint8_t buf[2][nmx::FRAME_LEN];
    size_t len[2] = {0, 0};
    out = ScanCount{};

    const uint32_t t0 = millis();
    while ((millis() - t0) < durMs) {
        if (Serial.available() > 0) {
            return false;
        }
        for (Port& p : g_port) {
            const uint8_t i = p.index;
            while (p.uart->available() > 0) {
                const int v = p.uart->read();
                if (v < 0) {
                    break;
                }
                const uint8_t b = static_cast<uint8_t>(v);
                if (len[i] == 0) {
                    if (!nmx::isHeader(b)) {
                        continue;
                    }
                    buf[i][0] = b;
                    len[i] = 1;
                    continue;
                }
                buf[i][len[i]++] = b;
                if (len[i] == nmx::FRAME_LEN) {
                    out.frames[i]++;
                    if (nmx::isChecksumOk(buf[i])) {
                        out.chkOk[i]++;
                    }
                    len[i] = 0;
                }
            }
        }
        delay(1);
    }
    return true;
}

void handleScan(char* save) {
    bool full = false;
    uint32_t sec = SCAN_DEFAULT_SEC;

    for (char* arg = strtok_r(nullptr, " \t", &save); arg != nullptr;
         arg = strtok_r(nullptr, " \t", &save)) {
        if (strcmp(arg, "full") == 0) {
            full = true;
            continue;
        }
        char* end = nullptr;
        const long v = strtol(arg, &end, 10);
        if (end == arg || *end != '\0' || v < 1 || v > static_cast<long>(SCAN_MAX_SEC)) {
            Serial.printf("usage: scan [full] [1..%u초]\n", static_cast<unsigned>(SCAN_MAX_SEC));
            return;
        }
        sec = static_cast<uint32_t>(v);
    }

    // 현재 설정을 기억해 뒀다가 스캔이 끝나면 반드시 되돌린다.
    const uint32_t savedBaud[2] = {g_port[0].baud, g_port[1].baud};
    const uint32_t savedFmt[2] = {g_port[0].format, g_port[1].format};

    const size_t nBaud = sizeof(BAUD_TABLE) / sizeof(BAUD_TABLE[0]);
    const size_t nFmt = full ? (sizeof(FORMAT_TABLE) / sizeof(FORMAT_TABLE[0])) : 1;
    const uint32_t total = static_cast<uint32_t>(nBaud * nFmt) * sec;

    Serial.printf("scan: %u baud x %u format, %u초씩 (약 %u초)\n", static_cast<unsigned>(nBaud),
                  static_cast<unsigned>(nFmt), static_cast<unsigned>(sec),
                  static_cast<unsigned>(total));
    Serial.println(F("장비가 조용하면 아무것도 못 찾는다. 스캔 중 컨트롤러를 계속 조작한다."));
    Serial.println(F("아무 키나 누르면 중단한다."));
    Serial.printf("%7s %5s %10s %10s %7s\n", "baud", "fmt", "p1 f/ok", "p2 f/ok", "score");

    uint32_t bestScore = 0;
    uint32_t secondScore = 0;
    uint32_t bestBaud = savedBaud[0];
    uint32_t bestFmt = savedFmt[0];
    bool aborted = false;

    for (size_t fi = 0; fi < nFmt && !aborted; ++fi) {
        const uint32_t fmt = full ? FORMAT_TABLE[fi].cfg : g_port[0].format;
        for (size_t bi = 0; bi < nBaud; ++bi) {
            ScanCount c;
            if (!scanMeasure(BAUD_TABLE[bi], fmt, sec * 1000, c)) {
                aborted = true;
                break;
            }
            const uint32_t score = c.chkOk[0] + c.chkOk[1];
            char p1[12];
            char p2[12];
            snprintf(p1, sizeof(p1), "%u/%u", static_cast<unsigned>(c.frames[0]),
                     static_cast<unsigned>(c.chkOk[0]));
            snprintf(p2, sizeof(p2), "%u/%u", static_cast<unsigned>(c.frames[1]),
                     static_cast<unsigned>(c.chkOk[1]));
            Serial.printf("%7u %5s %10s %10s %7u%s\n", static_cast<unsigned>(BAUD_TABLE[bi]),
                          formatName(fmt), p1, p2, static_cast<unsigned>(score),
                          (score > bestScore && score > 0) ? "  <-- best" : "");
            if (score > bestScore) {
                secondScore = bestScore;
                bestScore = score;
                bestBaud = BAUD_TABLE[bi];
                bestFmt = fmt;
            } else if (score > secondScore) {
                secondScore = score;
            }
        }
    }

    // 어떤 경로로 끝나든 원래 설정으로 복귀한 뒤에 결과를 판단한다.
    for (Port& p : g_port) {
        applyPortSettings(p, savedBaud[p.index], savedFmt[p.index], false);
    }
    dirClearEvidence();
    clearStats();

    if (aborted) {
        while (Serial.available() > 0) {
            Serial.read();
        }
        Serial.println(F("scan 중단. 원래 설정으로 되돌렸다."));
        return;
    }
    if (bestScore == 0) {
        Serial.println(F("result: 없음. 체크섬이 맞는 프레임을 한 개도 못 찾았다."));
        Serial.println(F("  - 조작 없이 스캔했다면 트래픽이 없었던 것이다. 다시 시도한다"));
        Serial.println(F("  - 'scan full' 로 프레임 형식까지 훑는다"));
        Serial.println(F("  - 그래도 없으면 신호 자체가 안 들어오는 것이다. 배선/GND를 본다 (ToDo A-2)"));
        return;
    }

    const char* confidence;
    if (bestScore >= 10 && (secondScore == 0 || bestScore >= secondScore * 3)) {
        confidence = "high";
    } else if (bestScore >= 5) {
        confidence = "medium";
    } else {
        confidence = "low";
    }
    Serial.printf("result: %u %s  score=%u  (confidence %s)\n", static_cast<unsigned>(bestBaud),
                  formatName(bestFmt), static_cast<unsigned>(bestScore), confidence);

    // y/N 확인. 스캔 중에는 콘솔을 읽지 않았으므로 여기서 직접 받는다.
    Serial.print(F("apply? (y/N) "));
    while (Serial.available() > 0) {
        Serial.read();
    }
    bool yes = false;
    const uint32_t t0 = millis();
    while ((millis() - t0) < 15000) {
        const int v = Serial.read();
        if (v < 0) {
            delay(5);
            continue;
        }
        const char c = static_cast<char>(v);
        if (c == 'y' || c == 'Y') {
            yes = true;
            break;
        }
        if (c == 'n' || c == 'N' || c == '\r' || c == '\n') {
            break;
        }
    }
    Serial.println(yes ? F("y") : F("n"));

    if (!yes) {
        Serial.println(F("변경하지 않았다."));
        return;
    }
    for (Port& p : g_port) {
        applyPortSettings(p, bestBaud, bestFmt, false);
    }
    dirClearEvidence();
    clearStats();
    Serial.printf("applied: 양쪽 포트 %u %s\n", static_cast<unsigned>(bestBaud),
                  formatName(bestFmt));
}

// ---------------------------------------------------------------- 콘솔

void handleCommand(char* line) {
    char* save = nullptr;
    char* cmd = strtok_r(line, " \t", &save);
    if (cmd == nullptr) {
        return;
    }
    for (char* p = cmd; *p != '\0'; ++p) {
        *p = static_cast<char>(tolower(static_cast<unsigned char>(*p)));
    }

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        printHelp();
        return;
    }
    if (strcmp(cmd, "raw") == 0) {
        char* arg = strtok_r(nullptr, " \t", &save);
        if (!parseOnOff(arg, g_rawEnabled)) {
            Serial.println(F("usage: raw on|off"));
            return;
        }
        if (!g_rawEnabled) {
            for (Port& p : g_port) {
                rawFlush(p);
            }
        }
        Serial.printf("raw = %s\n", g_rawEnabled ? "on" : "off");
        return;
    }
    if (strcmp(cmd, "frame") == 0) {
        char* arg = strtok_r(nullptr, " \t", &save);
        if (!parseOnOff(arg, g_frameEnabled)) {
            Serial.println(F("usage: frame on|off"));
            return;
        }
        Serial.printf("frame = %s\n", g_frameEnabled ? "on" : "off");
        return;
    }
    if (strcmp(cmd, "dir") == 0) {
        handleDir(save);
        return;
    }
    if (strcmp(cmd, "baud") == 0) {
        handleBaud(save, -1);
        return;
    }
    if (strcmp(cmd, "baud1") == 0) {
        handleBaud(save, 0);
        return;
    }
    if (strcmp(cmd, "baud2") == 0) {
        handleBaud(save, 1);
        return;
    }
    if (strcmp(cmd, "format") == 0) {
        handleFormat(save, -1);
        return;
    }
    if (strcmp(cmd, "format1") == 0) {
        handleFormat(save, 0);
        return;
    }
    if (strcmp(cmd, "format2") == 0) {
        handleFormat(save, 1);
        return;
    }
    if (strcmp(cmd, "scan") == 0) {
        handleScan(save);
        return;
    }
    if (strcmp(cmd, "stats") == 0) {
        printStats();
        return;
    }
    if (strcmp(cmd, "clear") == 0) {
        clearStats();
        Serial.println(F("stats cleared"));
        return;
    }
    if (strcmp(cmd, "pins") == 0) {
        printPins();
        return;
    }
    if (strcmp(cmd, "selftest") == 0) {
        nmx::runSelfTest();
        return;
    }
    if (strcmp(cmd, "relay") == 0 || strcmp(cmd, "inject") == 0 ||
        strcmp(cmd, "sendmain") == 0 || strcmp(cmd, "sendctrl") == 0) {
        Serial.printf("ERR: '%s' 는 브리지 전용 명령이다. 이 펌웨어는 RX 전용이라 송신할 수 없다.\n",
                      cmd);
        return;
    }

    Serial.printf("ERR: unknown command '%s' ('help' 참고)\n", cmd);
}

void pumpConsole() {
    while (Serial.available() > 0) {
        const int v = Serial.read();
        if (v < 0) {
            break;
        }
        const char c = static_cast<char>(v);
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            g_cmdLine[g_cmdLen] = '\0';
            if (g_cmdLen > 0) {
                handleCommand(g_cmdLine);
            }
            g_cmdLen = 0;
            continue;
        }
        if (g_cmdLen + 1 < CMD_LINE_MAX) {
            g_cmdLine[g_cmdLen++] = c;
        }
    }
}

}  // namespace

// ---------------------------------------------------------------- setup/loop

void setup() {
    Serial.setTxBufferSize(LOG_TX_BUFFER);
    Serial.begin(nmx::DEBUG_BAUD);

    // txPin 에 -1 을 넘겨 UART 드라이버가 어떤 핀도 출력으로 잡지 않게 한다.
    // 브리지 펌웨어에 있던 TX idle 고정(pinMode/digitalWrite)은 여기에 없다.
    // 출력 핀이 아예 없으므로 부팅 순간 장비 쪽으로 나갈 바이트도 없다.
    for (Port& p : g_port) {
        p.uart->setRxBufferSize(NMX_RX_BUFFER);
        p.uart->begin(p.baud, p.format, p.rxPin, -1);
    }

    delay(200);
    Serial.println();
    Serial.println(F("======================================================"));
    Serial.printf(" NMX-1108 / NMX-1106 RS-232C passive tap sniffer  v%s\n", FIRMWARE_VERSION);
    Serial.println(F("======================================================"));
    printPins();
    Serial.println(F("mode: PASSIVE (RX only). 송신 불가. ESP32가 꺼져도 장비 통신은 유지된다."));
    Serial.println(F("방향은 헤더 바이트로 자동 판정한다. 확정 전에는 [P1?] [P2?] 로 찍힌다."));
    Serial.println(F("'help' 를 입력하면 명령 목록이 나온다."));
    Serial.println();
}

void loop() {
    for (Port& p : g_port) {
        pumpPort(p);
    }
    pumpConsole();
}
