// NMX-1108 / NMX-1106 RS-232C 브리지 스니퍼 (ESP32 / Arduino / PlatformIO)
//
// ESP32가 컨트롤러와 본체 사이에 끼어들어 양방향 데이터를 그대로 중계하면서
// 지나가는 프레임을 해석해 PC로 기록한다.
//
//   [NMX 컨트롤러] <--RS232--> MAX3232 #1 <--> ESP32 <--> MAX3232 #2 <--RS232--> [NMX 본체]
//                                          Serial1      Serial2
//
//   Serial1 RX (컨트롤러 송신) --중계--> Serial2 TX (본체로)      : CTRL->MAIN
//   Serial2 RX (본체 송신)     --중계--> Serial1 TX (컨트롤러로)  : MAIN->CTRL
//
// 여기서 "중계"는 아래 pumpDirection()이 read()/write()로 옮기는 소프트웨어 동작이다.
// RX 핀과 반대편 TX 핀을 전선으로 잇는 것이 아니다. GPIO끼리는 절대 연결하지 않는다.
//
// 중계는 프레임을 모으지 않고 바이트 단위로 즉시 통과시킨다. 파서가 동기를 잃거나
// 체크섬이 깨져도 중계에는 영향이 없다.
//
// 주의: ESP32가 꺼지거나 리셋되면 컨트롤러-본체 통신도 함께 끊긴다.

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "nmx_protocol.h"
#include "nmx_selftest.h"

// ---------------------------------------------------------------- 핀 설정
// build_flags 에서 -DCTRL_RX_PIN=... 형태로 덮어쓸 수 있다.
#ifndef CTRL_RX_PIN
#define CTRL_RX_PIN 16  // Serial1 RX  <- MAX3232 #1  <- 컨트롤러 TxD
#endif
#ifndef CTRL_TX_PIN
#define CTRL_TX_PIN 17  // Serial1 TX  -> MAX3232 #1  -> 컨트롤러 RxD
#endif
#ifndef MAIN_RX_PIN
#define MAIN_RX_PIN 26  // Serial2 RX  <- MAX3232 #2  <- 본체 TxD
#endif
#ifndef MAIN_TX_PIN
#define MAIN_TX_PIN 27  // Serial2 TX  -> MAX3232 #2  -> 본체 RxD
#endif

// ---------------------------------------------------------------- 동작 상수
namespace {

constexpr char FIRMWARE_VERSION[] = "2.0.0-bridge";

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

// 중계 지연을 줄이기 위해 장비 UART는 넉넉한 RX 버퍼를 잡고,
// PC 로그는 큰 TX 버퍼로 순간 폭주를 흡수한다.
constexpr size_t NMX_RX_BUFFER = 512;
constexpr size_t LOG_TX_BUFFER = 4096;

// ---------------------------------------------------------------- 상태
struct Stats {
    uint32_t bytes;
    uint32_t relayed;
    uint32_t frames;
    uint32_t chkOk;
    uint32_t chkBad;
    uint32_t unknownCmd;
    uint32_t resync;
    uint32_t incomplete;
    uint32_t skipped;
};

struct Direction {
    HardwareSerial* in;   // 이 방향의 데이터가 들어오는 포트
    HardwareSerial* out;  // 그대로 내보낼 반대편 포트
    const char* tag;

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

    Stats stats;
};

Direction g_dir[2] = {
    {&Serial1, &Serial2, "CTRL->MAIN"},
    {&Serial2, &Serial1, "MAIN->CTRL"},
};

bool g_relayEnabled = true;  // 기본 ON. 끄면 장비 통신이 끊긴다.
bool g_frameEnabled = true;
bool g_rawEnabled = false;
bool g_injectArmed = false;  // 임의 프레임 송신은 기본 차단

uint32_t g_logDropped = 0;  // 중계 보호를 위해 버린 로그 줄 수

char g_cmdLine[CMD_LINE_MAX];
size_t g_cmdLen = 0;

// ---------------------------------------------------------------- 로그 출력
//
// USB 로그 때문에 중계가 멈추면 안 되므로, 출력 버퍼에 여유가 없으면
// 로그 줄을 통째로 버리고 카운트만 올린다. 중계가 항상 우선이다.
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

// "[CTRL->MAIN] 00:01:23.456  " 를 채우고 그 길이를 반환한다.
size_t lineHead(char* out, size_t outLen, const Direction& d, uint32_t ms) {
    char ts[16];
    formatTime(ms, ts, sizeof(ts));
    const int n = snprintf(out, outLen, "[%s] %s  ", d.tag, ts);
    return (n < 0) ? 0 : static_cast<size_t>(n);
}

// ---------------------------------------------------------------- raw 모드

void rawFlush(Direction& d) {
    if (d.rawLen == 0) {
        return;
    }
    char hex[HEX_MAX];
    char line[LOG_LINE_MAX];
    nmx::formatHex(d.raw, d.rawLen, hex, sizeof(hex));
    const size_t n = lineHead(line, sizeof(line), d, d.rawFirstMs);
    snprintf(line + n, sizeof(line) - n, "RAW  %s", hex);
    emitLine(line);
    d.rawLen = 0;
}

void rawFeed(Direction& d, uint8_t b, uint32_t now) {
    if (d.rawLen == 0) {
        d.rawFirstMs = now;
    }
    d.raw[d.rawLen++] = b;
    if (d.rawLen >= RAW_CHUNK) {
        rawFlush(d);
    }
}

// ---------------------------------------------------------------- 프레임 파서

void skipFlush(Direction& d) {
    if (d.skipLen == 0) {
        return;
    }
    if (g_frameEnabled) {
        char hex[HEX_MAX];
        char line[LOG_LINE_MAX];
        nmx::formatHex(d.skip, d.skipLen, hex, sizeof(hex));
        const size_t n = lineHead(line, sizeof(line), d, d.skipFirstMs);
        snprintf(line + n, sizeof(line) - n, "SKIP %u byte(s): %s",
                 static_cast<unsigned>(d.skipLen), hex);
        emitLine(line);
    }
    d.skipLen = 0;
}

void skipPush(Direction& d, uint8_t b, uint32_t now) {
    if (d.skipLen == 0) {
        d.skipFirstMs = now;
    }
    if (d.skipLen < SKIP_MAX) {
        d.skip[d.skipLen++] = b;
    }
    d.stats.skipped++;
    if (d.skipLen >= SKIP_MAX) {
        skipFlush(d);
    }
}

void frameFeed(Direction& d, uint8_t b, uint32_t now);

// 6바이트가 모였을 때 호출한다. 체크섬이 틀리면 프레임 내부의 헤더 바이트를
// 기준으로 동기를 다시 잡는다. (정상 프레임은 데이터에 0xAA/0xAB가 있어도 건드리지 않는다)
void frameComplete(Direction& d, uint32_t now) {
    const bool ok = nmx::isChecksumOk(d.frame);

    d.stats.frames++;
    if (ok) {
        d.stats.chkOk++;
    } else {
        d.stats.chkBad++;
    }
    if (!nmx::isKnownCmd(d.frame[1])) {
        d.stats.unknownCmd++;
    }

    if (g_frameEnabled) {
        char hex[HEX_MAX];
        char desc[DESC_MAX];
        char line[LOG_LINE_MAX];
        nmx::formatHex(d.frame, nmx::FRAME_LEN, hex, sizeof(hex));
        nmx::describeFrame(d.frame, desc, sizeof(desc));

        size_t n = lineHead(line, sizeof(line), d, now);
        n += snprintf(line + n, sizeof(line) - n, "%s  CHK=%-3s  %s", hex, ok ? "OK" : "BAD", desc);
        if (!ok && n < sizeof(line)) {
            snprintf(line + n, sizeof(line) - n, "  [expected CHK=%02X]",
                     static_cast<unsigned>(nmx::checksum(d.frame)));
        }
        emitLine(line);
    }

    d.frameLen = 0;

    if (ok) {
        return;
    }

    // 체크섬 불일치 → 프레임 중간의 헤더 바이트부터 재동기 시도
    size_t resyncAt = 0;
    for (size_t i = 1; i < nmx::FRAME_LEN; ++i) {
        if (nmx::isHeader(d.frame[i])) {
            resyncAt = i;
            break;
        }
    }
    if (resyncAt == 0) {
        return;
    }

    d.stats.resync++;
    if (g_frameEnabled) {
        char line[LOG_LINE_MAX];
        const size_t n = lineHead(line, sizeof(line), d, now);
        snprintf(line + n, sizeof(line) - n, "RESYNC from offset %u",
                 static_cast<unsigned>(resyncAt));
        emitLine(line);
    }

    uint8_t tail[nmx::FRAME_LEN];
    const size_t tailLen = nmx::FRAME_LEN - resyncAt;
    memcpy(tail, d.frame + resyncAt, tailLen);
    // tailLen <= 5 이므로 여기서 frameComplete()가 다시 불리는 일은 없다.
    for (size_t i = 0; i < tailLen; ++i) {
        frameFeed(d, tail[i], now);
    }
}

void frameFeed(Direction& d, uint8_t b, uint32_t now) {
    d.lastByteMs = now;

    if (d.frameLen == 0) {
        if (!nmx::isHeader(b)) {
            skipPush(d, b, now);
            return;
        }
        skipFlush(d);
        d.frame[0] = b;
        d.frameLen = 1;
        return;
    }

    d.frame[d.frameLen++] = b;
    if (d.frameLen == nmx::FRAME_LEN) {
        frameComplete(d, now);
    }
}

// 프레임이 도중에 끊긴 채 유휴 상태가 되면 버리지 않고 그대로 출력한다.
void frameTimeout(Direction& d, uint32_t now) {
    if (d.frameLen == 0) {
        return;
    }
    d.stats.incomplete++;
    if (g_frameEnabled) {
        char hex[HEX_MAX];
        char line[LOG_LINE_MAX];
        nmx::formatHex(d.frame, d.frameLen, hex, sizeof(hex));
        const size_t n = lineHead(line, sizeof(line), d, now);
        snprintf(line + n, sizeof(line) - n, "%s  INCOMPLETE (%u/%u bytes, timeout)", hex,
                 static_cast<unsigned>(d.frameLen), static_cast<unsigned>(nmx::FRAME_LEN));
        emitLine(line);
    }
    d.frameLen = 0;
}

// ---------------------------------------------------------------- 중계 + 파싱

void pumpDirection(Direction& d) {
    const uint32_t now = millis();
    bool got = false;

    while (d.in->available() > 0) {
        const int v = d.in->read();
        if (v < 0) {
            break;
        }
        const uint8_t b = static_cast<uint8_t>(v);

        // 1) 중계가 최우선. 해석보다 먼저 반대편으로 내보낸다.
        if (g_relayEnabled) {
            d.out->write(b);
            d.stats.relayed++;
        }

        // 2) 그 다음에 관찰용으로 해석한다.
        d.stats.bytes++;
        got = true;
        if (g_rawEnabled) {
            rawFeed(d, b, now);
        }
        frameFeed(d, b, now);
    }

    if (got) {
        return;
    }
    // 유휴 구간: 미완성 프레임 / 잡음 / raw 줄을 마감한다.
    if (d.frameLen > 0 && (now - d.lastByteMs) > FRAME_TIMEOUT_MS) {
        frameTimeout(d, now);
    }
    if (d.skipLen > 0 && (now - d.lastByteMs) > FRAME_TIMEOUT_MS) {
        skipFlush(d);
    }
    if (d.rawLen > 0 && (now - d.lastByteMs) > RAW_GAP_MS) {
        rawFlush(d);
    }
}

// ---------------------------------------------------------------- 명령 처리

void printHelp() {
    Serial.println(F("--- NMX bridge sniffer commands ---"));
    Serial.println(F("  help                 이 도움말"));
    Serial.println(F("  relay on|off         양방향 중계 (기본 on. off면 장비 통신 끊김)"));
    Serial.println(F("  raw on|off           수신 바이트 raw hex 출력"));
    Serial.println(F("  frame on|off         6바이트 프레임 파서 출력"));
    Serial.println(F("  stats                방향별 수신/중계/체크섬 통계"));
    Serial.println(F("  clear                통계 초기화"));
    Serial.println(F("  pins                 현재 UART 핀 설정"));
    Serial.println(F("  selftest             문서 예제로 디코더 검증"));
    Serial.println(F("  inject on|off        임의 프레임 송신 허용 (기본 off)"));
    Serial.println(F("  sendmain <hex...>    본체로 송신   (5바이트면 CHK 자동)"));
    Serial.println(F("  sendctrl <hex...>    컨트롤러로 송신 (5바이트면 CHK 자동)"));
}

void printPins() {
    Serial.printf("Serial1 (컨트롤러측) RX=GPIO%d TX=GPIO%d\n", CTRL_RX_PIN, CTRL_TX_PIN);
    Serial.printf("Serial2 (본체측)     RX=GPIO%d TX=GPIO%d\n", MAIN_RX_PIN, MAIN_TX_PIN);
    Serial.printf("relay: Serial1 RX -> Serial2 TX (%s), Serial2 RX -> Serial1 TX (%s)\n",
                  g_dir[0].tag, g_dir[1].tag);
    Serial.printf("baud=%u 8N1, debug baud=%u\n", static_cast<unsigned>(nmx::NMX_BAUD),
                  static_cast<unsigned>(nmx::DEBUG_BAUD));
}

void printStats() {
    char ts[16];
    formatTime(millis(), ts, sizeof(ts));
    Serial.printf("--- STATS (uptime %s) ---\n", ts);
    Serial.printf("%-14s %12s %12s\n", "", g_dir[0].tag, g_dir[1].tag);

    struct Row {
        const char* label;
        uint32_t Stats::*field;
    };
    static const Row rows[] = {
        {"bytes", &Stats::bytes},
        {"relayed", &Stats::relayed},
        {"frames", &Stats::frames},
        {"chk OK", &Stats::chkOk},
        {"chk BAD", &Stats::chkBad},
        {"unknown cmd", &Stats::unknownCmd},
        {"resync", &Stats::resync},
        {"incomplete", &Stats::incomplete},
        {"skipped bytes", &Stats::skipped},
    };
    for (const Row& r : rows) {
        Serial.printf("%-14s %12u %12u\n", r.label,
                      static_cast<unsigned>(g_dir[0].stats.*(r.field)),
                      static_cast<unsigned>(g_dir[1].stats.*(r.field)));
    }
    Serial.printf("relay=%s raw=%s frame=%s inject=%s\n", g_relayEnabled ? "ON" : "OFF",
                  g_rawEnabled ? "on" : "off", g_frameEnabled ? "on" : "off",
                  g_injectArmed ? "ARMED" : "off");
    Serial.printf("dropped log lines (중계 보호): %u\n", static_cast<unsigned>(g_logDropped));
}

void clearStats() {
    for (Direction& d : g_dir) {
        d.stats = Stats{};
    }
    g_logDropped = 0;
    Serial.println(F("stats cleared"));
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

// "AB 09 00 00 01 4A" 형태의 공백 구분 hex를 파싱한다.
size_t parseHexBytes(char* rest, uint8_t* out, size_t maxLen) {
    size_t n = 0;
    char* save = nullptr;
    for (char* tok = strtok_r(rest, " \t", &save); tok != nullptr && n < maxLen;
         tok = strtok_r(nullptr, " \t", &save)) {
        char* end = nullptr;
        const long v = strtol(tok, &end, 16);
        if (end == tok || *end != '\0' || v < 0 || v > 0xFF) {
            return 0;
        }
        out[n++] = static_cast<uint8_t>(v);
    }
    return n;
}

// port로 임의 프레임을 내보낸다. 중계 트래픽과 섞이므로 신중히 쓴다.
void handleSend(HardwareSerial* port, const char* what, char* rest) {
    if (!g_injectArmed) {
        Serial.println(F("ERR: injection disarmed. 'inject on' 먼저 실행"));
        return;
    }
    uint8_t buf[nmx::FRAME_LEN];
    size_t n = parseHexBytes(rest, buf, nmx::FRAME_LEN);
    if (n != 5 && n != nmx::FRAME_LEN) {
        Serial.println(F("ERR: 5바이트(CHK 자동) 또는 6바이트 hex가 필요하다"));
        return;
    }
    if (n == 5) {
        buf[5] = nmx::checksum(buf);
        n = nmx::FRAME_LEN;
    }
    port->write(buf, n);
    port->flush();

    char hex[HEX_MAX];
    char desc[DESC_MAX];
    nmx::formatHex(buf, n, hex, sizeof(hex));
    nmx::describeFrame(buf, desc, sizeof(desc));
    Serial.printf("TX ->%s: %s  CHK=%-3s  %s\n", what, hex, nmx::isChecksumOk(buf) ? "OK" : "BAD",
                  desc);
}

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
    if (strcmp(cmd, "relay") == 0) {
        char* arg = strtok_r(nullptr, " \t", &save);
        if (!parseOnOff(arg, g_relayEnabled)) {
            Serial.println(F("usage: relay on|off"));
            return;
        }
        if (g_relayEnabled) {
            Serial.println(F("relay = ON"));
        } else {
            Serial.println(F("relay = OFF  !! 컨트롤러-본체 통신이 끊긴 상태다"));
        }
        return;
    }
    if (strcmp(cmd, "raw") == 0) {
        char* arg = strtok_r(nullptr, " \t", &save);
        if (!parseOnOff(arg, g_rawEnabled)) {
            Serial.println(F("usage: raw on|off"));
            return;
        }
        if (!g_rawEnabled) {
            for (Direction& d : g_dir) {
                rawFlush(d);
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
    if (strcmp(cmd, "stats") == 0) {
        printStats();
        return;
    }
    if (strcmp(cmd, "clear") == 0) {
        clearStats();
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
    if (strcmp(cmd, "inject") == 0) {
        char* arg = strtok_r(nullptr, " \t", &save);
        if (!parseOnOff(arg, g_injectArmed)) {
            Serial.println(F("usage: inject on|off"));
            return;
        }
        Serial.printf("inject = %s\n", g_injectArmed ? "ARMED (송신 가능)" : "off");
        return;
    }
    if (strcmp(cmd, "sendmain") == 0) {
        handleSend(&Serial2, "MAIN", save);
        return;
    }
    if (strcmp(cmd, "sendctrl") == 0) {
        handleSend(&Serial1, "CTRL", save);
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

    // UART 초기화 전 TX 핀이 떠 있으면 MAX3232 입력이 불확정 상태가 되어
    // 장비 쪽에 쓰레기 바이트가 실릴 수 있다. UART idle 레벨(High)로 먼저 고정한다.
    pinMode(CTRL_TX_PIN, OUTPUT);
    digitalWrite(CTRL_TX_PIN, HIGH);
    pinMode(MAIN_TX_PIN, OUTPUT);
    digitalWrite(MAIN_TX_PIN, HIGH);

    Serial1.setRxBufferSize(NMX_RX_BUFFER);
    Serial2.setRxBufferSize(NMX_RX_BUFFER);
    Serial1.begin(nmx::NMX_BAUD, SERIAL_8N1, CTRL_RX_PIN, CTRL_TX_PIN);
    Serial2.begin(nmx::NMX_BAUD, SERIAL_8N1, MAIN_RX_PIN, MAIN_TX_PIN);

    delay(200);
    Serial.println();
    Serial.println(F("======================================================"));
    Serial.printf(" NMX-1108 / NMX-1106 RS-232C bridge sniffer  v%s\n", FIRMWARE_VERSION);
    Serial.println(F("======================================================"));
    printPins();
    Serial.println(F("mode: bridge (양방향 중계 + 감청). ESP32가 꺼지면 통신도 끊긴다."));
    Serial.println(F("'help' 를 입력하면 명령 목록이 나온다."));
    Serial.println();
}

void loop() {
    for (Direction& d : g_dir) {
        pumpDirection(d);
    }
    pumpConsole();
}
