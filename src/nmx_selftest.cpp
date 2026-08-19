#include "nmx_selftest.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "nmx_protocol.h"

namespace nmx {
namespace {

struct Case {
    const char* hex;         // "AB 09 00 00 01 4A"
    bool chkOk;              // 기대 체크섬 판정
    const char* expectDesc;  // 기대 디코딩 문자열
};

// 출처: doc/NMX-1106_통신 프로토콜.txt 및 doc/README.md
// '?' 가 붙은 항목은 NMX-1108 미확인 추정값이다.
const Case kCases[] = {
    // SETUP
    {"AB 01 00 00 06 4D", true, "SETUP ENTER(MENU)"},
    {"AB 01 00 00 08 4B", true, "SETUP ESC"},
    {"AB 01 00 00 01 52", true, "SETUP UP?"},
    {"AB 01 00 00 00 53", true, "SETUP DOWN?"},
    {"AB 01 00 00 03 50", true, "SETUP LEFT (sub menu -)"},
    {"AB 01 00 00 02 51", true, "SETUP RIGHT (sub menu +)"},
    // AUDIO EXT
    {"AB 02 00 00 00 52", true, "AUDIO SELECT EXT OFF"},
    {"AB 02 00 00 01 51", true, "AUDIO SELECT EXT ON"},
    // EFFECT
    {"AB 05 00 00 01 4E", true, "EFFECT 1 (LEFT/RIGHT)"},
    {"AB 05 00 00 02 4D", true, "EFFECT 2 (UP/DOWN)"},
    {"AB 05 00 00 03 4C", true, "EFFECT 3"},
    {"AB 05 00 00 07 48", true, "EFFECT 7 (FOUT/IN)"},
    {"AB 05 00 00 08 47", true, "EFFECT 8 (MIX)"},
    {"AB 05 00 00 11 3E", true, "EFFECT 1 INV (LEFT/RIGHT)"},
    {"AB 05 00 00 18 37", true, "EFFECT 8 INV (MIX)"},
    // FADER
    {"AB 06 00 00 02 4C", true, "FADER MODE CUT"},
    {"AB 06 00 00 01 4D", true, "FADER MODE TAKE"},
    {"AB 07 00 00 00 4D", true, "FADER OFF"},
    {"AB 07 00 00 11 3C", true, "FADER 0x011 (17)"},
    {"AB 07 00 00 FF 4E", true, "FADER 0x0FF (255)"},
    {"AB 07 00 01 00 4C", true, "FADER 0x100 (256)"},
    {"AB 07 00 01 E0 6C", true, "FADER 0x1E0 (480)"},
    {"AB 07 00 01 FF 4D", true, "FADER 0x1FF (511)"},
    // PGM OVERLAY
    {"AB 08 00 00 00 4C", true, "PGM OVERLAY ALL OFF"},
    {"AB 08 00 00 04 48", true, "PGM OVERLAY PIP"},
    {"AB 08 00 00 02 4A", true, "PGM OVERLAY DSK"},
    {"AB 08 00 00 01 4B", true, "PGM OVERLAY CKEY"},
    {"AB 08 00 00 07 45", true, "PGM OVERLAY CKEY DSK PIP"},
    {"AB 08 00 00 03 49", true, "PGM OVERLAY CKEY DSK"},
    // PGM CH
    {"AB 09 00 00 00 4B", true, "PGM CH OFF"},
    {"AB 09 00 00 01 4A", true, "PGM CH 1"},
    {"AB 09 00 00 06 45", true, "PGM CH 6"},
    {"AB 09 00 00 07 44", true, "PGM CH 7?"},
    {"AB 09 00 00 08 43", true, "PGM CH 8?"},
    // NEXT CH
    {"AB 0B 00 00 00 49", true, "NEXT CH OFF"},
    {"AB 0B 00 00 02 47", true, "NEXT CH 2"},
    {"AB 0B 00 00 06 43", true, "NEXT CH 6"},
    {"AB 0B 00 00 07 42", true, "NEXT CH 7?"},
    {"AB 0B 00 00 08 41", true, "NEXT CH 8?"},
    // DEFAULT
    {"AB D0 00 00 01 83", true, "DEFAULT"},
    // STATUS 요청 / 응답
    {"AB D2 00 00 01 81", true, "STATUS REQ PGM/PVW"},
    {"AA D2 55 01 02 2A", true, "STATUS PGM=CH1 PVW=CH2"},
    {"AB D3 00 00 01 80", true, "STATUS REQ EFFECT/CKEY/PIP/DSK/AUDIO"},
    // README 9장의 체크섬 오류 예시 줄
    {"AA D3 55 02 04 7C", false, "STATUS EFFECT=1 INV=OFF CG=OFF PIP=ON DSK=OFF AUDIO_EXT=OFF"},
    // NMX-1108 미확인 영역: 0x40 이 OFF 인지 CH7 인지 로그에 남긴다
    {"AA D2 55 40 40 AD", true, "STATUS PGM=OFF|CH7? PVW=OFF|CH7?"},
    {"AA D2 55 80 80 2D", true, "STATUS PGM=CH8? PVW=CH8?"},
    // 알 수 없는 CMD 도 반드시 출력되어야 한다
    {"AB 7E 00 00 00 00", false, "UNKNOWN CMD 0x7E"},
};

void parseHex(const char* s, uint8_t* out) {
    for (size_t i = 0; i < FRAME_LEN; ++i) {
        unsigned v = 0;
        sscanf(s + i * 3, "%2x", &v);
        out[i] = static_cast<uint8_t>(v);
    }
}

}  // namespace

int runSelfTest() {
    int failures = 0;
    const size_t total = sizeof(kCases) / sizeof(kCases[0]);

    Serial.println(F("--- decoder selftest ---"));
    for (size_t i = 0; i < total; ++i) {
        const Case& c = kCases[i];
        uint8_t frame[FRAME_LEN];
        parseHex(c.hex, frame);

        const bool ok = isChecksumOk(frame);
        char desc[160];
        describeFrame(frame, desc, sizeof(desc));

        const bool chkMatch = (ok == c.chkOk);
        const bool descMatch = (strcmp(desc, c.expectDesc) == 0);
        if (chkMatch && descMatch) {
            continue;
        }
        failures++;
        Serial.printf("FAIL %s\n", c.hex);
        if (!chkMatch) {
            Serial.printf("     CHK=%s, expected %s\n", ok ? "OK" : "BAD", c.chkOk ? "OK" : "BAD");
        }
        if (!descMatch) {
            Serial.printf("     got  \"%s\"\n     want \"%s\"\n", desc, c.expectDesc);
        }
    }

    Serial.printf("%s: %u/%u cases passed\n", failures ? "SELFTEST FAILED" : "SELFTEST PASSED",
                  static_cast<unsigned>(total - failures), static_cast<unsigned>(total));
    return failures;
}

}  // namespace nmx
