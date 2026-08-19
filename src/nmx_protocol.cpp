#include "nmx_protocol.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace nmx {
namespace {

// out의 끝에 이어붙인다. 버퍼가 가득 차면 조용히 무시한다.
void appendf(char* out, size_t outLen, const char* fmt, ...) {
    const size_t used = strlen(out);
    if (used + 1 >= outLen) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(out + used, outLen - used, fmt, ap);
    va_end(ap);
}

inline unsigned u(uint32_t v) {
    return static_cast<unsigned>(v);
}

// NMX-1106 문서에 이름이 적혀 있는 이펙트만 부가 설명을 붙인다.
const char* effectName(uint8_t n) {
    switch (n) {
        case 1: return "LEFT/RIGHT";
        case 2: return "UP/DOWN";
        case 7: return "FOUT/IN";
        case 8: return "MIX";
        default: return nullptr;
    }
}

void describeSetup(uint8_t d3, char* out, size_t outLen) {
    switch (d3) {
        case 0x06: appendf(out, outLen, "SETUP ENTER(MENU)"); break;
        case 0x08: appendf(out, outLen, "SETUP ESC"); break;
        // 원문 문서에서 0x00/0x01 라벨이 깨져 있어 방향키로 추정한다.
        case 0x01: appendf(out, outLen, "SETUP UP?"); break;
        case 0x00: appendf(out, outLen, "SETUP DOWN?"); break;
        case 0x03: appendf(out, outLen, "SETUP LEFT (sub menu -)"); break;
        case 0x02: appendf(out, outLen, "SETUP RIGHT (sub menu +)"); break;
        default: appendf(out, outLen, "SETUP KEY 0x%02X ?", u(d3)); break;
    }
}

void describeEffect(uint8_t d3, char* out, size_t outLen) {
    const bool inv = (d3 & 0x10) != 0;
    const uint8_t n = d3 & 0x0F;
    if (n >= 1 && n <= 8 && (d3 & 0xE0) == 0) {
        appendf(out, outLen, "EFFECT %u%s", u(n), inv ? " INV" : "");
        const char* name = effectName(n);
        if (name != nullptr) {
            appendf(out, outLen, " (%s)", name);
        }
    } else {
        appendf(out, outLen, "EFFECT RAW 0x%02X ?", u(d3));
    }
}

void describeOverlay(uint8_t d3, char* out, size_t outLen) {
    appendf(out, outLen, "PGM OVERLAY");
    if (d3 == 0x00) {
        appendf(out, outLen, " ALL OFF");
        return;
    }
    // bit0 = CKEY, bit1 = DSK, bit2 = PIP
    if (d3 & 0x01) appendf(out, outLen, " CKEY");
    if (d3 & 0x02) appendf(out, outLen, " DSK");
    if (d3 & 0x04) appendf(out, outLen, " PIP");
    if (d3 & 0xF8) appendf(out, outLen, " +0x%02X?", u(d3 & 0xF8));
}

// PGM CH / NEXT CH 는 채널 번호를 그대로 싣는다. CH7/CH8은 NMX-1108 추정.
void describeChannelSelect(const char* label, uint8_t d3, char* out, size_t outLen) {
    if (d3 == 0x00) {
        appendf(out, outLen, "%s OFF", label);
    } else if (d3 <= 6) {
        appendf(out, outLen, "%s %u", label, u(d3));
    } else if (d3 <= 8) {
        appendf(out, outLen, "%s %u?", label, u(d3));  // NMX-1108 확장 추정
    } else {
        appendf(out, outLen, "%s RAW 0x%02X ?", label, u(d3));
    }
}

void describeStatusPgmPvw(bool response, const uint8_t* f, char* out, size_t outLen) {
    if (!response) {
        appendf(out, outLen, "STATUS REQ PGM/PVW");
        return;
    }
    char pgm[32] = {0};
    char pvw[32] = {0};
    formatChannelMask(f[3], pgm, sizeof(pgm));
    formatChannelMask(f[4], pvw, sizeof(pvw));
    appendf(out, outLen, "STATUS PGM=%s PVW=%s", pgm, pvw);
    if (f[2] != 0x55) {
        appendf(out, outLen, " (mark=0x%02X?)", u(f[2]));
    }
}

void describeStatusEffect(bool response, const uint8_t* f, char* out, size_t outLen) {
    if (!response) {
        appendf(out, outLen, "STATUS REQ EFFECT/CKEY/PIP/DSK/AUDIO");
        return;
    }
    // BYTE_3: bit0 = EFFECT INV, bit1..bit7 = EFFECT 1..7
    // BYTE_4: bit0 = EFFECT 8, bit1 = CG, bit2 = PIP, bit3 = DSK, bit6 = EXT AUDIO
    const uint8_t eff = f[3];
    const uint8_t flags = f[4];

    appendf(out, outLen, "STATUS EFFECT=");
    bool any = false;
    for (uint8_t bit = 1; bit <= 7; ++bit) {
        if (eff & (1u << bit)) {
            appendf(out, outLen, "%s%u", any ? "," : "", u(bit));
            any = true;
        }
    }
    if (flags & D3_EFFECT_8) {
        appendf(out, outLen, "%s8", any ? "," : "");
        any = true;
    }
    if (!any) {
        appendf(out, outLen, "none");
    }

    appendf(out, outLen, " INV=%s", (eff & D3_EFFECT_INV) ? "ON" : "OFF");
    appendf(out, outLen, " CG=%s", (flags & D3_CG) ? "ON" : "OFF");
    appendf(out, outLen, " PIP=%s", (flags & D3_PIP) ? "ON" : "OFF");
    appendf(out, outLen, " DSK=%s", (flags & D3_DSK) ? "ON" : "OFF");
    appendf(out, outLen, " AUDIO_EXT=%s", (flags & D3_EXT_AUDIO) ? "ON" : "OFF");

    // 문서상 fixed 0 인 bit4/bit5가 켜져 있으면 NMX-1108 확장 신호일 수 있다.
    if (flags & 0x30) {
        appendf(out, outLen, " (fixed-0 bits set: 0x%02X?)", u(flags & 0x30));
    }
    if (f[2] != 0x55) {
        appendf(out, outLen, " (mark=0x%02X?)", u(f[2]));
    }
}

}  // namespace

bool isKnownCmd(uint8_t cmd) {
    return cmdName(cmd) != nullptr;
}

const char* cmdName(uint8_t cmd) {
    switch (cmd) {
        case CMD_SETUP: return "SETUP";
        case CMD_AUDIO_EXT: return "AUDIO_EXT";
        case CMD_EFFECT: return "EFFECT";
        case CMD_FADER_MODE: return "FADER_MODE";
        case CMD_FADER_VALUE: return "FADER";
        case CMD_PGM_OVERLAY: return "PGM_OVERLAY";
        case CMD_PGM_CH: return "PGM_CH";
        case CMD_NEXT_CH: return "NEXT_CH";
        case CMD_DEFAULT: return "DEFAULT";
        case CMD_STATUS_PGM_PVW: return "STATUS_PGM_PVW";
        case CMD_STATUS_EFFECT: return "STATUS_EFFECT";
        default: return nullptr;
    }
}

void formatChannelMask(uint8_t value, char* out, size_t outLen) {
    if (outLen == 0) {
        return;
    }
    out[0] = '\0';

    if (value == 0x00) {
        appendf(out, outLen, "NONE");
        return;
    }
    // NMX-1106에서 0x40은 OFF. NMX-1108은 CH7 비트일 가능성이 있어 양쪽을 표기한다.
    if (value == CH_MASK_OFF) {
        appendf(out, outLen, "OFF|CH7?");
        return;
    }
    if (value == 0x80) {
        appendf(out, outLen, "CH8?");
        return;
    }

    bool any = false;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        if ((value & (1u << bit)) == 0) {
            continue;
        }
        // bit6/bit7은 NMX-1106 문서에 없는 영역이라 '?'로 표시한다.
        appendf(out, outLen, "%sCH%u%s", any ? "+" : "", u(bit + 1), bit >= 6 ? "?" : "");
        any = true;
    }
}

void formatHex(const uint8_t* data, size_t len, char* out, size_t outLen) {
    if (outLen == 0) {
        return;
    }
    out[0] = '\0';
    for (size_t i = 0; i < len; ++i) {
        appendf(out, outLen, "%s%02X", i ? " " : "", u(data[i]));
    }
}

void describeFrame(const uint8_t frame[FRAME_LEN], char* out, size_t outLen) {
    if (outLen == 0) {
        return;
    }
    out[0] = '\0';

    const bool response = (frame[0] == HEADER_RESPONSE);
    const uint8_t cmd = frame[1];
    const uint8_t d2 = frame[3];
    const uint8_t d3 = frame[4];

    switch (cmd) {
        case CMD_SETUP:
            describeSetup(d3, out, outLen);
            break;
        case CMD_AUDIO_EXT:
            appendf(out, outLen, "AUDIO SELECT EXT %s", d3 ? "ON" : "OFF");
            break;
        case CMD_EFFECT:
            describeEffect(d3, out, outLen);
            break;
        case CMD_FADER_MODE:
            if (d3 == 0x01) {
                appendf(out, outLen, "FADER MODE TAKE");
            } else if (d3 == 0x02) {
                appendf(out, outLen, "FADER MODE CUT");
            } else {
                appendf(out, outLen, "FADER MODE 0x%02X ?", u(d3));
            }
            break;
        case CMD_FADER_VALUE: {
            // D2:D3 가 16비트 페이더 값 (문서 예: 0x000 ~ 0x1FF). 0이면 OFF.
            const uint16_t value = static_cast<uint16_t>(static_cast<uint16_t>(d2) << 8 | d3);
            if (value == 0) {
                appendf(out, outLen, "FADER OFF");
            } else {
                appendf(out, outLen, "FADER 0x%03X (%u)", u(value), u(value));
            }
            break;
        }
        case CMD_PGM_OVERLAY:
            describeOverlay(d3, out, outLen);
            break;
        case CMD_PGM_CH:
            describeChannelSelect("PGM CH", d3, out, outLen);
            break;
        case CMD_NEXT_CH:
            describeChannelSelect("NEXT CH", d3, out, outLen);
            break;
        case CMD_DEFAULT:
            appendf(out, outLen, "DEFAULT");
            break;
        case CMD_STATUS_PGM_PVW:
            describeStatusPgmPvw(response, frame, out, outLen);
            break;
        case CMD_STATUS_EFFECT:
            describeStatusEffect(response, frame, out, outLen);
            break;
        default:
            appendf(out, outLen, "UNKNOWN CMD 0x%02X", u(cmd));
            break;
    }
}

}  // namespace nmx
