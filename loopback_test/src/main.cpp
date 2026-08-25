#include <Arduino.h>

namespace {
constexpr uint32_t BAUD = 9600;
constexpr uint32_t LOOP_MS = 2000;

void runSinglePort(const char* label, HardwareSerial& serial, int rxPin, int txPin) {
    Serial.printf("[%s] begin RX=%d TX=%d\n", label, rxPin, txPin);
    serial.begin(BAUD, SERIAL_8N1, rxPin, txPin);
    delay(100);

    serial.println("HELLO");

    const uint32_t t0 = millis();
    String received = "";
    while (millis() - t0 < LOOP_MS) {
        while (serial.available()) {
            received += static_cast<char>(serial.read());
        }
        if (received.length() > 0) {
            break;
        }
        delay(10);
    }

    if (received.length() == 0) {
        Serial.printf("[%s] FAIL: no echo received\n", label);
    } else {
        Serial.printf("[%s] received: %s\n", label, received.c_str());
    }

    serial.end();
    delay(100);
}

void runDualPort() {
    Serial.println("Two-port loopback test");
    Serial.println("Port1: RX=16 TX=17");
    Serial.println("Port2: RX=26 TX=27");

    Serial1.begin(BAUD, SERIAL_8N1, 16, 17);
    Serial2.begin(BAUD, SERIAL_8N1, 26, 27);
    delay(100);

    Serial1.println("P1->P1");
    Serial2.println("P2->P2");

    const uint32_t t0 = millis();
    String p1 = "";
    String p2 = "";

    while (millis() - t0 < LOOP_MS) {
        while (Serial1.available()) {
            p1 += static_cast<char>(Serial1.read());
        }
        while (Serial2.available()) {
            p2 += static_cast<char>(Serial2.read());
        }
        if (p1.length() > 0 || p2.length() > 0) {
            break;
        }
        delay(10);
    }

    if (p1.length()) {
        Serial.printf("Port1 echo: %s\n", p1.c_str());
    } else {
        Serial.println("Port1: no echo");
    }

    if (p2.length()) {
        Serial.printf("Port2 echo: %s\n", p2.c_str());
    } else {
        Serial.println("Port2: no echo");
    }

    Serial1.end();
    Serial2.end();
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.println();
    Serial.println("=================================================");
    Serial.println("ESP32 MAX3232 loopback validation");
    Serial.println("Connect each MAX3232 TTL loopback as TIN <-> ROUT");
    Serial.println("Keep GND common and press any key to start");
    Serial.println("=================================================");

    while (!Serial.available()) {
        delay(10);
    }
    while (Serial.available()) {
        Serial.read();
    }

    runSinglePort("Serial1", Serial1, 16, 17);
    runSinglePort("Serial2", Serial2, 26, 27);
    runDualPort();

    Serial.println("Loopback test complete. Reset board to repeat.");
}

void loop() {
    delay(1000);
}
