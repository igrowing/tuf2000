// Finds a Modbus RTU device whose settings you do not know: tries every slave ID (1-247) at each
// common baud rate (8N1) and prints whoever answers. Use it when basic.ino only reports timeouts.
// Any reply at all - even an error frame - proves the wiring works; a clean reply gives you the
// ID and baud to put into Tuf2000FlowMeter::Config. A full scan takes a few minutes per baud rate.
// Pins are an example wiring - change them.
#include <Tuf2000.h>

static constexpr int8_t kRxPin = 17;  // ESP32 RX <- converter TXD
static constexpr int8_t kTxPin = 16;  // ESP32 TX -> converter RXD
static constexpr int8_t kDePin = -1;  // Set to the DE/RE pin if your transceiver needs it.
static constexpr uint32_t kReplyTimeoutMs = 250;
// Meter factory default first. Parity other than "none" is not scanned; see meter menu M62.
static constexpr uint32_t kBauds[] = {9600, 19200, 4800, 38400, 2400, 57600, 115200, 1200};

// Reads the first two registers (the TUF-2000M flow rate) from every slave ID at one baud rate.
static uint8_t scanAtBaud(uint32_t baud) {
    RTUutils::prepareHardwareSerial(Serial2);
    Serial2.begin(baud, SERIAL_8N1, kRxPin, kTxPin);

    ModbusClientRTU client(kDePin);
    client.setTimeout(kReplyTimeoutMs);
    client.begin(Serial2);

    uint8_t found = 0;
    for (uint16_t id = 1; id <= 247; ++id) {
        const ModbusMessage reply =
            client.syncRequest(id, static_cast<uint8_t>(id), READ_HOLD_REGISTER, 0, 2);
        const Modbus::Error error = reply.getError();
        if (error == TIMEOUT) {
            continue;  // Nobody there: the normal case for almost every ID.
        }
        ++found;
        if (error == SUCCESS) {
            Serial.printf("  slave ID %u answers at %lu baud\n", static_cast<unsigned>(id),
                          static_cast<unsigned long>(baud));
        } else {
            // Something put bytes on the bus but the frame was not a clean reply. At the wrong
            // baud rate this is what a real device sounds like.
            Serial.printf("  slave ID %u: garbled or error reply at %lu baud (%s)\n",
                          static_cast<unsigned>(id), static_cast<unsigned long>(baud),
                          static_cast<const char*>(ModbusError(error)));
        }
    }
    Serial2.end();
    return found;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Modbus RTU bus scan, 8N1");

    uint8_t total = 0;
    for (const uint32_t baud : kBauds) {
        Serial.printf("Scanning at %lu baud...\n", static_cast<unsigned long>(baud));
        total += scanAtBaud(baud);
    }

    if (total == 0) {
        Serial.println("Nothing answered. Check: A/B not swapped, 120 ohm termination, converter "
                       "powered, meter M63 = MODBUS RTU (not ASCII), M62 serial settings.");
    } else {
        Serial.println("Scan done. A clean 'answers' line is the ID and baud to use.");
    }
}

void loop() {}
