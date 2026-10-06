// Dumps a block of holding registers, showing each one raw and as the 32-bit float it would start,
// read both ways round. Use it to see what the meter really sends, or to check another firmware's
// register map: a plausible value (flow rate, sound speed ~1480 for water) tells you the right
// word order and alignment. The TUF-2000M sends low word first, so look at the "low-first" column.
// Pins are an example wiring - change them.
#include <Tuf2000.h>

static constexpr int8_t kRxPin = 17;  // ESP32 RX <- converter TXD
static constexpr int8_t kTxPin = 16;  // ESP32 TX -> converter RXD
static constexpr int8_t kDePin = -1;  // Set to the DE/RE pin if your transceiver needs it.
static constexpr uint8_t kSlaveId = 1;
static constexpr uint32_t kBaud = 9600;

// Register numbers as printed in the meter manual; the Modbus address on the wire is one less.
static constexpr uint16_t kFirstRegister = 1;
static constexpr uint16_t kRegisterCount = 16;
static constexpr uint16_t kMaxWordsPerRequest = 8;  // Keep requests small; the meter is slow.
static constexpr uint32_t kRepeatMs = 10000;

ModbusClientRTU client(kDePin);

static void printFloat(uint16_t lowWordReg, uint16_t highWordReg, bool haveBoth) {
    float value = 0.0f;
    if (haveBoth && tuf2000::decodeFloatLowWordFirst(lowWordReg, highWordReg, value)) {
        Serial.printf("%17.4f", value);
    } else {
        Serial.printf("%17s", "-");
    }
}

static bool readBlock(uint16_t firstRegister, uint16_t count, uint16_t* out) {
    for (uint16_t done = 0; done < count; done += kMaxWordsPerRequest) {
        const uint16_t words = min<uint16_t>(kMaxWordsPerRequest, count - done);
        const uint16_t address = firstRegister - 1 + done;
        ModbusMessage reply =
            client.syncRequest(done, kSlaveId, READ_HOLD_REGISTER, address, words);
        const Modbus::Error error = reply.getError();
        if (error != SUCCESS) {
            Serial.printf("REG%04u..%04u: %s\n", firstRegister + done,
                          firstRegister + done + words - 1,
                          static_cast<const char*>(ModbusError(error)));
            return false;
        }
        const uint8_t byteCount = reply.size() > 2 ? reply[2] : 0;
        if (!tuf2000::replyLengthMatches(reply.size(), byteCount, words)) {
            Serial.printf("REG%04u..%04u: reply of wrong length\n", firstRegister + done,
                          firstRegister + done + words - 1);
            return false;
        }
        uint16_t index = 3;  // [0]=slave ID, [1]=function code, [2]=byte count, then data.
        for (uint16_t i = 0; i < words; ++i) {
            index = reply.get(index, out[done + i]);
        }
    }
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    RTUutils::prepareHardwareSerial(Serial2);
    Serial2.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin);
    client.setTimeout(1000);
    client.begin(Serial2);
}

void loop() {
    uint16_t regs[kRegisterCount] = {};
    Serial.printf("\nSlave %u, registers %u..%u\n", kSlaveId, kFirstRegister,
                  kFirstRegister + kRegisterCount - 1);

    if (readBlock(kFirstRegister, kRegisterCount, regs)) {
        Serial.println("register     raw   u16  REAL4 low-first REAL4 high-first");
        for (uint16_t i = 0; i < kRegisterCount; ++i) {
            const bool haveNext = i + 1 < kRegisterCount;
            const uint16_t next = haveNext ? regs[i + 1] : 0;
            Serial.printf("REG%04u   0x%04X %5u", kFirstRegister + i, regs[i], regs[i]);
            printFloat(regs[i], next, haveNext);  // This register is the low word.
            printFloat(next, regs[i], haveNext);  // This register is the high word.
            Serial.println();
        }
    }
    delay(kRepeatMs);
}
