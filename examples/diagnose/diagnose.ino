// One-shot health check for a TUF-2000M link. Runs the checks in the order things usually go wrong
// and stops at the first failure with a hint, so you know what to fix next. Reads the same four
// register blocks the library polls (Tuf2000FlowMeter::Config defaults) but with plain eModbus,
// so it works even when basic.ino only says "poll failed". Pins are an example wiring.
#include <Tuf2000.h>

static constexpr int8_t kRxPin = 17;  // ESP32 RX <- converter TXD
static constexpr int8_t kTxPin = 16;  // ESP32 TX -> converter RXD
static constexpr int8_t kDePin = -1;  // Set to the DE/RE pin if your transceiver needs it.
static constexpr uint8_t kSlaveId = 1;
static constexpr uint32_t kBaud = 9600;

using tuf2000::Tuf2000PollTracker;

ModbusClientRTU client(kDePin);
static const tuf2000::Tuf2000FlowMeter::Config registers;  // Defaults only: the register map.

static bool failed = false;

static void pass(const char* what) { Serial.printf("[ OK ] %s\n", what); }

static void fail(const char* what, const char* hint) {
    Serial.printf("[FAIL] %s\n       -> %s\n", what, hint);
    failed = true;
}

static uint16_t startRegisterOf(Tuf2000PollTracker::Field field) {
    switch (field) {
        case Tuf2000PollTracker::kFlowRate:
            return registers.flowRateRegister;
        case Tuf2000PollTracker::kTotalizer:
            return registers.totalizerRegister;
        case Tuf2000PollTracker::kSignalQuality:
            return registers.signalQualityRegister;
        case Tuf2000PollTracker::kSoundSpeed:
            return registers.soundSpeedRegister;
    }
    return 0;
}

// Reads one field into `words` (as many as the field needs). Reports and returns false on failure.
static bool readField(Tuf2000PollTracker::Field field, const char* name, uint16_t* words) {
    const uint16_t count = Tuf2000PollTracker::wordCountOf(field);
    ModbusMessage reply = client.syncRequest(static_cast<uint32_t>(field), kSlaveId,
                                                   READ_HOLD_REGISTER, startRegisterOf(field), count);
    const Modbus::Error error = reply.getError();

    char what[48];
    if (error == TIMEOUT) {
        snprintf(what, sizeof(what), "%s: any reply from slave %u", name, kSlaveId);
        fail(what, "No reply. Check A/B (try swapping), 120 ohm termination, converter power, "
                   "meter M63=MODBUS RTU, M62=9600 8N1, M46 = slave ID. Try examples/bus_scan.");
        return false;
    }
    if (error != SUCCESS) {
        snprintf(what, sizeof(what), "%s: clean Modbus reply", name);
        fail(what, static_cast<const char*>(ModbusError(error)));
        return false;
    }
    const uint8_t byteCount = reply.size() > 2 ? reply[2] : 0;
    if (!tuf2000::replyLengthMatches(reply.size(), byteCount, count)) {
        snprintf(what, sizeof(what), "%s: reply length", name);
        fail(what, "Truncated or padded reply; usually a noisy bus or a wrong register map.");
        return false;
    }
    uint16_t index = 3;  // [0]=slave ID, [1]=function code, [2]=byte count, then data.
    for (uint16_t i = 0; i < count; ++i) {
        index = reply.get(index, words[i]);
    }
    snprintf(what, sizeof(what), "%s: valid reply", name);
    pass(what);
    return true;
}

static bool readFloat(Tuf2000PollTracker::Field field, const char* name, float& out) {
    uint16_t words[3] = {};
    if (!readField(field, name, words)) {
        return false;
    }
    if (!tuf2000::decodeFloatLowWordFirst(words[0], words[1], out)) {
        char what[48];
        snprintf(what, sizeof(what), "%s: finite number", name);
        fail(what, "NaN/infinity: wrong register map for this firmware. Try examples/register_dump.");
        return false;
    }
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("TUF-2000M link diagnosis");

    RTUutils::prepareHardwareSerial(Serial2);
    Serial2.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin);
    client.setTimeout(1000);
    client.begin(Serial2);
    pass("UART and Modbus client started");

    float flow = 0.0f;
    float total = 0.0f;
    float sound = 0.0f;
    uint16_t signal[3] = {};

    if (readFloat(Tuf2000PollTracker::kFlowRate, "flow rate", flow) &&
        readFloat(Tuf2000PollTracker::kTotalizer, "totalizer", total) &&
        readField(Tuf2000PollTracker::kSignalQuality, "signal block", signal) &&
        readFloat(Tuf2000PollTracker::kSoundSpeed, "sound speed", sound)) {
        Serial.printf("\nflow %.3f m3/h, total %.3f m3, sound %.1f m/s, quality %u, autogain step "
                      "%u, strength up/down %u/%u\n\n",
                      flow, total, sound, tuf2000::signalQualityValue(signal[0]),
                      tuf2000::signalAutogainStep(signal[0]), signal[1], signal[2]);

        if (total >= 0.0f) {
            pass("totalizer is not negative");
        } else {
            fail("totalizer is not negative", "A positive accumulator is never negative.");
        }
        if (sound > 1000.0f && sound < 2000.0f) {
            pass("sound speed is plausible for a liquid (1000-2000 m/s)");
        } else {
            fail("sound speed is plausible for a liquid (1000-2000 m/s)",
                 "Odd value: check M20 liquid type, or the word order of the registers.");
        }
        if (tuf2000::signalQualityValue(signal[0]) >= 60) {
            pass("signal quality is 60 or better");
        } else {
            fail("signal quality is 60 or better",
                 "Reposition the transducers with M90, check pipe settings M11-M25 and coupling.");
        }
    }

    Serial.println(failed ? "\nProblems found; fix the first [FAIL] above and run again."
                          : "\nAll checks passed.");
}

void loop() {}
