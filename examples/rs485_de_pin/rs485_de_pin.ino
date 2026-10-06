// Same as basic.ino, for a plain RS-485 transceiver (MAX485, SP3485, ...) that needs a
// direction-control pin. Wire DE and /RE of the transceiver together to kDePin: eModbus drives it
// high just before sending and low again so the meter reply can be received. Converters with
// automatic direction switching do not need this - use basic.ino. Pins are an example wiring.
#include <Tuf2000.h>

static constexpr int8_t kRxPin = 17;  // ESP32 RX <- transceiver RO
static constexpr int8_t kTxPin = 16;  // ESP32 TX -> transceiver DI
static constexpr int8_t kDePin = 4;   // Transceiver DE and /RE tied together.

tuf2000::Tuf2000FlowMeter meter;

void setup() {
    Serial.begin(115200);

    tuf2000::Tuf2000FlowMeter::Config cfg;
    cfg.rxPin = kRxPin;
    cfg.txPin = kTxPin;
    cfg.dePin = kDePin;
    cfg.slaveId = 1;  // must equal the meter's M46

    if (!meter.begin(Serial2, cfg)) {
        Serial.println("meter.begin() failed");
    }
}

void loop() {
    tuf2000::Tuf2000FlowMeter::Reading r;
    if (!meter.read(3000, r)) {
        Serial.println("no reply within 3 s");
    } else if (r.valid) {
        Serial.printf("flow %.3f m3/h, total %.3f m3\n", r.flowRateM3h, r.totalizerM3);
    } else {
        Serial.printf("poll failed: %s\n", tuf2000::tuf2000FailureText(r.failure));
    }
    delay(5000);  // read() starts a new poll only once cfg.pollIntervalMs (default 5 s) has passed
}
