// Simplest use: a blocking read once every few seconds. Fine for a bench sketch; for anything that
// must stay responsive see examples/async. Pins are an example wiring - change them.
#include <Tuf2000.h>

tuf2000::Tuf2000FlowMeter meter;

void setup() {
    Serial.begin(115200);

    tuf2000::Tuf2000FlowMeter::Config cfg;
    cfg.rxPin = 17;   // ESP32 RX <- converter TXD
    cfg.txPin = 16;   // ESP32 TX -> converter RXD
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
