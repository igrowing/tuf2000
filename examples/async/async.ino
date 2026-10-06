// Polls a TUF-2000M over RS-485 and prints each reading. Pins below are an example wiring for a
// generic ESP32 dev board - change them to match yours.
#include <Tuf2000.h>

static constexpr int8_t kRxPin = 17;
static constexpr int8_t kTxPin = 16;
static constexpr int8_t kDePin = -1;  // Set to the DE/RE pin if your transceiver needs it.

tuf2000::Tuf2000FlowMeter meter;

static void logLine(const char* message) { Serial.println(message); }

void setup() {
    Serial.begin(115200);

    tuf2000::Tuf2000FlowMeter::Config cfg;
    cfg.rxPin = kRxPin;
    cfg.txPin = kTxPin;
    cfg.dePin = kDePin;
    cfg.slaveId = 1;
    cfg.baudRate = 9600;
    cfg.pollIntervalMs = 5000;

    meter.setLogger(logLine);
    if (!meter.begin(Serial2, cfg)) {
        Serial.println("meter.begin() failed");
    }
}

void loop() {
    meter.readAsync(millis());

    tuf2000::Tuf2000FlowMeter::Reading r;
    if (meter.takeReading(r)) {
        if (r.valid) {
            Serial.printf("flow %.3f m3/h, total %.3f m3, quality %u, sound %.1f m/s\n",
                          r.flowRateM3h, r.totalizerM3, tuf2000::signalQualityValue(r.signalQuality),
                          r.soundSpeedMs);
        } else {
            Serial.printf("poll failed: %s\n", tuf2000::tuf2000FailureText(r.failure));
        }
    }
}
