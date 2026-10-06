#pragma once

#include <Arduino.h>
// Note for whatever includes this header: eModbus's RTUutils.h (pulled in transitively below) does
// a file-scope `using namespace Modbus;`, which is therefore also visible wherever this header is
// included - a known quirk of eModbus itself, not something this driver adds deliberately.
#include <ModbusClientRTU.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <memory>

#include "Tuf2000Protocol.h"

namespace tuf2000 {

// Modbus RTU driver for the TUF-2000 ultrasonic clamp-on flow meter. Named after the specific
// meter model (not a generic flow meter) because the register map/decode logic is
// TUF-2000-specific.
//
// This is a thin eModbus-facing wrapper: the poll-sequencing/staleness state machine and the
// register decode live in tuf2000::Tuf2000PollTracker/decodeFloatLowWordFirst (Tuf2000Protocol.*),
// which have no Arduino/eModbus dependency and are unit tested on the host. This class only glues
// that logic to eModbus's request/callback API.
//
// Everything board- or application-specific (pins, baud, slave ID, register map, timings, log
// output) comes in through Config / setLogger(); readings go out as a plain struct through
// responseReady()/takeReading().
//
// eModbus's ModbusClientRTU is async-only by design: it runs its own FreeRTOS worker task and
// queues/serializes requests internally, delivering results via onDataHandler/onErrorHandler
// callbacks rather than a blocking call, so this driver needs no FreeRTOS task of its own. This
// driver exposes that async engine (readAsync/responseReady/takeReading) plus a thin blocking
// convenience (read()) built on top of it, rather than two separate implementations.
class Tuf2000FlowMeter {
public:
    struct Config {
        // UART pins. There is no sensible default: the caller must set both.
        int8_t rxPin = -1;
        int8_t txPin = -1;
        // RS-485 driver-enable (DE/RE) pin, driven by eModbus around each transmission. -1 = none,
        // for transceivers that switch direction automatically.
        int8_t dePin = -1;
        uint32_t baudRate = 9600;
        uint8_t slaveId = 1;
        // Register map (function code 03, protocol addresses = the manual's register number minus
        // one). Defaults match the TUF-2000 manual: flow rate REG1-2, fluid sound speed REG7-8,
        // signal block REG92-94, positive totalizer (REAL4, m3) REG115-116.
        // flowRateRegister/totalizerRegister/soundSpeedRegister each span 2 words
        // (REAL4, low word first - see decodeFloatLowWordFirst()).
        // signalQualityRegister starts a 3-word block: quality, then upstream and downstream signal
        // strength in the two registers right after it.
        uint16_t flowRateRegister = 0;
        uint16_t totalizerRegister = 114;
        uint16_t signalQualityRegister = 91;
        uint16_t soundSpeedRegister = 6;
        uint32_t pollIntervalMs = 5000;
        // A poll still in flight after this long is abandoned so the next one can start - the
        // stuck-request watchdog, a backstop well beyond eModbus's own per-request timeout.
        uint32_t stuckRequestTimeoutMs = 8000;
        // Multiplied into flowRateM3h and totalizerM3 as they are decoded (1.0 = meter's own
        // values); corrects a meter whose calibration is off.
        float calibrationMultiplier = 1.0f;
        // Largest forward move of totalizerM3 (calibrated) accepted between two polls - see
        // TotalizerJumpGuard. A poll exceeding it is rejected like a Modbus failure.
        float maxTotalizerJumpM3 = 10.0f;
    };

    struct Reading {
        float flowRateM3h = 0.0f;
        float totalizerM3 = 0.0f;
        // Raw quality word: signalQualityValue()/signalAutogainStep() split it.
        uint16_t signalQuality = 0;
        uint16_t signalStrengthUp = 0;
        uint16_t signalStrengthDown = 0;
        float soundSpeedMs = 0.0f;
        uint32_t timestampMs = 0;
        // false when any of the four requests polled this cycle failed or returned something that
        // cannot be trusted (Modbus error/timeout, wrong reply length, NaN/infinity, an implausible
        // totalizer jump) - pushed through the same responseReady()/takeReading() path as a real
        // reading, so callers can tell "polled and it failed" apart from "nothing new yet" rather
        // than errors just vanishing. `failure` says which, and is kNone exactly when valid.
        bool valid = false;
        tuf2000::Tuf2000Failure failure = tuf2000::Tuf2000Failure::kNone;
    };

    // Receives one short, NUL-terminated message per call. May run on eModbus's worker task as
    // well as the caller's, so keep it quick and non-blocking. The text is only valid during the
    // call.
    using LogCallback = void (*)(const char* message);

    // Optional. Call before begin() to see failures and start-up errors; without it the library
    // logs nothing.
    void setLogger(LogCallback logger) { logger_ = logger; }

    // Configures the UART on cfg's RX/TX pins at cfg.baudRate and constructs the ModbusClientRTU
    // worker bound to it. Call once from setup(). Returns false (and logs) if cfg is unusable -
    // pins unset or a zero poll interval - in which case nothing is started.
    bool begin(HardwareSerial& serial, const Config& cfg);

    // Non-blocking; throttles to cfg.pollIntervalMs and no-ops while a poll is already in flight -
    // requests are never overlapped with a NEW poll's, which directly targets the reference
    // project's (github.com/tczerwonka/tuf2000-modbus-reader) "the modbus stuff just didn't give
    // back consistent data for two sequential queries" failure. Call every loop() tick.
    void readAsync(uint32_t nowMs);

    // True once a fresh Reading (success or failure) has landed since the last takeReading().
    bool responseReady() const;

    // Copies the latest Reading out and clears responseReady(). Returns false if nothing was ready.
    bool takeReading(Reading& out);

    // Blocking convenience: calls readAsync() then spins on takeReading() until it succeeds or
    // timeoutMs elapses. For one-off "read now" uses (a bench test sketch, a future MQTT command) -
    // never call this from loop() or anything loop() calls.
    bool read(uint32_t timeoutMs, Reading& out);

    // Most recent Modbus error seen on any request (Modbus::Error value, 0 = SUCCESS = none since
    // boot). Sticky: a later successful poll does NOT clear it, so an intermittent fault stays
    // diagnosable after the fact. Written on eModbus's worker task, read from loop() - a single
    // byte, so the volatile is enough.
    uint8_t lastErrorCode() const { return lastErrorCode_; }
    // Human-readable text for lastErrorCode() (a static string, never null).
    const char* lastErrorText() const;

    // Why the most recent failed poll was rejected, sticky like lastErrorCode(): it also covers the
    // failures that are not Modbus errors at all (kNone = no failed poll since boot).
    tuf2000::Tuf2000Failure lastFailure() const {
        return static_cast<tuf2000::Tuf2000Failure>(lastFailure_);
    }

private:
    void log(const char* msg) const {
        if (logger_ != nullptr) {
            logger_(msg);
        }
    }
    void recordError(Modbus::Error error);
    void issueRequest(uint32_t pollSeq, tuf2000::Tuf2000PollTracker::Field field,
                      uint16_t startRegister);
    void handleData(ModbusMessage& msg, uint32_t token);
    void handleError(Modbus::Error error, uint32_t token);
    // Common "one field of the current poll is done" bookkeeping shared by handleData()/
    // handleError(): delegates to pollTracker_ and, once every field has reported in, publishes
    // scratch_ to resultQueue_. Runs on eModbus's worker task.
    void finishField(uint32_t token, bool ok);
    // Reports one field of the current poll as failed for `failure` (a stale token's failure
    // belongs to no current poll and is not remembered), then completes that field via
    // finishField().
    void failField(uint32_t token, tuf2000::Tuf2000Failure failure);
    // Keeps only the FIRST failure reason of the poll being built (and mirrors it into the sticky
    // lastFailure_): later ones are usually knock-on effects of the first.
    void noteFailure(tuf2000::Tuf2000Failure failure);
    // Decodes one reply's register data into scratch_. False if any value in it is NaN/infinity.
    bool parseField(tuf2000::Tuf2000PollTracker::Field field, const ModbusMessage& msg);
    // Reads the next two registers of msg at idx as a low-word-first REAL4, advancing idx. False -
    // and out untouched - if it decodes to NaN/infinity.
    static bool takeFloat(const ModbusMessage& msg, uint16_t& idx, float& out);

    std::unique_ptr<ModbusClientRTU> client_;
    Config config_;
    LogCallback logger_ = nullptr;

    // Poll-sequencing/staleness state machine - see tuf2000::Tuf2000PollTracker's
    // class comment. readAsync() (whatever task calls it) fully starts a new poll (beginPoll())
    // BEFORE issuing any request for it, then never touches pollTracker_/scratch_ again until the
    // next poll starts - so there is exactly one handoff point per poll, not ongoing concurrent
    // access (pollTracker_.inFlight() itself is the one field genuinely read cross-task; see its
    // own volatile comment).
    tuf2000::Tuf2000PollTracker pollTracker_;
    Reading scratch_;
    // First failure reason seen in the poll being built; reset by readAsync() at the same handoff
    // point as scratch_, and only touched by the worker task's callbacks after that.
    tuf2000::Tuf2000Failure pollFailure_ = tuf2000::Tuf2000Failure::kNone;
    // Judged once per completed poll, on the worker task only. Real limit comes from begin().
    tuf2000::TotalizerJumpGuard jumpGuard_{0.0f};

    volatile uint8_t lastErrorCode_ = 0;
    volatile uint8_t lastFailure_ = 0;

    uint32_t lastPollMs_ = 0;
    bool everPolled_ = false;

    // Depth-1 "latest value" mailbox from eModbus's worker task (where the callbacks above run) to
    // whatever task calls responseReady()/takeReading() - xQueueOverwrite/
    // xQueueReceive give atomic, tear-free handoff of the whole Reading struct with no hand-written
    // locking.
    QueueHandle_t resultQueue_ = nullptr;
};

}  // namespace tuf2000
