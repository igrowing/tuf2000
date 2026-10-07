#include "Tuf2000FlowMeter.h"

#include <stdio.h>

namespace tuf2000 {

using tuf2000::decodeFloatLowWordFirst;
using tuf2000::isFiniteFloat;
using tuf2000::replyLengthMatches;
using tuf2000::TotalizerJumpGuard;
using tuf2000::Tuf2000Failure;
using tuf2000::Tuf2000PollTracker;

bool Tuf2000FlowMeter::begin(HardwareSerial& serial, const Config& cfg) {
    if (cfg.rxPin < 0 || cfg.txPin < 0 || cfg.pollIntervalMs == 0) {
        log("tuf2000: invalid config (rx/tx pin unset or zero poll interval)");
        return false;
    }
    config_ = cfg;
    jumpGuard_ = TotalizerJumpGuard(cfg.maxTotalizerJumpM3);
    // Depth 1: only the latest poll's result ever matters to a caller of takeReading() - an
    // unconsumed previous reading is simply overwritten, never queued up.
    resultQueue_ = xQueueCreate(1, sizeof(Reading));

    RTUutils::prepareHardwareSerial(serial);
    serial.begin(cfg.baudRate, SERIAL_8N1, cfg.rxPin, cfg.txPin);

    // No DE/RE pin means a transceiver that switches direction automatically - the default
    // constructor is exactly that case.
    if (cfg.dePin < 0) {
        client_ = std::make_unique<ModbusClientRTU>();
    } else {
        client_ = std::make_unique<ModbusClientRTU>(cfg.dePin);
    }
    // The by-value parameter is the signature eModbus's handler type dictates.
    // cppcheck-suppress passedByValue
    client_->onDataHandler([this](ModbusMessage msg, uint32_t token) { handleData(msg, token); });
    client_->onErrorHandler(
        [this](Modbus::Error error, uint32_t token) { handleError(error, token); });
    client_->begin(serial);
    return true;
}

void Tuf2000FlowMeter::readAsync(uint32_t nowMs) {
    if (client_ == nullptr) {
        return;
    }

    if (pollTracker_.inFlight()) {
        if (pollTracker_.isStuck(nowMs, config_.stuckRequestTimeoutMs)) {
            // Stuck-request watchdog: a bug, or a wedged transceiver, left the in-flight flag set
            // far longer than eModbus's own per-request timeout could explain. Force-clear it so
            // the throttle check below can retry instead of every future poll silently no-op'ing
            // forever - see Tuf2000PollTracker's class comment for the one edge case this doesn't
            // fully close.
            pollTracker_.forceClear();
        } else {
            return;
        }
    }

    if (everPolled_ && nowMs - lastPollMs_ < config_.pollIntervalMs) {
        return;
    }
    everPolled_ = true;
    lastPollMs_ = nowMs;

    // Fully start this poll BEFORE issuing any request for it: issueRequest() below can enqueue
    // onto eModbus's worker task, whose callbacks (handleData/handleError) touch pollTracker_/
    // scratch_ - by ordering it this way, that worker task can never observe a half-initialized
    // poll.
    const uint32_t pollSeq = pollTracker_.beginPoll(nowMs);
    scratch_ = Reading{};
    scratch_.timestampMs = nowMs;
    pollFailure_ = Tuf2000Failure::kNone;

    issueRequest(pollSeq, Tuf2000PollTracker::kFlowRate, config_.flowRateRegister);
    issueRequest(pollSeq, Tuf2000PollTracker::kTotalizer, config_.totalizerRegister);
    issueRequest(pollSeq, Tuf2000PollTracker::kSignalQuality, config_.signalQualityRegister);
    issueRequest(pollSeq, Tuf2000PollTracker::kSoundSpeed, config_.soundSpeedRegister);
}

bool Tuf2000FlowMeter::takeFloat(const ModbusMessage& msg, uint16_t& idx, float& out) {
    uint16_t reg0 = 0;
    uint16_t reg1 = 0;
    idx = msg.get(idx, reg0);
    idx = msg.get(idx, reg1);
    return decodeFloatLowWordFirst(reg0, reg1, out);
}

const char* Tuf2000FlowMeter::lastErrorText() const {
    return static_cast<const char*>(ModbusError(static_cast<Modbus::Error>(lastErrorCode_)));
}

void Tuf2000FlowMeter::recordError(Modbus::Error error) {
    lastErrorCode_ = static_cast<uint8_t>(error);
}

bool Tuf2000FlowMeter::responseReady() const {
    return resultQueue_ != nullptr && uxQueueMessagesWaiting(resultQueue_) > 0;
}

bool Tuf2000FlowMeter::takeReading(Reading& out) {
    if (resultQueue_ == nullptr) {
        return false;
    }
    return xQueueReceive(resultQueue_, &out, 0) == pdTRUE;
}

bool Tuf2000FlowMeter::read(uint32_t timeoutMs, Reading& out) {
    readAsync(millis());
    const uint32_t startMs = millis();
    while (millis() - startMs < timeoutMs) {
        if (takeReading(out)) {
            return true;
        }
        delay(5);
    }
    return false;
}

void Tuf2000FlowMeter::issueRequest(uint32_t pollSeq, Tuf2000PollTracker::Field field,
                                    uint16_t startRegister) {
    const uint32_t token = Tuf2000PollTracker::makeToken(pollSeq, field);
    const Error rc = client_->addRequest(token, config_.slaveId, READ_HOLD_REGISTER, startRegister,
                                         Tuf2000PollTracker::wordCountOf(field));
    if (rc != SUCCESS) {
        // Couldn't even enqueue the request (e.g. eModbus's own internal request queue full) -
        // treat it exactly like a wire-level failure for this field so the poll still completes,
        // rather than hanging pollTracker_ forever waiting for a callback that will never fire.
        recordError(rc);
        char msg[64];
        formatEnqueueFailed(msg, sizeof(msg), lastErrorText());
        log(msg);
        failField(token, Tuf2000Failure::kModbusError);
    }
}

bool Tuf2000FlowMeter::parseField(Tuf2000PollTracker::Field field, const ModbusMessage& msg) {
    // Response layout: [0]=server ID, [1]=function code, [2]=byte count, [3..]=register data.
    uint16_t idx = 3;
    float value = 0.0f;
    switch (field) {
        case Tuf2000PollTracker::kFlowRate:
        case Tuf2000PollTracker::kTotalizer: {
            if (!takeFloat(msg, idx, value)) {
                return false;
            }
            // The corrected figure has to be finite as well: a huge but finite raw value can
            // overflow to infinity once scaled.
            value *= config_.calibrationMultiplier;
            if (!isFiniteFloat(value)) {
                return false;
            }
            (field == Tuf2000PollTracker::kFlowRate ? scratch_.flowRateM3h : scratch_.totalizerM3) =
                value;
            return true;
        }
        case Tuf2000PollTracker::kSignalQuality:
            idx = msg.get(idx, scratch_.signalQuality);
            idx = msg.get(idx, scratch_.signalStrengthUp);
            msg.get(idx, scratch_.signalStrengthDown);
            return true;
        case Tuf2000PollTracker::kSoundSpeed:
            if (!takeFloat(msg, idx, value)) {
                return false;
            }
            scratch_.soundSpeedMs = value;
            return true;
    }
    return false;
}

void Tuf2000FlowMeter::handleData(ModbusMessage& msg, uint32_t token) {
    if (!pollTracker_.isCurrent(token)) {
        return;  // Reply to an already-abandoned poll: nothing to record (see the tracker).
    }
    const Tuf2000PollTracker::Field field = Tuf2000PollTracker::fieldOf(token);
    const uint16_t size = msg.size();
    const uint8_t byteCount = size > 2 ? msg[2] : 0;
    if (!replyLengthMatches(size, byteCount, Tuf2000PollTracker::wordCountOf(field))) {
        failField(token, Tuf2000Failure::kBadReplyLength);
        return;
    }
    if (!parseField(field, msg)) {
        failField(token, Tuf2000Failure::kNonFiniteValue);
        return;
    }
    finishField(token, true);
}

void Tuf2000FlowMeter::handleError(Modbus::Error error, uint32_t token) {
    recordError(error);
    failField(token, Tuf2000Failure::kModbusError);
}

void Tuf2000FlowMeter::noteFailure(Tuf2000Failure failure) {
    if (pollFailure_ == Tuf2000Failure::kNone) {
        pollFailure_ = failure;
        lastFailure_ = static_cast<uint8_t>(failure);
        char msg[64];
        formatPollRejected(msg, sizeof(msg), failure);
        log(msg);
    }
}

void Tuf2000FlowMeter::failField(uint32_t token, Tuf2000Failure failure) {
    if (pollTracker_.isCurrent(token)) {
        noteFailure(failure);
    }
    finishField(token, false);
}

void Tuf2000FlowMeter::finishField(uint32_t token, bool ok) {
    if (pollTracker_.recordField(token, ok)) {
        scratch_.valid = !pollTracker_.failed();
        // Judged only once the whole poll is in and otherwise clean: the totalizer of a poll that
        // already failed is not trustworthy enough to move the guard's baseline.
        if (scratch_.valid && !jumpGuard_.accept(scratch_.totalizerM3)) {
            scratch_.valid = false;
            noteFailure(Tuf2000Failure::kTotalizerJump);
        }
        scratch_.failure = pollFailure_;
        if (resultQueue_ != nullptr) {
            xQueueOverwrite(resultQueue_, &scratch_);
        }
    }
}

}  // namespace tuf2000
