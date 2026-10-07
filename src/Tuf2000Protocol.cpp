#include "Tuf2000Protocol.h"

#include <stdio.h>
#include <string.h>

namespace tuf2000 {

bool isFiniteFloat(float value) {
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}

bool decodeFloatLowWordFirst(uint16_t reg0, uint16_t reg1, float& out) {
    const uint32_t bits = (static_cast<uint32_t>(reg1) << 16) | reg0;
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    if (!isFiniteFloat(value)) {
        return false;
    }
    out = value;
    return true;
}

const char* tuf2000FailureText(Tuf2000Failure failure) {
    switch (failure) {
        case Tuf2000Failure::kNone:
            return "none";
        case Tuf2000Failure::kModbusError:
            return "modbus_error_or_timeout";
        case Tuf2000Failure::kBadReplyLength:
            return "bad_reply_length";
        case Tuf2000Failure::kNonFiniteValue:
            return "non_finite_value";
        case Tuf2000Failure::kTotalizerJump:
            return "totalizer_jump_too_large";
    }
    return "unknown";
}

void formatEnqueueFailed(char* buf, size_t size, const char* modbusErrorText) {
    snprintf(buf, size, "tuf2000: enqueue failed: %s", modbusErrorText);
}

void formatPollRejected(char* buf, size_t size, Tuf2000Failure failure) {
    snprintf(buf, size, "tuf2000: poll rejected: %s", tuf2000FailureText(failure));
}

bool replyLengthMatches(size_t messageSize, uint8_t byteCount, uint16_t wordsRequested) {
    const size_t dataBytes = static_cast<size_t>(wordsRequested) * 2;
    return byteCount == dataBytes && messageSize == 3 + dataBytes;
}

bool TotalizerJumpGuard::accept(float totalizerM3) {
    if (totalizerM3 < 0.0f) {
        return false;
    }
    if (haveBaseline_ && totalizerM3 - lastAcceptedM3_ > maxJumpM3_) {
        return false;
    }
    lastAcceptedM3_ = totalizerM3;
    haveBaseline_ = true;
    return true;
}

uint32_t Tuf2000PollTracker::beginPoll(uint32_t nowMs) {
    ++activePollSeq_;
    pendingFieldCount_ = kFieldCount;
    failed_ = false;
    inFlight_ = true;
    startedAtMs_ = nowMs;
    return activePollSeq_;
}

bool Tuf2000PollTracker::recordField(uint32_t token, bool ok) {
    if (!isCurrent(token)) {
        return false;  // Stale reply from an already-abandoned poll - see the class comment.
    }
    if (!ok) {
        failed_ = true;
    }
    if (pendingFieldCount_ > 0) {
        --pendingFieldCount_;
    }
    if (pendingFieldCount_ == 0) {
        inFlight_ = false;
        return true;
    }
    return false;
}

}  // namespace tuf2000
