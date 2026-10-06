#pragma once

#include <stddef.h>
#include <stdint.h>

namespace tuf2000 {

// Pure (no Arduino/FreeRTOS/eModbus dependency) TUF-2000 protocol logic, kept apart from
// Tuf2000FlowMeter so it can be unit tested under `pio test -e native` without any Modbus/serial
// hardware or mocking - the driver itself is just a thin eModbus-facing wrapper around this.

// The meter sends every 32-bit value low word first: reg0 (the lower-numbered register) is the low
// 16 bits, reg1 the high 16 bits - the manual's "lower byte first" note, confirmed at bench
// bring-up (a high-word-first decode turned the fluid sound speed into an exact integer, its low
// mantissa word replaced by the next register's zero). True unless value is NaN or +/-infinity.
// Checked on the IEEE-754 bit pattern (all-ones exponent) rather than via std::isfinite, so it
// stays correct even under -ffast-math.
bool isFiniteFloat(float value);

// REAL4: reinterprets the assembled 32 bits as an IEEE-754 float. Returns false - leaving out
// untouched - when the result is NaN or infinity, which no real flow/totalizer/sound-speed reading
// can be: a garbage frame or a wrong word order would otherwise flow into every total and every
// published payload as if it were data.
bool decodeFloatLowWordFirst(uint16_t reg0, uint16_t reg1, float& out);

// Why a poll was rejected (Reading::valid == false). kModbusError covers a wire-level error, a
// timeout and a failed enqueue; the others are replies that arrived but cannot be trusted.
enum class Tuf2000Failure : uint8_t {
    kNone = 0,
    kModbusError,
    kBadReplyLength,
    kNonFiniteValue,
    kTotalizerJump
};
// Static, never-null text for logs, the cmd/meter reply and the /alert reason.
const char* tuf2000FailureText(Tuf2000Failure failure);

// A function-3 reply is [0]=server ID, [1]=function code, [2]=byte count, then the register data
// (the CRC is already stripped by eModbus). True only if the byte count is exactly two per word
// requested AND the message really is that long - a truncated reply otherwise reads back as zeros.
bool replyLengthMatches(size_t messageSize, uint8_t byteCount, uint16_t wordsRequested);

// Rejects a totalizer reading that cannot be real, judged against the last one it accepted: a
// positive accumulator is never negative, and no real poll interval moves it forward by more than
// maxJumpM3 (a garbage-but-finite value would otherwise be credited to a zone's lifetime total). A
// decrease is accepted - a totalizer reset on the meter is legitimate - and consumers already only
// count positive deltas. The very first reading has nothing to compare against and is accepted.
// A rejected reading does not move the baseline, so a later plausible one is judged against the
// last good value; consequently a genuine jump above maxJumpM3 (e.g. more than that flowed while
// the bus was down) keeps being rejected until the firmware restarts and re-baselines.
class TotalizerJumpGuard {
public:
    explicit TotalizerJumpGuard(float maxJumpM3) : maxJumpM3_(maxJumpM3) {}
    bool accept(float totalizerM3);

private:
    float maxJumpM3_;
    float lastAcceptedM3_ = 0.0f;
    bool haveBaseline_ = false;
};

// The signal-quality register packs two numbers: the low byte is the actual 0-100 quality, the high
// byte is the autogain step (see Tuf2000FlowMeter::Config::signalQualityRegister).
inline uint8_t signalQualityValue(uint16_t reg) { return static_cast<uint8_t>(reg & 0xFF); }
inline uint8_t signalAutogainStep(uint16_t reg) { return static_cast<uint8_t>(reg >> 8); }

// Tracks one poll cycle's four outstanding Modbus sub-requests (flow rate / totalizer / signal
// block / sound speed - see Tuf2000FlowMeter::Config's register fields), issued together and never
// overlapped with the NEXT poll's. Encodes/decodes the eModbus request "token" (poll sequence
// number + which field) so a reply arriving after this poll has already been abandoned (stuck-
// request watchdog) is recognized as stale and ignored, instead of corrupting whatever poll comes
// next.
class Tuf2000PollTracker {
public:
    // kSignalQuality covers the whole signal block (quality + upstream/downstream strength words).
    // The token's field slot is kFieldBits wide, so 4 fields is the hard ceiling - a fifth one
    // needs a wider slot (bump kFieldBits), which shifts pollSeq accordingly.
    enum Field : uint32_t { kFlowRate = 0, kTotalizer = 1, kSignalQuality = 2, kSoundSpeed = 3 };
    static constexpr uint8_t kFieldCount = 4;
    static constexpr uint32_t kFieldBits = 2;
    static constexpr uint32_t kFieldMask = (1u << kFieldBits) - 1u;

    // How many 16-bit registers each field's request asks for - the single source for both the
    // request the driver sends and the reply-length check on what comes back. Every REAL4 is 2
    // words; the signal block is quality + upstream strength + downstream strength.
    static constexpr uint16_t wordCountOf(Field field) { return field == kSignalQuality ? 3 : 2; }

    // Starts a new poll: bumps the poll sequence number and resets pending/failed state. Returns
    // the sequence number the caller should pass to makeToken() for each of this poll's requests.
    uint32_t beginPoll(uint32_t nowMs);

    static uint32_t makeToken(uint32_t pollSeq, Field field) {
        return (pollSeq << kFieldBits) | static_cast<uint32_t>(field);
    }
    static Field fieldOf(uint32_t token) { return static_cast<Field>(token & kFieldMask); }
    static uint32_t pollSeqOf(uint32_t token) { return token >> kFieldBits; }

    // True if token was issued under the currently active poll - false for a stale reply from a
    // poll already abandoned by forceClear() (or superseded by a later beginPoll()).
    bool isCurrent(uint32_t token) const { return pollSeqOf(token) == activePollSeq_; }

    // Records one field's outcome for the active poll. Stale tokens (see isCurrent()) are ignored
    // outright - they don't count against pendingFieldCount_ and can't flip failed(). Returns true
    // exactly once per poll: the moment every field has reported in (success or failure), at which
    // point inFlight() also goes false.
    bool recordField(uint32_t token, bool ok);

    bool failed() const { return failed_; }
    bool inFlight() const { return inFlight_; }
    uint32_t startedAtMs() const { return startedAtMs_; }
    uint32_t activePollSeq() const { return activePollSeq_; }

    // True once the active poll has been in flight longer than timeoutMs - the stuck-request
    // watchdog condition (a bug, or a wedged transceiver, left a reply from eModbus never arriving
    // at all). Always false when nothing is in flight.
    bool isStuck(uint32_t nowMs, uint32_t timeoutMs) const {
        return inFlight_ && (nowMs - startedAtMs_) >= timeoutMs;
    }

    // Force-ends the active poll without every field having reported (the watchdog's response to
    // isStuck()) - inFlight() goes false so the caller can retry, but activePollSeq_ is left alone
    // (only beginPoll() bumps it) so any of this poll's replies that do eventually trickle in are
    // still correctly recognized as stale by isCurrent() until the NEXT beginPoll() moves on.
    void forceClear() { inFlight_ = false; }

private:
    uint32_t activePollSeq_ = 0;
    uint8_t pendingFieldCount_ = 0;
    bool failed_ = false;
    // volatile: in the real driver, recordField()/forceClear() run on a different task (eModbus's
    // worker task / the caller of readAsync() respectively) than inFlight()/isStuck() are read
    // from - a single flag tolerated across tasks without locking. Doesn't change this class's
    // value semantics for the (single-threaded) unit tests, only prevents the compiler from
    // caching/reordering the read.
    volatile bool inFlight_ = false;
    uint32_t startedAtMs_ = 0;
};

}  // namespace tuf2000
