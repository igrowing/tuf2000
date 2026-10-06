#include <unity.h>

#include "Tuf2000Protocol.h"

using namespace tuf2000;

void setUp() {}
void tearDown() {}


// --- Tuf2000Protocol: low-word-first REAL4 decoding -------------------------------------------

static float decodeOrFail(uint16_t reg0, uint16_t reg1) {
    float out = -999.0f;
    TEST_ASSERT_TRUE(tuf2000::decodeFloatLowWordFirst(reg0, reg1, out));
    return out;
}

void test_decode_float_low_word_first_takes_first_register_as_low_word() {
    // 12.5f is 0x41480000: the first register carries the low word (0x0000), the second the high.
    TEST_ASSERT_EQUAL_FLOAT(12.5f, decodeOrFail(0x0000, 0x4148));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, decodeOrFail(0x0000, 0x0000));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, decodeOrFail(0x0000, 0xBF80));
    // 3.14159274f is 0x40490FDB - a non-zero low word exercises both halves.
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.14159274f, decodeOrFail(0x0FDB, 0x4049));
}

void test_decode_float_rejects_nan_and_infinity_and_leaves_out_untouched() {
    using tuf2000::decodeFloatLowWordFirst;
    float out = 123.0f;

    TEST_ASSERT_FALSE(decodeFloatLowWordFirst(0x0000, 0x7F80, out));  // +infinity
    TEST_ASSERT_FALSE(decodeFloatLowWordFirst(0x0000, 0xFF80, out));  // -infinity
    TEST_ASSERT_FALSE(decodeFloatLowWordFirst(0x0000, 0x7FC0, out));  // quiet NaN
    TEST_ASSERT_FALSE(decodeFloatLowWordFirst(0x0001, 0x7F80, out));  // signalling NaN
    TEST_ASSERT_FALSE(decodeFloatLowWordFirst(0xFFFF, 0xFFFF, out));  // all ones, e.g. a garbage frame
    TEST_ASSERT_EQUAL_FLOAT(123.0f, out);

    // Neighbours that ARE real numbers: the largest finite float and a subnormal.
    TEST_ASSERT_TRUE(decodeFloatLowWordFirst(0xFFFF, 0x7F7F, out));
    TEST_ASSERT_TRUE(decodeFloatLowWordFirst(0x0001, 0x0000, out));
}

void test_is_finite_float_matches_the_ieee_754_classes() {
    using tuf2000::isFiniteFloat;
    TEST_ASSERT_TRUE(isFiniteFloat(0.0f));
    TEST_ASSERT_TRUE(isFiniteFloat(-2.5f));
    TEST_ASSERT_TRUE(isFiniteFloat(3.0e38f));
    TEST_ASSERT_FALSE(isFiniteFloat(3.0e38f * 10.0f));   // Overflows to +infinity.
    TEST_ASSERT_FALSE(isFiniteFloat(-3.0e38f * 10.0f));  // -infinity.
    TEST_ASSERT_FALSE(isFiniteFloat(0.0f / 0.0f));       // NaN.
}

void test_word_count_of_matches_each_fields_request() {
    using tuf2000::Tuf2000PollTracker;
    TEST_ASSERT_EQUAL_UINT16(2, Tuf2000PollTracker::wordCountOf(Tuf2000PollTracker::kFlowRate));
    TEST_ASSERT_EQUAL_UINT16(2, Tuf2000PollTracker::wordCountOf(Tuf2000PollTracker::kTotalizer));
    TEST_ASSERT_EQUAL_UINT16(3, Tuf2000PollTracker::wordCountOf(Tuf2000PollTracker::kSignalQuality));
    TEST_ASSERT_EQUAL_UINT16(2, Tuf2000PollTracker::wordCountOf(Tuf2000PollTracker::kSoundSpeed));
}

void test_reply_length_matches_only_an_exact_well_formed_reply() {
    using tuf2000::replyLengthMatches;

    // 2 words: 3 header bytes + 4 data bytes, byte count 4.
    TEST_ASSERT_TRUE(replyLengthMatches(7, 4, 2));
    TEST_ASSERT_TRUE(replyLengthMatches(9, 6, 3));

    TEST_ASSERT_FALSE(replyLengthMatches(7, 2, 2));   // Byte count says fewer words than asked.
    TEST_ASSERT_FALSE(replyLengthMatches(7, 6, 2));   // ... or more.
    TEST_ASSERT_FALSE(replyLengthMatches(5, 4, 2));   // Truncated: header claims 4 but only 2 arrived.
    TEST_ASSERT_FALSE(replyLengthMatches(9, 4, 2));   // Trailing bytes beyond the claimed data.
    TEST_ASSERT_FALSE(replyLengthMatches(3, 0, 2));   // Header only.
    TEST_ASSERT_FALSE(replyLengthMatches(0, 0, 2));   // Empty message.
    TEST_ASSERT_FALSE(replyLengthMatches(7, 4, 3));   // Right reply, but for a different request.
}

void test_failure_text_is_distinct_and_keeps_the_original_modbus_reason() {
    using tuf2000::Tuf2000Failure;
    using tuf2000::tuf2000FailureText;

    // The alert automation keys on this exact pre-existing string.
    TEST_ASSERT_EQUAL_STRING("modbus_error_or_timeout", tuf2000FailureText(Tuf2000Failure::kModbusError));
    TEST_ASSERT_EQUAL_STRING("none", tuf2000FailureText(Tuf2000Failure::kNone));
    TEST_ASSERT_EQUAL_STRING("bad_reply_length", tuf2000FailureText(Tuf2000Failure::kBadReplyLength));
    TEST_ASSERT_EQUAL_STRING("non_finite_value", tuf2000FailureText(Tuf2000Failure::kNonFiniteValue));
    TEST_ASSERT_EQUAL_STRING("totalizer_jump_too_large", tuf2000FailureText(Tuf2000Failure::kTotalizerJump));
}

void test_totalizer_jump_guard_accepts_the_first_reading_and_normal_growth() {
    tuf2000::TotalizerJumpGuard guard(10.0f);

    TEST_ASSERT_TRUE(guard.accept(2.68f));  // Nothing to compare against yet.
    TEST_ASSERT_TRUE(guard.accept(2.69f));
    TEST_ASSERT_TRUE(guard.accept(2.69f));  // No flow: unchanged.
    TEST_ASSERT_TRUE(guard.accept(12.69f)); // Exactly the limit is still plausible.
}

void test_totalizer_jump_guard_rejects_a_jump_above_the_limit_without_moving_the_baseline() {
    tuf2000::TotalizerJumpGuard guard(10.0f);
    TEST_ASSERT_TRUE(guard.accept(2.68f));

    TEST_ASSERT_FALSE(guard.accept(12.69f));   // 10.01 m3 forward.
    TEST_ASSERT_FALSE(guard.accept(1.0e30f));  // Garbage but finite.
    // Baseline is still 2.68, so a plausible reading right after is judged against that.
    TEST_ASSERT_TRUE(guard.accept(2.70f));
}

void test_totalizer_jump_guard_rejects_a_negative_total_but_accepts_a_reset_to_zero() {
    tuf2000::TotalizerJumpGuard guard(10.0f);
    TEST_ASSERT_TRUE(guard.accept(50.0f));

    TEST_ASSERT_FALSE(guard.accept(-0.5f));    // A positive accumulator is never negative.
    TEST_ASSERT_FALSE(guard.accept(-1.0e30f)); // Would otherwise poison the baseline downwards.
    TEST_ASSERT_TRUE(guard.accept(0.0f));      // Totalizer reset on the meter: legitimate decrease.
    TEST_ASSERT_TRUE(guard.accept(0.4f));      // ... and counting resumes from the new baseline.
}

void test_totalizer_jump_guard_rejects_a_negative_very_first_reading() {
    tuf2000::TotalizerJumpGuard guard(10.0f);
    TEST_ASSERT_FALSE(guard.accept(-3.0f));
    TEST_ASSERT_TRUE(guard.accept(3.0f));  // Still no baseline: the first sane one is adopted.
}

// --- Tuf2000Protocol: Tuf2000PollTracker -----------------------------------------------------

void test_poll_tracker_token_round_trips_field_and_sequence() {
    using tuf2000::Tuf2000PollTracker;

    const uint32_t token = Tuf2000PollTracker::makeToken(7, Tuf2000PollTracker::kTotalizer);
    TEST_ASSERT_EQUAL_UINT32(7, Tuf2000PollTracker::pollSeqOf(token));
    TEST_ASSERT_TRUE(Tuf2000PollTracker::kTotalizer == Tuf2000PollTracker::fieldOf(token));

    // Every field, at a poll sequence large enough to matter (not just 0/1).
    const uint32_t seq = 12345;
    for (uint32_t f = 0; f < Tuf2000PollTracker::kFieldCount; ++f) {
        const auto field = static_cast<Tuf2000PollTracker::Field>(f);
        const uint32_t t = Tuf2000PollTracker::makeToken(seq, field);
        TEST_ASSERT_TRUE(field == Tuf2000PollTracker::fieldOf(t));
        TEST_ASSERT_EQUAL_UINT32(seq, Tuf2000PollTracker::pollSeqOf(t));
    }
}

// Records every field of poll seq except the last one as ok; returns what the last one reported.
static bool completeTufPollWithLastField(tuf2000::Tuf2000PollTracker& tracker,
                                         uint32_t seq, bool lastOk) {
    using tuf2000::Tuf2000PollTracker;
    for (uint32_t f = 0; f + 1 < Tuf2000PollTracker::kFieldCount; ++f) {
        TEST_ASSERT_FALSE(tracker.recordField(
            Tuf2000PollTracker::makeToken(seq, static_cast<Tuf2000PollTracker::Field>(f)), true));
        TEST_ASSERT_TRUE(tracker.inFlight());  // Still waiting on the remaining fields.
    }
    return tracker.recordField(
        Tuf2000PollTracker::makeToken(
            seq, static_cast<Tuf2000PollTracker::Field>(Tuf2000PollTracker::kFieldCount - 1)),
        lastOk);
}

void test_poll_tracker_begin_poll_increments_sequence_and_starts_in_flight() {
    tuf2000::Tuf2000PollTracker tracker;

    const uint32_t seq1 = tracker.beginPoll(1000);
    TEST_ASSERT_EQUAL_UINT32(seq1, tracker.activePollSeq());
    TEST_ASSERT_TRUE(tracker.inFlight());
    TEST_ASSERT_FALSE(tracker.failed());
    TEST_ASSERT_EQUAL_UINT32(1000, tracker.startedAtMs());

    const uint32_t seq2 = tracker.beginPoll(2000);
    TEST_ASSERT_TRUE(seq2 > seq1);  // Monotonically increasing, never reused.
    TEST_ASSERT_EQUAL_UINT32(seq2, tracker.activePollSeq());
}

void test_poll_tracker_completes_only_after_every_field_reports() {
    using tuf2000::Tuf2000PollTracker;
    Tuf2000PollTracker tracker;

    const uint32_t seq = tracker.beginPoll(0);
    // The last field completes the poll; every earlier one leaves it in flight.
    TEST_ASSERT_TRUE(completeTufPollWithLastField(tracker, seq, true));
    TEST_ASSERT_FALSE(tracker.inFlight());
    TEST_ASSERT_FALSE(tracker.failed());
}

void test_poll_tracker_any_field_error_marks_the_whole_poll_failed() {
    using tuf2000::Tuf2000PollTracker;
    Tuf2000PollTracker tracker;

    const uint32_t seq = tracker.beginPoll(0);
    tracker.recordField(Tuf2000PollTracker::makeToken(seq, Tuf2000PollTracker::kTotalizer), false);
    bool complete = false;
    for (uint32_t f = 0; f < Tuf2000PollTracker::kFieldCount; ++f) {
        if (f != Tuf2000PollTracker::kTotalizer) {
            complete = tracker.recordField(
                Tuf2000PollTracker::makeToken(seq, static_cast<Tuf2000PollTracker::Field>(f)), true);
        }
    }

    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_TRUE(tracker.failed());  // One failed field poisons the whole poll's reading.
}

void test_poll_tracker_stale_reply_from_abandoned_poll_is_ignored() {
    using tuf2000::Tuf2000PollTracker;
    Tuf2000PollTracker tracker;

    const uint32_t staleSeq = tracker.beginPoll(0);
    const uint32_t staleToken = Tuf2000PollTracker::makeToken(staleSeq, Tuf2000PollTracker::kFlowRate);
    tracker.forceClear();  // Watchdog gave up on this poll (see isStuck()) before it completed.

    const uint32_t newSeq = tracker.beginPoll(9000);

    // The old poll's late reply must not be mistaken for part of the new one: isCurrent() rejects
    // it, and feeding it to recordField() must neither complete the new poll nor corrupt its state.
    TEST_ASSERT_FALSE(tracker.isCurrent(staleToken));
    TEST_ASSERT_FALSE(tracker.recordField(staleToken, false));
    TEST_ASSERT_TRUE(tracker.inFlight());   // New poll still waiting on all of its fields.
    TEST_ASSERT_FALSE(tracker.failed());    // The stale error must not poison the new poll.

    // The new poll still completes normally on its own fields.
    TEST_ASSERT_TRUE(completeTufPollWithLastField(tracker, newSeq, true));
    TEST_ASSERT_FALSE(tracker.failed());
}

void test_poll_tracker_is_stuck_only_once_timeout_elapsed_while_in_flight() {
    tuf2000::Tuf2000PollTracker tracker;

    tracker.beginPoll(1000);
    TEST_ASSERT_FALSE(tracker.isStuck(1000, 8000));     // No time elapsed yet.
    TEST_ASSERT_FALSE(tracker.isStuck(8999, 8000));     // Just under the timeout.
    TEST_ASSERT_TRUE(tracker.isStuck(9000, 8000));      // Exactly at the timeout.
    TEST_ASSERT_TRUE(tracker.isStuck(20000, 8000));     // Well past it.

    tracker.forceClear();
    TEST_ASSERT_FALSE(tracker.isStuck(999999, 8000));   // Nothing in flight - can't be stuck.
}

void test_poll_tracker_force_clear_lets_a_new_poll_start_without_finishing_the_old_one() {
    using tuf2000::Tuf2000PollTracker;
    Tuf2000PollTracker tracker;

    tracker.beginPoll(0);
    TEST_ASSERT_TRUE(tracker.inFlight());
    tracker.forceClear();
    TEST_ASSERT_FALSE(tracker.inFlight());

    // A fresh poll behaves normally afterwards - forceClear() doesn't wedge the tracker: only its
    // last field completes it.
    const uint32_t seq = tracker.beginPoll(100);
    TEST_ASSERT_TRUE(tracker.inFlight());
    TEST_ASSERT_FALSE(tracker.failed());
    TEST_ASSERT_TRUE(completeTufPollWithLastField(tracker, seq, true));
    TEST_ASSERT_FALSE(tracker.inFlight());
}

void test_signal_quality_register_splits_into_quality_and_autogain_step() {
    // Low byte = the actual 0-100 quality, high byte = autogain step.
    TEST_ASSERT_EQUAL_UINT8(87, tuf2000::signalQualityValue(0x0C57));
    TEST_ASSERT_EQUAL_UINT8(12, tuf2000::signalAutogainStep(0x0C57));
    TEST_ASSERT_EQUAL_UINT8(0, tuf2000::signalQualityValue(0xFF00));
    TEST_ASSERT_EQUAL_UINT8(255, tuf2000::signalAutogainStep(0xFF00));
}


int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_decode_float_low_word_first_takes_first_register_as_low_word);
    RUN_TEST(test_decode_float_rejects_nan_and_infinity_and_leaves_out_untouched);
    RUN_TEST(test_is_finite_float_matches_the_ieee_754_classes);
    RUN_TEST(test_word_count_of_matches_each_fields_request);
    RUN_TEST(test_reply_length_matches_only_an_exact_well_formed_reply);
    RUN_TEST(test_failure_text_is_distinct_and_keeps_the_original_modbus_reason);
    RUN_TEST(test_totalizer_jump_guard_accepts_the_first_reading_and_normal_growth);
    RUN_TEST(test_totalizer_jump_guard_rejects_a_jump_above_the_limit_without_moving_the_baseline);
    RUN_TEST(test_totalizer_jump_guard_rejects_a_negative_total_but_accepts_a_reset_to_zero);
    RUN_TEST(test_totalizer_jump_guard_rejects_a_negative_very_first_reading);
    RUN_TEST(test_poll_tracker_token_round_trips_field_and_sequence);
    RUN_TEST(test_poll_tracker_begin_poll_increments_sequence_and_starts_in_flight);
    RUN_TEST(test_poll_tracker_completes_only_after_every_field_reports);
    RUN_TEST(test_poll_tracker_any_field_error_marks_the_whole_poll_failed);
    RUN_TEST(test_poll_tracker_stale_reply_from_abandoned_poll_is_ignored);
    RUN_TEST(test_poll_tracker_is_stuck_only_once_timeout_elapsed_while_in_flight);
    RUN_TEST(test_poll_tracker_force_clear_lets_a_new_poll_start_without_finishing_the_old_one);
    RUN_TEST(test_signal_quality_register_splits_into_quality_and_autogain_step);
    return UNITY_END();
}
