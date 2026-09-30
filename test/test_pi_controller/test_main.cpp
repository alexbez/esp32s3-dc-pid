#include <unity.h>
#include <math.h>
#include "PiController.h"

static constexpr float DT = 0.02f;

void setUp() {}
void tearDown() {}

// Output limits 0..1, generous integral limit so that anti-windup tests
// exercise the conditional-integration logic rather than the hard clamp.
static PiController makeController(float kp, float ki, float integralLimit = 100.0f) {
    return PiController(kp, ki, 0.0f, 1.0f, integralLimit);
}

void test_proportional_only() {
    PiController pi = makeController(0.01f, 0.0f);
    float out = pi.update(50.0f, 40.0f, 0.0f, DT);  // error = 10
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, out);
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, pi.integral());
}

void test_feedforward_added_to_output() {
    PiController pi = makeController(0.01f, 0.0f);
    float out = pi.update(50.0f, 50.0f, 0.4f, DT);  // zero error -> output == feed-forward
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, out);
}

void test_integral_accumulates() {
    PiController pi = makeController(0.0f, 0.5f);
    for (int i = 0; i < 5; ++i) pi.update(10.0f, 0.0f, 0.0f, DT);  // error = 10
    // 5 steps * ki * error * dt = 5 * 0.5 * 10 * 0.02 = 0.5 (still below the output limit)
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, pi.integral());
}

void test_gain_update_changes_output() {
    PiController pi = makeController(0.01f, 0.0f);
    float before = pi.update(50.0f, 40.0f, 0.0f, DT);
    TEST_ASSERT_TRUE(pi.setGains(0.02f, 0.0f));
    float after = pi.update(50.0f, 40.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, before);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, after);
}

void test_gain_update_keeps_integral() {
    PiController pi = makeController(0.0f, 0.5f);
    for (int i = 0; i < 5; ++i) pi.update(10.0f, 0.0f, 0.0f, DT);
    const float before = pi.integral();
    TEST_ASSERT_TRUE(pi.setGains(0.01f, 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, before, pi.integral());  // no output bump on retune
}

void test_invalid_gains_rejected() {
    PiController pi = makeController(0.01f, 0.03f);
    TEST_ASSERT_FALSE(pi.setGains(-0.1f, 0.03f));
    TEST_ASSERT_FALSE(pi.setGains(0.01f, -1.0f));
    TEST_ASSERT_FALSE(pi.setGains(NAN, 0.03f));
    TEST_ASSERT_FALSE(pi.setGains(0.01f, INFINITY));
    TEST_ASSERT_FALSE(pi.setGains(PiController::kGainMax + 1.0f, 0.03f));
    // Gains are unchanged after every rejected update.
    TEST_ASSERT_EQUAL_FLOAT(0.01f, pi.kp);
    TEST_ASSERT_EQUAL_FLOAT(0.03f, pi.ki);
}

void test_output_clamped_to_limits() {
    PiController pi = makeController(1.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pi.update(100.0f, 0.0f, 0.0f, DT));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pi.update(0.0f, 100.0f, 0.0f, DT));
}

void test_antiwindup_freezes_integral_at_upper_saturation() {
    PiController pi = makeController(0.01f, 0.1f);

    // Error of 50 -> P = 0.5, integral gains 0.1 per step -> output saturates after a few steps.
    for (int i = 0; i < 50; ++i) pi.update(50.0f, 0.0f, 0.0f, DT);
    const float integralAt50 = pi.integral();
    for (int i = 0; i < 500; ++i) pi.update(50.0f, 0.0f, 0.0f, DT);

    // Integral stopped growing once saturated and stays bounded just above the
    // value needed to reach the limit (0.5 + one step of 0.1).
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, integralAt50, pi.integral());
    TEST_ASSERT_TRUE(pi.integral() <= 0.6f + 1e-6f);
}

void test_antiwindup_fast_recovery_after_saturation() {
    PiController pi = makeController(0.01f, 0.1f);
    float out = 0.0f;
    for (int i = 0; i < 500; ++i) out = pi.update(50.0f, 0.0f, 0.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out);  // saturated

    // Speed reaches the setpoint: with no windup the output must leave the
    // limit on the very next step. A wound-up integrator would hold it at 1.0.
    out = pi.update(50.0f, 50.0f, 0.0f, DT);
    TEST_ASSERT_TRUE(out < 1.0f);
}

void test_antiwindup_lower_saturation() {
    // ff = 0.5, P = -0.25 (error -50), integral drops 0.1 per step. On step 3 the next
    // integral value (-0.3) would push the output below 0, so the integrator freezes at -0.2
    // (output 0.05, i.e. within one integration step of the limit) and stays there.
    PiController pi = makeController(0.005f, 0.1f);
    float out = 1.0f;
    for (int i = 0; i < 500; ++i) out = pi.update(0.0f, 50.0f, 0.5f, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.2f, pi.integral());  // frozen, not running away
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.05f, out);

    // Error back to zero -> output recovers immediately (no wound-up negative integral).
    out = pi.update(50.0f, 50.0f, 0.5f, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.3f, out);
}

void test_integral_hard_clamp() {
    // Output limit far above the integral limit so the hard clamp is what bounds it.
    PiController pi(0.0f, 5.0f, 0.0f, 100.0f, 0.2f);
    for (int i = 0; i < 100; ++i) pi.update(10.0f, 0.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, pi.integral());
}

void test_reset_clears_integral() {
    PiController pi = makeController(0.0f, 0.5f);
    for (int i = 0; i < 5; ++i) pi.update(10.0f, 0.0f, 0.0f, DT);
    TEST_ASSERT_TRUE(pi.integral() > 0.0f);
    pi.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pi.integral());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_proportional_only);
    RUN_TEST(test_feedforward_added_to_output);
    RUN_TEST(test_integral_accumulates);
    RUN_TEST(test_gain_update_changes_output);
    RUN_TEST(test_gain_update_keeps_integral);
    RUN_TEST(test_invalid_gains_rejected);
    RUN_TEST(test_output_clamped_to_limits);
    RUN_TEST(test_antiwindup_freezes_integral_at_upper_saturation);
    RUN_TEST(test_antiwindup_fast_recovery_after_saturation);
    RUN_TEST(test_antiwindup_lower_saturation);
    RUN_TEST(test_integral_hard_clamp);
    RUN_TEST(test_reset_clears_integral);
    return UNITY_END();
}
