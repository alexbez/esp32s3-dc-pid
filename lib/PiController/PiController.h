#pragma once

#include <math.h>

// Hardware-independent PI controller with feed-forward and anti-windup.
// Kept header-only and free of Arduino dependencies so it can be unit-tested
// on the host (see test/test_pi_controller).
class PiController {
public:
    static constexpr float kGainMax = 10.0f;  // sanity limit for runtime gain changes

    float kp;
    float ki;

    PiController(float kp_, float ki_, float outMin_, float outMax_, float integralLimit_)
        : kp(kp_), ki(ki_), outMin(outMin_), outMax(outMax_),
          integralLimit(integralLimit_), integral_(0.0f) {}

    // Updates both gains if (and only if) both are finite and within 0..kGainMax.
    // The integral term is stored in output units, so a gain change does not
    // cause a bump in the output.
    bool setGains(float newKp, float newKi) {
        if (!validGain(newKp) || !validGain(newKi)) return false;
        kp = newKp;
        ki = newKi;
        return true;
    }

    void reset() { integral_ = 0.0f; }

    float integral() const { return integral_; }

    // One control step. Returns the output clamped to [outMin, outMax].
    // Anti-windup: conditional integration (the integrator is frozen while the
    // output is saturated and the error would push it further into saturation)
    // plus a hard clamp of the integral term.
    float update(float setpoint, float measured, float feedforward, float dt) {
        const float error = setpoint - measured;
        const float p = kp * error;

        float integralNext = integral_ + ki * error * dt;
        const float unsat = feedforward + p + integralNext;

        if ((unsat > outMax && error > 0.0f) || (unsat < outMin && error < 0.0f)) {
            integralNext = integral_;  // freeze
        }
        integral_ = clampf(integralNext, -integralLimit, integralLimit);

        return clampf(feedforward + p + integral_, outMin, outMax);
    }

private:
    float outMin;
    float outMax;
    float integralLimit;
    float integral_;

    static bool validGain(float g) { return isfinite(g) && g >= 0.0f && g <= kGainMax; }
    static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
};
