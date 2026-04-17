#pragma once

#include <cmath>
// #include <algorithm>
#include <chrono>

namespace viper {

    class PID {
    public:
        float p, i, d;
        float windup;
        float dtMax;

        float derivative = 0;
        float integral = 0;

        LowPassFilter<float> lpf;

        PID(float p, float i, float d, float windup = 0, float dAlpha = 1, float dtMax = 0.1)
            : p(p), i(i), d(d), windup(windup), lpf(dAlpha), dtMax(dtMax) {}

        float update(float error) {
            using clock = std::chrono::steady_clock;

            auto now = clock::now();

            float dt = 0.0f;

            if (initialized) {
                dt = std::chrono::duration<float>(now - prevTime).count();
            }

            if (initialized && dt > 0 && dt < dtMax) {
                integral += error * dt;
                derivative = lpf.update((error - prevError) / dt);
            } else {
                integral = 0;
                derivative = 0;
            }

            prevError = error;
            prevTime = now;
            initialized = true;

            return p * error
                + std::clamp(i * integral, -windup, windup)
                + d * derivative;
        }

        void reset() {
            initialized = false;
            integral = 0;
            derivative = 0;
            lpf.reset();
        }

    private:
        using clock = std::chrono::steady_clock;

        clock::time_point prevTime;
        float prevError = 0;
        bool initialized = false;
    };

}  // namespace viper
