#pragma once

#include <cmath>
#include <algorithm>
#include <chrono>

namespace viper {

/**
 * @brief Simple PID controller with integral windup limiting and derivative filtering
 * 
 * Computes control output u = Kp*e + Ki*integral(e) + Kd*derivative(e)
 * 
 * @note Uses wall-clock time for dt calculation (no global time variable)
 */
class PID {
public:
    float p;          ///< Proportional gain
    float i;          ///< Integral gain
    float d;          ///< Derivative gain
    float windup_limit;  ///< Integral windup limit (symmetric ±)
    float dt_max;     ///< Maximum dt threshold before reset (prevents jumps)
    float d_alpha;    ///< Derivative low-pass filter coefficient (0.0 to 1.0), 1.0 = no filter

    PID(float p_gain = 0.0f, float i_gain = 0.0f, float d_gain = 0.0f, 
        float windup = 0.0f, float d_lpf_alpha = 1.0f, float dt_max = 0.5f)
        : p(p_gain), i(i_gain), d(d_gain), windup_limit(windup), 
          dt_max(dt_max), d_alpha(d_lpf_alpha),
          integral_(0.0f), derivative_(0.0f), prev_error_(std::nan("")),
          prev_derivative_(0.0f), last_update_time_(clock::now()) {}

    /**
     * @brief Update PID controller and compute output
     * @param error Current control error
     * @return Control output signal
     */
    float update(float error) {
        auto now = clock::now();
        float dt = std::chrono::duration<float>(now - last_update_time_).count();
        last_update_time_ = now;

        // Reset if first call or dt too large (prevents integration jumps)
        if (std::isnan(prev_error_) || dt > dt_max) {
            integral_ = 0.0f;
            derivative_ = 0.0f;
        } else if (dt > 0.0f) {
            // Accumulate integral
            integral_ += error * dt;

            // Clamp integral to prevent windup
            if (windup_limit > 0.0f) {
                integral_ = std::clamp(integral_, -windup_limit, windup_limit);
            }

            // Compute raw derivative
            float raw_derivative = (error - prev_error_) / dt;

            // Low-pass filter on derivative term (first-order: d_new = alpha*d_raw + (1-alpha)*d_prev)
            derivative_ = d_alpha * raw_derivative + (1.0f - d_alpha) * prev_derivative_;
            prev_derivative_ = derivative_;
        }

        prev_error_ = error;

        // Compute control output
        float output = p * error + i * integral_ + d * derivative_;
        return output;
    }

    /**
     * @brief Reset controller state (integral, derivative, time tracking)
     */
    void reset() {
        integral_ = 0.0f;
        derivative_ = 0.0f;
        prev_error_ = std::nan("");
        prev_derivative_ = 0.0f;
        last_update_time_ = clock::now();
    }

private:
    using clock = std::chrono::high_resolution_clock;

    float integral_;           ///< Accumulated integral term
    float derivative_;         ///< Filtered derivative term
    float prev_error_;         ///< Previous error for derivative calculation
    float prev_derivative_;    ///< Previous derivative for low-pass filtering
    clock::time_point last_update_time_;  ///< Timestamp of last update
};

}  // namespace viper
