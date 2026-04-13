#pragma once

#include "PID.h"
#include "MathUtilities.h"

namespace viper {

/**
 * @brief Angular rate controller that produces motor torques from rate error
 * 
 * This is the inner loop in a cascaded control system:
 * - Input: target angular rates (rad/s), current angular rates (rad/s from gyro)
 * - Process: compute rate error → apply PID gains → output torque commands
 * - Output: target torques (normalized to [-1, 1] range for motor mixing)
 * 
 * The controller uses 3 separate PID loops (one per axis):
 * - Roll rate PID: controls roll rate error → roll torque
 * - Pitch rate PID: controls pitch rate error → pitch torque
 * - Yaw rate PID: controls yaw rate error → yaw torque
 */
class RateController {
public:
    /**
     * @brief Constructor with default flix rate control gains
     * 
     * Rate controllers typically use higher gains to respond quickly to disturbances.
     * 
     * @param roll_p       Roll rate proportional gain (default: 0.05)
     * @param roll_i       Roll rate integral gain (default: 0.2)
     * @param roll_d       Roll rate derivative gain (default: 0.001)
     * @param pitch_p      Pitch rate proportional gain (default: 0.05)
     * @param pitch_i      Pitch rate integral gain (default: 0.2)
     * @param pitch_d      Pitch rate derivative gain (default: 0.001)
     * @param yaw_p        Yaw rate proportional gain (default: 0.3)
     * @param yaw_i        Yaw rate integral gain (default: 0.0)
     * @param yaw_d        Yaw rate derivative gain (default: 0.0)
     * @param windup       Integral windup limit (default: 0.3)
     */
    RateController(float roll_p = 0.05f, float roll_i = 0.2f, float roll_d = 0.001f,
                   float pitch_p = 0.05f, float pitch_i = 0.2f, float pitch_d = 0.001f,
                   float yaw_p = 0.3f, float yaw_i = 0.0f, float yaw_d = 0.0f,
                   float windup = 0.3f, float d_lpf_alpha = 0.2f)
        : roll_pid_(roll_p, roll_i, roll_d, windup, d_lpf_alpha),
          pitch_pid_(pitch_p, pitch_i, pitch_d, windup, d_lpf_alpha),
          yaw_pid_(yaw_p, yaw_i, yaw_d, windup, d_lpf_alpha) {}

    /**
     * @brief Update rate controller and compute target torques
     * 
     * @param rate_target   Target angular rates (rad/s) - from attitude controller
     * @param rate_current  Current angular rates (rad/s) - from gyroscope
     * @return Vector3 Target torques (x=roll_torque, y=pitch_torque, z=yaw_torque) in [-1, 1]
     */
    Vector3 update(const Vector3& rate_target, const Vector3& rate_current) {
        // Compute rate errors
        Vector3 error = rate_target - rate_current;

        // Apply PID gains to get target torques
        float roll_torque = roll_pid_.update(error.x);
        float pitch_torque = pitch_pid_.update(error.y);
        float yaw_torque = yaw_pid_.update(error.z);

        return Vector3(roll_torque, pitch_torque, yaw_torque);
    }

    /**
     * @brief Set all rate controller gains
     */
    void set_gains(float roll_p, float roll_i, float roll_d,
                   float pitch_p, float pitch_i, float pitch_d,
                   float yaw_p, float yaw_i, float yaw_d) {
        roll_pid_.p = roll_p;
        roll_pid_.i = roll_i;
        roll_pid_.d = roll_d;

        pitch_pid_.p = pitch_p;
        pitch_pid_.i = pitch_i;
        pitch_pid_.d = pitch_d;

        yaw_pid_.p = yaw_p;
        yaw_pid_.i = yaw_i;
        yaw_pid_.d = yaw_d;
    }

    /**
     * @brief Get roll rate PID controller for parameter adjustment
     */
    PID& get_roll_pid() { return roll_pid_; }

    /**
     * @brief Get pitch rate PID controller for parameter adjustment
     */
    PID& get_pitch_pid() { return pitch_pid_; }

    /**
     * @brief Get yaw rate PID controller for parameter adjustment
     */
    PID& get_yaw_pid() { return yaw_pid_; }

    /**
     * @brief Reset all PID controller states
     */
    void reset() {
        roll_pid_.reset();
        pitch_pid_.reset();
        yaw_pid_.reset();
    }

private:
    PID roll_pid_;   ///< PID controller for roll rate
    PID pitch_pid_;  ///< PID controller for pitch rate
    PID yaw_pid_;    ///< PID controller for yaw rate
};

}  // namespace viper
