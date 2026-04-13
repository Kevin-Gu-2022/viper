#pragma once

#include "PID.h"
#include "MathUtilities.h"

namespace viper {

/**
 * @brief Attitude controller that produces target angular rates from attitude error
 * 
 * This is the outer loop in a cascaded control system:
 * - Input: current attitude (quaternion), target attitude (quaternion)
 * - Process: compute attitude error → apply PID gains → output target rates
 * - Output: target angular rates (rad/s)
 * 
 * The controller uses 3 separate PID loops:
 * - Roll PID: controls roll angle error → target roll rate
 * - Pitch PID: controls pitch angle error → target pitch rate
 * - Yaw PID: controls yaw angle error → target yaw rate
 */
class AttitudeController {
public:
    /**
     * @brief Constructor with default PID gains from flix reference
     * 
     * @param roll_p    Roll proportional gain (default: 6.0)
     * @param pitch_p   Pitch proportional gain (default: 6.0)
     * @param yaw_p     Yaw proportional gain (default: 3.0)
     */
    AttitudeController(float roll_p = 6.0f, float pitch_p = 6.0f, float yaw_p = 3.0f)
        : roll_pid_(roll_p, 0.0f, 0.0f),
          pitch_pid_(pitch_p, 0.0f, 0.0f),
          yaw_pid_(yaw_p, 0.0f, 0.0f) {}

    /**
     * @brief Update attitude controller and compute target rates
     * 
     * @param attitude_current Current quaternion attitude (w, x, y, z)
     * @param attitude_target  Target quaternion attitude (w, x, y, z)
     * @return Vector3 Target angular rates (x=roll_rate, y=pitch_rate, z=yaw_rate) in rad/s
     */
    Vector3 update(const Quaternion& attitude_current, const Quaternion& attitude_target) {
        // Compute attitude error as 3D rotation vector
        Vector3 error = attitude_current.attitude_error(attitude_target);

        // Apply PID gains to get target rates
        float roll_rate_target = roll_pid_.update(error.x);
        float pitch_rate_target = pitch_pid_.update(error.y);
        float yaw_rate_target = yaw_pid_.update(error.z);

        return Vector3(roll_rate_target, pitch_rate_target, yaw_rate_target);
    }

    /**
     * @brief Set all attitude controller gains
     */
    void set_gains(float roll_p, float pitch_p, float yaw_p) {
        roll_pid_.p = roll_p;
        pitch_pid_.p = pitch_p;
        yaw_pid_.p = yaw_p;
    }

    /**
     * @brief Get roll PID controller for parameter adjustment
     */
    PID& get_roll_pid() { return roll_pid_; }

    /**
     * @brief Get pitch PID controller for parameter adjustment
     */
    PID& get_pitch_pid() { return pitch_pid_; }

    /**
     * @brief Get yaw PID controller for parameter adjustment
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
    PID roll_pid_;   ///< PID controller for roll angle
    PID pitch_pid_;  ///< PID controller for pitch angle
    PID yaw_pid_;    ///< PID controller for yaw angle
};

}  // namespace viper
