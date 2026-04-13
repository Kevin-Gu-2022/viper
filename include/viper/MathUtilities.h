#pragma once

#include <cmath>
#include <array>
#include <algorithm>

namespace viper {

/**
 * @brief Simple 3D vector class for rotation and attitude control
 */
class Vector3 {
public:
    float x, y, z;

    Vector3(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    Vector3 operator/(float scalar) const {
        return Vector3(x / scalar, y / scalar, z / scalar);
    }

    float dot(const Vector3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    Vector3 cross(const Vector3& v) const {
        return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    float norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vector3 normalized() const {
        float n = norm();
        if (n > 1e-6f) {
            return *this / n;
        }
        return Vector3(0, 0, 0);
    }
};

/**
 * @brief Quaternion class for attitude representation and operations
 * 
 * Uses (w, x, y, z) convention where w is the scalar part
 */
class Quaternion {
public:
    float w, x, y, z;

    Quaternion(float w = 1.0f, float x = 0.0f, float y = 0.0f, float z = 0.0f) 
        : w(w), x(x), y(y), z(z) {}

    /**
     * @brief Create quaternion from ROS IMU message convention (x, y, z, w)
     */
    static Quaternion from_msg(float mx, float my, float mz, float mw) {
        return Quaternion(mw, mx, my, mz);
    }

    /**
     * @brief Get angle error as a 3D rotation vector using quaternion conjugate multiply
     * 
     * Computes the rotation vector from current attitude to target attitude.
     * This is the error that the attitude controller needs to correct.
     * 
     * @param target Target attitude quaternion
     * @return 3D error vector (rotation axis scaled by angle)
     */
    Vector3 attitude_error(const Quaternion& target) const {
        // q_error = target * q_current^-1
        // For normalized quaternions, inverse = conjugate
        Quaternion q_conj(w, -x, -y, -z);  // conjugate of current
        Quaternion q_error = target.multiply(q_conj);  // error quaternion

        // Shortest path check: Ensure we take the rotation < 180 degrees
        if (q_error.w < 0.0f) {
            q_error.w = -q_error.w;
            q_error.x = -q_error.x;
            q_error.y = -q_error.y;
            q_error.z = -q_error.z;
        }
        
        // Convert to rotation vector (angle-axis representation)
        // If q_error = [cos(θ/2), sin(θ/2)*n], then rotation vector = 2*θ*sin(θ/2)*n
        float sin_half_angle_sq = q_error.x * q_error.x + q_error.y * q_error.y + q_error.z * q_error.z;
        
        if (sin_half_angle_sq < 1e-6f) {
            // Small angle approximation
            return Vector3(2.0f * q_error.x, 2.0f * q_error.y, 2.0f * q_error.z);
        }

        float sin_half_angle = std::sqrt(sin_half_angle_sq);
        float half_angle = 2.0f * std::atan2(sin_half_angle, q_error.w);
        float scale = half_angle / sin_half_angle;
        
        return Vector3(scale * q_error.x, scale * q_error.y, scale * q_error.z);
    }

    /**
     * @brief Multiply two quaternions: q1 * q2
     */
    Quaternion multiply(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    /**
     * @brief Rotate a 3D vector by this quaternion
     */
    Vector3 rotate(const Vector3& v) const {
        // v' = q * v * q^-1 = q * v * q_conj (for unit quaternions)
        Quaternion v_quat(0, v.x, v.y, v.z);
        Quaternion q_conj(w, -x, -y, -z);
        Quaternion result = this->multiply(v_quat).multiply(q_conj);
        return Vector3(result.x, result.y, result.z);
    }

    /**
     * @brief Normalize quaternion to unit length
     */
    Quaternion normalized() const {
        float norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm > 1e-6f) {
            return Quaternion(w / norm, x / norm, y / norm, z / norm);
        }
        return Quaternion(1, 0, 0, 0);  // Identity if zero-length
    }

    /**
     * @brief Get Euler angles (roll, pitch, yaw) from quaternion
     * @return Vector3 with x=roll, y=pitch, z=yaw in radians
     */
    Vector3 to_euler() const {
        float roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        
        // Safety clamp to prevent NaN
        float pitch_val = 2 * (w * y - z * x);
        pitch_val = std::clamp(pitch_val, -1.0f, 1.0f);
        float pitch = std::asin(pitch_val);
        
        float yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        return Vector3(roll, pitch, yaw);
    }

private:
    /**
     * @brief Multiply this quaternion by another
     */
    Quaternion multiply_impl(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }
};

}  // namespace viper
