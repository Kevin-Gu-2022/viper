/**
 * EstimatorBase.h
 *
 * Pure computation object, composed inside viper::Node.
 * Fed from the existing IMU callback, read from ctrl_loop().
 * Used this instead of template for easier debugging.
 */

#pragma once

#include <cmath>
#include <optional>
#include <chrono>

#include <sensor_msgs/msg/imu.hpp>

#include "quaternion.h"
#include "vector.h"

namespace viper
{

// Struct to hold angular rate (used in control algo) and orientation
struct AttitudeEstimate
{
  Quaternion orientation;       // fused attitude
  Vector     angular_rate;      // body-frame rates (rad/s)
  Vector     euler;             // roll, pitch, yaw (rad)
  bool       valid{false};
};

class EstimatorBase
{
public:
  explicit EstimatorBase(char const * name) : _name{name} {}
  virtual ~EstimatorBase() = default; // Clean up child first

  // Prevent copying of this object
  EstimatorBase(EstimatorBase const &) = delete;
  EstimatorBase & operator=(EstimatorBase const &) = delete;

  /** Process and return estimate (bundled in AttitudeEstimate). Return nullopt 
   * if timestamp differences too quick or too far apart
   */
  std::optional<AttitudeEstimate> process(sensor_msgs::msg::Imu const & msg)
  {
    auto const stamp = std::chrono::seconds(msg.header.stamp.sec)
                     + std::chrono::nanoseconds(msg.header.stamp.nanosec);

    float dt = 0.0f;

    if (_sample_count > 0 && _prev_stamp.has_value())
    {
      auto const diff = stamp - _prev_stamp.value();
      dt = std::chrono::duration<float>(diff).count();
      
      if (dt < min_dt) return std::nullopt;

      if (dt > max_dt)
      {
        reset();
        return std::nullopt;
      }
    }

    _prev_stamp = stamp;
    _sample_count++;

    _estimate = on_update(msg, dt);
    _estimate.euler = _estimate.orientation.toEuler();

    return _estimate;
  }

  // Return the estimate variable when call estimate() function
  AttitudeEstimate const & estimate() const { return _estimate; }

  void reset()
  {
    _prev_stamp.reset();
    _sample_count = 0;
    _estimate = AttitudeEstimate{};
    on_reset();
  }

  // Return name of estimator (e.g. complementary, Kalman, etc.)
  char const * name() const { return _name; }

  float max_dt{0.10f};
  float min_dt{1e-6f};

protected:
  // Functions that derived classes must implement
  virtual AttitudeEstimate on_update(sensor_msgs::msg::Imu const & msg, float dt) = 0;
  virtual void on_reset() {}

private:
  char const * _name;
  AttitudeEstimate _estimate{};
  std::optional<std::chrono::nanoseconds> _prev_stamp{};
  size_t _sample_count{0};
};
}
