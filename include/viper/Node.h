 /**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/viper/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <actuator_msgs/msg/actuators.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cyphal++/cyphal++.h>

#include <mp-units/systems/si/si.h>
#include <mp-units/systems/angular/angular.h>

#include "CanManager.h"
#include "vector.h"
#include "ComplementaryFilter.h"
#include "ExternalEstimator.h"
#include "CascadedController.h"
#include "MotorMixer.h"


/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace mp_units;
using mp_units::si::unit_symbols::m;
using mp_units::si::unit_symbols::s;
using mp_units::angular::unit_symbols::rad;

namespace viper
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
   Node();
  ~Node();

private:
  static size_t constexpr CYPHAL_O1HEAP_SIZE = (cyphal::Node::DEFAULT_O1HEAP_SIZE * 16);
  static size_t constexpr CYPHAL_TX_QUEUE_SIZE = 256;
  static size_t constexpr CYPHAL_RX_QUEUE_SIZE = 256;
  cyphal::Node::Heap<CYPHAL_O1HEAP_SIZE> _node_heap;
  cyphal::Node _node_hdl;
  std::mutex _node_mtx;
  std::chrono::steady_clock::time_point const _node_start;
  std::unique_ptr<CanManager> _can_mgr;
  static std::chrono::microseconds constexpr NODE_LOOP_RATE{100};
  rclcpp::TimerBase::SharedPtr _node_loop_timer;

  cyphal::Publisher<uavcan::node::Heartbeat_1_0> _cyphal_heartbeat_pub;
  static std::chrono::milliseconds constexpr CYPHAL_HEARTBEAT_PERIOD{1000};
  rclcpp::TimerBase::SharedPtr _cyphal_heartbeat_timer;
  void init_cyphal_heartbeat();

  cyphal::NodeInfo _cyphal_node_info;
  void init_cyphal_node_info();

  CanardMicrosecond micros();

  rclcpp::QoS _teleop_qos_profile;
  rclcpp::SubscriptionOptions _teleop_sub_options;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _teleop_sub;

  // Vectors representing target given from PS3 controller
  Vector _target_linear_velocity;
  Vector _target_angular_velocity;

  void init_teleop_sub();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;
  bool _armed;
  void init_joy_sub();

  rclcpp::QoS _imu_qos_profile;
  rclcpp::SubscriptionOptions _imu_sub_options;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
//   sensor_msgs::msg::Imu _imu_data; // Removed. Replaced with new Estimator class
  void init_imu_sub();


  static uint16_t constexpr CYPHAL_DEMO_PORT_ID = 1234;
  cyphal::Publisher<uavcan::primitive::scalar::Integer8_1_0> _cyphal_demo_pub;

  static uint16_t constexpr SETPOINT_VELOCITY_ID_1 = 113;
  cyphal::Publisher<zubax::primitive::real16::Vector4_1_0> _setpoint_velocity_pub_1;

  static uint16_t constexpr SETPOINT_VELOCITY_ID_2 = 114;
  cyphal::Publisher<zubax::primitive::real16::Vector4_1_0> _setpoint_velocity_pub_2;


  static uint16_t constexpr SETPOINT_VELOCITY_ID_3 = 115;
  cyphal::Publisher<zubax::primitive::real16::Vector4_1_0> _setpoint_velocity_pub_3;

  static uint16_t constexpr SETPOINT_VELOCITY_ID_4 = 116;
  cyphal::Publisher<zubax::primitive::real16::Vector4_1_0> _setpoint_velocity_pub_4;

  // Publisher for Gazebo motor commands (bridged to Ignition)
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr _gazebo_motor_pub;

  // Attitude control system (default constructor called)
  AttitudeController _attitude_controller;
  RateController _rate_controller;
  MotorMixer _motor_mixer;
  
  // Control targets
  Quaternion _attitude_target;  // Target attitude (identity = level)
  float _thrust_target = 0.0f;  // Target collective thrust [0, 1]
  
  // Declare parameters for tuning
  void declare_control_parameters();
  void on_parameter_changed();

  // ctrl_loop variables
  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;

  // Estimator object to perform state estimation
  std::unique_ptr<EstimatorBase> _estimator;

  void ctrl_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* viper */