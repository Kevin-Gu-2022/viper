/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/viper/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <viper/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace viper
{

/**************************************************************************************
 * CTOR/DTOR 
 **************************************************************************************/

Node::Node()
: rclcpp::Node("viper_node")
, _node_heap{}
, _node_hdl{_node_heap.data(),
            _node_heap.size(),
            [this] () { return micros(); },
            [this] (CanardFrame const & frame) { return _can_mgr->transmit(frame); },
            cyphal::Node::DEFAULT_NODE_ID,
            CYPHAL_TX_QUEUE_SIZE,
            CYPHAL_RX_QUEUE_SIZE,
            cyphal::Node::DEFAULT_MTU_SIZE}
, _node_mtx{}
, _node_start{std::chrono::steady_clock::now()}
, _teleop_qos_profile{rclcpp::KeepLast(10)}
, _teleop_sub_options{}
, _teleop_sub{}
, _target_linear_velocity{}  // Zero target linear velocity (PS3 controller)
, _target_angular_velocity{}  // Zero target angular velocity (PS3 controller)
, _armed(false)
, _imu_qos_profile{rclcpp::KeepLast(10), rmw_qos_profile_sensor_data}
, _attitude_target(1.0f, 0.0f, 0.0f, 0.0f)  // Initialize to level attitude (identity quaternion)
, _thrust_target(0.0f)
{
  init_cyphal_heartbeat();
  init_cyphal_node_info();

  declare_parameter("can_iface", "can0");
  declare_parameter("can_node_id", 100);

  RCLCPP_INFO(get_logger(),
              "configuring CAN2233 bus:\n\tDevice: %s\n\tNode Id: %ld",
              get_parameter("can_iface").as_string().c_str(),
              get_parameter("can_node_id").as_int());

  _node_hdl.setNodeId(get_parameter("can_node_id").as_int());

  _can_mgr = std::make_unique<CanManager>(
    get_logger(),
    get_parameter("can_iface").as_string(),
    [this](CanardFrame const & frame)
    {
      std::lock_guard<std::mutex> lock(_node_mtx);
      _node_hdl.onCanFrameReceived(frame);
    });

  _node_loop_timer = create_wall_timer(NODE_LOOP_RATE,
                                       [this]()
                                       {
                                         std::lock_guard <std::mutex> lock(_node_mtx);
                                         _node_hdl.spinSome();
                                       });

  // Initialise the state estimator object (estimator can be interchanged here)
  _estimator = std::make_unique<ComplementaryFilter>();

  init_teleop_sub();
  init_joy_sub();
  init_imu_sub();

  // Initialize attitude control parameters
  declare_control_parameters();

  _ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->ctrl_loop(); });

  _cyphal_demo_pub = _node_hdl.create_publisher<uavcan::primitive::scalar::Integer8_1_0>(CYPHAL_DEMO_PORT_ID, 1*1000*1000UL);

  _setpoint_velocity_pub_1 = _node_hdl.create_publisher<zubax::primitive::real16::Vector4_1_0>(SETPOINT_VELOCITY_ID_1, 1*1000*1000UL);
  _setpoint_velocity_pub_2 = _node_hdl.create_publisher<zubax::primitive::real16::Vector4_1_0>(SETPOINT_VELOCITY_ID_2, 1*1000*1000UL);
  _setpoint_velocity_pub_3 = _node_hdl.create_publisher<zubax::primitive::real16::Vector4_1_0>(SETPOINT_VELOCITY_ID_3, 1*1000*1000UL);
  _setpoint_velocity_pub_4 = _node_hdl.create_publisher<zubax::primitive::real16::Vector4_1_0>(SETPOINT_VELOCITY_ID_4, 1*1000*1000UL);


  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "%s shut down successfully.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_cyphal_heartbeat()
{
  _cyphal_heartbeat_pub = _node_hdl.create_publisher<uavcan::node::Heartbeat_1_0>(1*1000*1000UL /* = 1 sec in usecs. */);

  _cyphal_heartbeat_timer = create_wall_timer(CYPHAL_HEARTBEAT_PERIOD,
                                              [this]()
                                              {
                                                uavcan::node::Heartbeat_1_0 msg;

                                                auto const now = std::chrono::steady_clock::now();

                                                msg.uptime = std::chrono::duration_cast<std::chrono::seconds>(now - _node_start).count();
                                                msg.health.value = uavcan::node::Health_1_0::NOMINAL;
                                                msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
                                                msg.vendor_specific_status_code = 0;

                                                {
                                                  std::lock_guard <std::mutex> lock(_node_mtx);
                                                  _cyphal_heartbeat_pub->publish(msg);

                                                }
                                              });
}

void Node::init_cyphal_node_info()
{
  _cyphal_node_info = _node_hdl.create_node_info(
    /* uavcan.node.Version.1.0 protocol_version */
    1, 0,
    /* uavcan.node.Version.1.0 hardware_version */
    1, 0,
    /* uavcan.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    std::array<uint8_t, 16>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F},
    /* saturated uint8[<=50] name */
    "107-systems.viper"
  );
}

CanardMicrosecond Node::micros()
{
  auto const now = std::chrono::steady_clock::now();
  auto const node_uptime = (now - _node_start);
  return std::chrono::duration_cast<std::chrono::microseconds>(node_uptime).count();
}

void Node::init_teleop_sub()
{
  // Declare default parameters for teleop topic and QoS settings
  declare_parameter("teleop_topic", "cmd_vel_head");
  declare_parameter("teleop_topic_deadline_ms", 100);
  declare_parameter("teleop_topic_liveliness_lease_duration", 1000);

  // Use parameters declared in viper-quad.py to confgure subscriptions
  auto const teleop_topic = get_parameter("teleop_topic").as_string();
  auto const teleop_topic_deadline = std::chrono::milliseconds(get_parameter("teleop_topic_deadline_ms").as_int());
  auto const teleop_topic_liveliness_lease_duration = std::chrono::milliseconds(get_parameter("teleop_topic_liveliness_lease_duration").as_int());

  _teleop_qos_profile.deadline(teleop_topic_deadline);
  _teleop_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
  _teleop_qos_profile.liveliness_lease_duration(teleop_topic_liveliness_lease_duration);

  // Called when deadline missed, though currently set to 0ms, i.e. no deadline. Currently, this callback will never trigger
  _teleop_sub_options.event_callbacks.deadline_callback =
    [this, teleop_topic](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5*1000UL,
                            "deadline missed for \"%s\" (total_count: %d, total_count_change: %d).",
                            teleop_topic.c_str(), event.total_count, event.total_count_change);

      _target_linear_velocity.reset();
      _target_angular_velocity.reset();

      // Reset attitude and thrust targets to safe values
      _attitude_target = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // Identity quaternion (level)
      _thrust_target = 0.0f;  // No thrust
    };

  // Called when teleop publisher stops asserting liveliness
  _teleop_sub_options.event_callbacks.liveliness_callback =
    [this, teleop_topic](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      if (event.alive_count > 0)
      {
        // At least one node is publishing to teleop topic
        RCLCPP_INFO(get_logger(), "liveliness gained for \"%s\"", teleop_topic.c_str());
      }
      else
      {
        // No nodes publishing to teleop topic, i.e. joystick node not running / disconnected
        RCLCPP_WARN(get_logger(), "liveliness lost for \"%s\"", teleop_topic.c_str());

        _target_linear_velocity.reset();
        _target_angular_velocity.reset();

        // Reset attitude and thrust targets to safe values
        _attitude_target = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // Identity quaternion (level)
        _thrust_target = 0.0f;  // No thrust
      }
    };

  // Create subscription to Twist messages from joystick  
  _teleop_sub = create_subscription<geometry_msgs::msg::Twist>(
    teleop_topic,
    _teleop_qos_profile,
    [this](geometry_msgs::msg::Twist::SharedPtr const msg)
    {
      // Only process inputs if motors are enabled
      if (!_armed) {
        // Disarm motors when not enabled
        _attitude_target = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);  // Identity quaternion (level)
        _thrust_target = 0.0f;  // No thrust
        return;
      }

      // Store velocity commands (could just use an assignment method in Vector class...)
      _target_linear_velocity.x = static_cast<double>(msg->linear.x);
      _target_linear_velocity.y = static_cast<double>(msg->linear.y);
      _target_linear_velocity.z = static_cast<double>(msg->linear.z);

      _target_angular_velocity.x = static_cast<double>(msg->angular.x);
      _target_angular_velocity.y = static_cast<double>(msg->angular.y);
      _target_angular_velocity.z = static_cast<double>(msg->angular.z);

      // Debug: Log controller input changes
      RCLCPP_INFO(get_logger(),
                   "PS3 Controller Input - Linear: [x=%.2f, y=%.2f, z=%.2f] m/s | Angular: [x=%.2f, y=%.2f, z=%.2f] rad/s",
                   msg->linear.x, msg->linear.y, msg->linear.z,
                   msg->angular.x, msg->angular.y, msg->angular.z);

      // === Convert velocity commands to attitude and thrust targets ===
      
      // Get max tilt angle from parameters
      float max_tilt = static_cast<float>(get_parameter("max_tilt_angle").as_double());

      // Compute desired attitude from linear velocities
      // linear.x → pitch (forward/backward)
      // linear.y → roll (lateral)
      // These two are scaled to not exceed max_tilt_angle
      float pitch_target = 0.0f;
      float roll_target = 0.0f;

      // Simple velocity to angle mapping (can be tuned)
      // Assuming teleop max velocities are around ±1 m/s
      if (std::abs(msg->linear.x) > 1e-3 || std::abs(msg->linear.y) > 1e-3) {
        float vz_max = 1.0f;  // Expected max velocity in m/s
        pitch_target = std::clamp(static_cast<float>(msg->linear.x) / vz_max, -1.0f, 1.0f) * max_tilt;
        roll_target = -std::clamp(static_cast<float>(msg->linear.y) / vz_max, -1.0f, 1.0f) * max_tilt;
      }

      // Missing this implementation for resetting yaw. Not really needed
      //   if (!armed || yaw_target != 0) yawTarget = attitude.getYaw();

      _attitude_target = Quaternion::fromEuler(Vector(roll_target, pitch_target, _target_angular_velocity.z));

      // Map thrust: linear.z velocity (up/down) to throttle [0, 1]
      // Assuming 0 m/s = hover (0.5), positive = up, negative = down
      _thrust_target = std::clamp(0.5f + static_cast<float>(msg->linear.z) * 0.5f, 0.0f, 1.0f);
    },
    _teleop_sub_options);
}

void Node::init_joy_sub()
{
  _joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    "joy",
    rclcpp::QoS(10),
    [this](sensor_msgs::msg::Joy::SharedPtr const msg)
    {
      // Check enable button (button 7 = Right Bumper/R1)
      const int ENABLE_BUTTON_INDEX = 7;
      if (msg->buttons.size() > ENABLE_BUTTON_INDEX) {
        bool enable_pressed = (msg->buttons[ENABLE_BUTTON_INDEX] == 1);
        _armed = enable_pressed;
        
        // If deadman switch released, reset target velocities/attitude and reset controllers
        if (!enable_pressed) {
          // Targets from joystick
          _target_linear_velocity.reset();
          _target_angular_velocity.reset();

          // Targets used in ctrl loop
          _thrust_target = 0.0f;
          _attitude_target = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);

          _attitude_controller.reset();
          _rate_controller.reset();

          _estimator->reset(); // Flix would continue to perform estimation...
        }
      }
  });
}

void Node::init_imu_sub()
{
  declare_parameter("imu_topic", "imu");
  declare_parameter("imu_topic_deadline_ms", 100);
  declare_parameter("imu_topic_liveliness_lease_duration", 1000);

  auto const imu_topic = get_parameter("imu_topic").as_string();
  auto const imu_topic_deadline = std::chrono::milliseconds(get_parameter("imu_topic_deadline_ms").as_int());
  auto const imu_topic_liveliness_lease_duration = std::chrono::milliseconds(get_parameter("imu_topic_liveliness_lease_duration").as_int());

  _imu_qos_profile.deadline(imu_topic_deadline);
  // _imu_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  // Likely will need to setup some sort of liveliness QOS later
  _imu_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
  _imu_qos_profile.liveliness_lease_duration(imu_topic_liveliness_lease_duration);

  _imu_sub_options.event_callbacks.deadline_callback =
    [this, imu_topic](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5*1000UL,
                            "deadline missed for \"%s\" (total_count: %d, total_count_change: %d).",
                            imu_topic.c_str(), event.total_count, event.total_count_change);
    };

  _imu_sub_options.event_callbacks.liveliness_callback =
    [this, imu_topic](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      if (event.alive_count > 0)
      {
        RCLCPP_INFO(get_logger(), "liveliness gained for \"%s\"", imu_topic.c_str());
      }
      else
      {
        RCLCPP_WARN(get_logger(), "liveliness lost for \"%s\"", imu_topic.c_str());
      }
    };

  _imu_sub = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic,
    _imu_qos_profile,
    [this](sensor_msgs::msg::Imu::SharedPtr const msg)
    {
    //   _imu_data = *msg;
      _estimator->process(*msg);

      // RCLCPP_INFO(get_logger(),
      //             "IMU Pose (x,y,z,w): %0.2f %0.2f %0.2f %0.2f",
      //             _imu_data.orientation.x,
      //             _imu_data.orientation.y,
      //             _imu_data.orientation.z,
      //             _imu_data.orientation.w);
    },
    _imu_sub_options);
}

void Node::ctrl_loop()
{

  // !TODO Add somewhere here: return if landed
  
  // This only fetches latest estimate. The estimator is decoupled
  // and constantly running while new IMU data coming in 
  auto const & est = _estimator->estimate();
  if (!est.valid) return;
  
  Quaternion attitude_current = est.orientation;
  Vector     rate_current     = est.angular_rate;

  // === Cascaded PID Control Loop ===

  // Level 1: Attitude Controller
  // Inputs: current attitude, target attitude
  // Output: target angular rates
  Vector rate_target = _attitude_controller.update(attitude_current, _attitude_target);

  // Level 2: Rate Controller
  // Inputs: target rates, current rates
  // Output: target torques
  Vector torque_target = _rate_controller.update(rate_target, rate_current);

  // Motor Mixer
  // Inputs: thrust, torques
  // Output: 4 motor commands [0, 1]
  auto motor_commands = _motor_mixer.mix(_thrust_target, torque_target);

  // Scale motor commands to appropriate range for Cyphal message
  // The motor controller expects values in some range (e.g., 0-255 or 0-1)
  // Adjust the scaling factor based on your motor controller's input requirements
  const float MOTOR_SCALE = 1000.0f;  // Scale to 0-1000 range for visualization/control
  
  zubax::primitive::real16::Vector4_1_0 motor_msg{
    motor_commands[0] * MOTOR_SCALE,
    motor_commands[1] * MOTOR_SCALE,
    motor_commands[2] * MOTOR_SCALE,
    motor_commands[3] * MOTOR_SCALE
  };

  // Publish motor commands to all 4 motors
  _setpoint_velocity_pub_1->publish(motor_msg);
  _setpoint_velocity_pub_2->publish(motor_msg);
  _setpoint_velocity_pub_3->publish(motor_msg);
  _setpoint_velocity_pub_4->publish(motor_msg);

  // Optional: Log controller state for debugging
  if (true) {  // Set to true for verbose logging
    Vector attitude_euler = attitude_current.toEuler();
    RCLCPP_INFO(get_logger(),
      "Attitude (deg): roll=%.1f pitch=%.1f yaw=%.1f | "
      "Rates (rad/s): x=%.2f y=%.2f z=%.2f | "
      "Motors: [%.2f, %.2f, %.2f, %.2f]",
      attitude_euler.x * 180 / M_PI, attitude_euler.y * 180 / M_PI, attitude_euler.z * 180 / M_PI,
      rate_current.x, rate_current.y, rate_current.z,
      motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]
    );
  }
}


void Node::declare_control_parameters()
{
  // Attitude controller parameters
  declare_parameter("attitude_roll_p", 6.0);
  declare_parameter("attitude_pitch_p", 6.0);
  declare_parameter("attitude_yaw_p", 3.0);

  // Rate controller parameters (with default flix values)
  declare_parameter("rate_roll_p", 0.05);
  declare_parameter("rate_roll_i", 0.2);
  declare_parameter("rate_roll_d", 0.001);

  declare_parameter("rate_pitch_p", 0.05);
  declare_parameter("rate_pitch_i", 0.2);
  declare_parameter("rate_pitch_d", 0.001);

  declare_parameter("rate_yaw_p", 0.3);
  declare_parameter("rate_yaw_i", 0.0);
  declare_parameter("rate_yaw_d", 0.0);

  declare_parameter("rate_integral_windup", 0.3);
  declare_parameter("rate_derivative_filter_alpha", 0.2);

  // Velocity to attitude mapping
  declare_parameter("max_tilt_angle", 0.5236);  // 30 degrees in radians

  // Load and apply parameters
  on_parameter_changed();

  // Add parameter change callback for live tuning
  auto handle = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
    (void)params;  // Suppress unused parameter warning
    on_parameter_changed();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";
    return result;
  });
}


void Node::on_parameter_changed()
{
  // Update attitude controller gains
  _attitude_controller.set_gains(
    static_cast<float>(get_parameter("attitude_roll_p").as_double()),
    static_cast<float>(get_parameter("attitude_pitch_p").as_double()),
    static_cast<float>(get_parameter("attitude_yaw_p").as_double())
  );

  // Update rate controller gains
  _rate_controller.set_gains(
    static_cast<float>(get_parameter("rate_roll_p").as_double()),
    static_cast<float>(get_parameter("rate_roll_i").as_double()),
    static_cast<float>(get_parameter("rate_roll_d").as_double()),
    
    static_cast<float>(get_parameter("rate_pitch_p").as_double()),
    static_cast<float>(get_parameter("rate_pitch_i").as_double()),
    static_cast<float>(get_parameter("rate_pitch_d").as_double()),
    
    static_cast<float>(get_parameter("rate_yaw_p").as_double()),
    static_cast<float>(get_parameter("rate_yaw_i").as_double()),
    static_cast<float>(get_parameter("rate_yaw_d").as_double())
  );

  // Update rate controller PID windup and filter settings
  _rate_controller.get_roll_pid().windup_limit = 
    static_cast<float>(get_parameter("rate_integral_windup").as_double());
  _rate_controller.get_pitch_pid().windup_limit = 
    static_cast<float>(get_parameter("rate_integral_windup").as_double());
  _rate_controller.get_yaw_pid().windup_limit = 
    static_cast<float>(get_parameter("rate_integral_windup").as_double());

    // Should be setting the cutoff instead
  // _rate_controller.get_roll_pid().d_alpha = 
  //   static_cast<float>(get_parameter("rate_derivative_filter_alpha").as_double());
  // _rate_controller.get_pitch_pid().d_alpha = 
  //   static_cast<float>(get_parameter("rate_derivative_filter_alpha").as_double());
  // _rate_controller.get_yaw_pid().d_alpha = 
  //   static_cast<float>(get_parameter("rate_derivative_filter_alpha").as_double());
}


/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* viper */
