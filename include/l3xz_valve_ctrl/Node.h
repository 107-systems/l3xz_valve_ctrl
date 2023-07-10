/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_valve_ctrl/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <tuple>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <l3xz/l3xz.h>

#include <ros2_heartbeat/publisher/Publisher.h>
#include <ros2_loop_rate_monitor/Monitor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
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
  heartbeat::Publisher::SharedPtr _heartbeat_pub;
  void init_heartbeat();

  std::map<HydraulicLegJointKey, float> _angle_actual_rad_map;
  std::map<HydraulicLegJointKey,
           rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_actual_sub;
  std::map<HydraulicLegJointKey,
           std::optional<std::chrono::steady_clock::time_point>> _opt_last_angle_actual_msg;
  std::map<HydraulicLegJointKey, float> _angle_target_rad_map;
  std::map<HydraulicLegJointKey,
           rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_target_sub;
  std::map<HydraulicLegJointKey,
           std::optional<std::chrono::steady_clock::time_point>> _opt_last_angle_target_msg;
  void init_sub();

  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr _servo_pulse_width_pub;
  void init_pub();

  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  loop_rate::Monitor::SharedPtr _ctrl_loop_rate_monitor;
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void ctrl_loop();

  std::map<HydraulicLegJointKey, float> _angle_error_sum_deg_map;
  void init_error_sum_map();

  typedef std::array<uint16_t, 12> ServoPulseWidth;
  ServoPulseWidth _servo_pulse_width;

  enum class State { Init, Control };
  State _state;
  State handle_Init();
  State handle_Control();

  static std_msgs::msg::UInt16MultiArray toServoPulseWidthMessage(ServoPulseWidth const & servo_pulse_width);

  static uint16_t constexpr SERVO_PULSE_WIDTH_MIN_us     = 1000U;
  static uint16_t constexpr SERVO_PULSE_WIDTH_NEUTRAL_us = 1500U;
  static uint16_t constexpr SERVO_PULSE_WIDTH_MAX_us     = 2000U;

  ServoPulseWidth const DEFAULT_SERVO_PULSE_WIDTH = {
    SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us,
    SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us,
    SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us
  };
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
