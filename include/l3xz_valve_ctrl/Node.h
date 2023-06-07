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

#include <ros2_heartbeat/Publisher.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * TYPEDEFS
 **************************************************************************************/

enum class HydraulicJoint { Femur, Tibia };
enum class Leg { LeftFront, LeftMiddle, LeftBack, RightFront, RightMiddle, RightBack };
typedef std::tuple<Leg, HydraulicJoint> HydraulicLegJointKey;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
  Node();
  ~Node();


private:
  static std::chrono::milliseconds constexpr HEARTBEAT_LOOP_RATE{100};
  heartbeat::Publisher::SharedPtr _heartbeat_pub;
  void init_heartbeat();

  std::map<HydraulicLegJointKey, float> _angle_actual_rad_map;
  std::map<HydraulicLegJointKey,
           rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_actual_sub;
  std::map<HydraulicLegJointKey, float> _angle_target_rad_map;
  std::map<HydraulicLegJointKey,
           rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_target_sub;
  void init_sub();

  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr _servo_pulse_width_pub;
  void init_pub();
  void publish_servo_pulse_width();

  std::chrono::steady_clock::time_point _prev_ctrl_loop_timepoint;
  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void ctrl_loop();

  typedef std::array<uint16_t, 12> ServoPulseWidth;
  ServoPulseWidth _servo_pulse_width;

  enum class State { Init, Control };
  State _state;
  State handle_Init();
  State handle_Control();

  static uint16_t constexpr SERVO_PULSE_WIDTH_MIN_us     = 1000U;
  static uint16_t constexpr SERVO_PULSE_WIDTH_NEUTRAL_us = 1500U;
  static uint16_t constexpr SERVO_PULSE_WIDTH_MAX_us     = 2000U;

  ServoPulseWidth const DEFAULT_SERVO_PULSE_WIDTH = {
    SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us,
    SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us,
    SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us, SERVO_PULSE_WIDTH_NEUTRAL_us
  };

  static ServoPulseWidth const calc_ServoPulseWidth(std::map<HydraulicLegJointKey, float> const & angle_actual_rad_map,
                                                    std::map<HydraulicLegJointKey, float> const & angle_target_rad_map);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
