/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_valve_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_valve_ctrl/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * MODULE INTERNAL FUNCTIONS
 **************************************************************************************/

static std::string JointToStr(HydraulicJoint const joint)
{
  switch(joint)
  {
    case HydraulicJoint::Femur: return std::string("femur"); break;
    case HydraulicJoint::Tibia: return std::string("tibia"); break;
    default: __builtin_unreachable();
  }
}

static std::string LegToStr(Leg const leg)
{
  switch(leg)
  {
    case Leg::LeftFront:   return std::string("left_front");   break;
    case Leg::LeftMiddle:  return std::string("left_middle");  break;
    case Leg::LeftBack:    return std::string("left_back");    break;
    case Leg::RightFront:  return std::string("right_front");  break;
    case Leg::RightMiddle: return std::string("right_middle"); break;
    case Leg::RightBack:   return std::string("right_back");   break;
    default: __builtin_unreachable();
  }
}

inline HydraulicLegJointKey make_key(Leg const leg, HydraulicJoint const joint)
{
  return std::tuple(leg, joint);
}

struct leg_joint_map_key_equal : public std::binary_function<HydraulicLegJointKey, HydraulicLegJointKey, bool>
{
  bool operator()(const HydraulicLegJointKey & v0, const HydraulicLegJointKey & v1) const
  {
    return (
      std::get<0>(v0) == std::get<0>(v1) &&
      std::get<1>(v0) == std::get<1>(v1)
    );
  }
};

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static std::list<Leg> const LEG_LIST =
{
  Leg::LeftFront, Leg::LeftMiddle, Leg::LeftBack, Leg::RightFront, Leg::RightMiddle, Leg::RightBack
};

static std::list<HydraulicJoint> const HYDRAULIC_JOINT_LIST =
{
  HydraulicJoint::Femur, HydraulicJoint::Tibia
};

static std::map<HydraulicLegJointKey, size_t> const LEG_JOINT_to_SERVO_NUM_MAP =
  {
    {make_key(Leg::RightBack,   HydraulicJoint::Tibia),  0},
    {make_key(Leg::RightBack,   HydraulicJoint::Femur),  1},
    {make_key(Leg::RightMiddle, HydraulicJoint::Tibia),  2},
    {make_key(Leg::RightMiddle, HydraulicJoint::Femur),  3},
    {make_key(Leg::RightFront,  HydraulicJoint::Tibia),  4},
    {make_key(Leg::RightFront,  HydraulicJoint::Femur),  5},
    {make_key(Leg::LeftFront,   HydraulicJoint::Femur),  6},
    {make_key(Leg::LeftFront,   HydraulicJoint::Tibia),  7},
    {make_key(Leg::LeftMiddle,  HydraulicJoint::Femur),  8},
    {make_key(Leg::LeftMiddle,  HydraulicJoint::Tibia),  9},
    {make_key(Leg::LeftBack,    HydraulicJoint::Femur), 10},
    {make_key(Leg::LeftBack,    HydraulicJoint::Tibia), 11}
  };

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_valve_ctrl")
, _prev_ctrl_loop_timepoint{std::chrono::steady_clock::now()}
, _servo_pulse_width{DEFAULT_SERVO_PULSE_WIDTH}
, _state{State::Init}
{
  init_heartbeat();
  init_sub();
  init_pub();

  init_error_sum_map();

  _ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->ctrl_loop(); });

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "%s shut down.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_heartbeat()
{
  std::stringstream heartbeat_topic;
  heartbeat_topic << "/l3xz/" << get_name() << "/heartbeat";

  _heartbeat_pub = heartbeat::Publisher::create(*this, heartbeat_topic.str(), HEARTBEAT_LOOP_RATE);
}

void Node::init_sub()
{
  for (auto leg : LEG_LIST)
    for (auto joint : HYDRAULIC_JOINT_LIST)
    {
      {
        std::stringstream angle_actual_sub_topic;
        angle_actual_sub_topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/actual";

        _angle_actual_rad_map[make_key(leg, joint)] = 0.0f;

        _opt_last_angle_actual_msg[make_key(leg, joint)] = std::nullopt;

        _angle_actual_sub[make_key(leg, joint)] = create_subscription<std_msgs::msg::Float32>(
          angle_actual_sub_topic.str(),
          1,
          [this, leg, joint](std_msgs::msg::Float32::SharedPtr const msg)
          {
            _angle_actual_rad_map.at(make_key(leg, joint)) = msg->data;
            _opt_last_angle_actual_msg.at(make_key(leg, joint)) = std::chrono::steady_clock::now();
          });
      }
      {
        std::stringstream angle_target_sub_topic;
        angle_target_sub_topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/target";

        _angle_target_rad_map[make_key(leg, joint)] = 0.0f;

        _opt_last_angle_target_msg[make_key(leg, joint)] = std::nullopt;

        _angle_target_sub[make_key(leg, joint)] = create_subscription<std_msgs::msg::Float32>(
          angle_target_sub_topic.str(),
          1,
          [this, leg, joint](std_msgs::msg::Float32::SharedPtr const msg)
          {
            _angle_target_rad_map.at(make_key(leg, joint)) = msg->data;
            _opt_last_angle_target_msg.at(make_key(leg, joint)) = std::chrono::steady_clock::now();
          });
      }
    }
}

void Node::init_pub()
{
  _servo_pulse_width_pub = create_publisher<std_msgs::msg::UInt16MultiArray>("/l3xz/servo_pulse_width/target", 1);
}

void Node::ctrl_loop()
{
  auto const now = std::chrono::steady_clock::now();
  auto const ctrl_loop_rate = (now - _prev_ctrl_loop_timepoint);
  if (ctrl_loop_rate > (CTRL_LOOP_RATE + std::chrono::milliseconds(1)))
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "ctrl_loop should be called every %ld ms, but is %ld ms instead",
                         CTRL_LOOP_RATE.count(),
                         std::chrono::duration_cast<std::chrono::milliseconds>(ctrl_loop_rate).count());
  _prev_ctrl_loop_timepoint = now;


  /* Perform state dependent actions. */
  switch (_state)
  {
    case State::Init:    _state = handle_Init();    break;
    case State::Control: _state = handle_Control(); break;
    default: __builtin_unreachable(); break;
  }

  auto const servo_pulse_width_msg = toServoPulseWidthMessage(_servo_pulse_width);
  _servo_pulse_width_pub->publish(servo_pulse_width_msg);
}

void Node::init_error_sum_map()
{
  for (auto leg : LEG_LIST)
    for (auto joint : HYDRAULIC_JOINT_LIST)
      _angle_error_sum_rad_map[make_key(leg, joint)] = 0.0f;
}

Node::State Node::handle_Init()
{
  _servo_pulse_width = DEFAULT_SERVO_PULSE_WIDTH;

  /* Check if data from all subscribed topics has arrived. Before
   * this pre-condition is not met it makes no sense to do any
   * actual servo actuator control.
   */
  std::stringstream angle_actual_no_message_list,
                    angle_target_no_message_list;

  bool angle_actual_no_message = false,
       angle_target_no_message = false;

  for (auto leg : LEG_LIST)
    for (auto joint : HYDRAULIC_JOINT_LIST)
    {
      if (!_opt_last_angle_actual_msg.at(make_key(leg, joint)).has_value())
      {
        angle_actual_no_message = true;
        angle_actual_no_message_list << LegToStr(leg) << ":" << JointToStr(joint) << " ";
      }
      if (!_opt_last_angle_target_msg.at(make_key(leg, joint)).has_value())
      {
        angle_target_no_message = true;
        angle_target_no_message_list << LegToStr(leg) << ":" << JointToStr(joint) << " ";
      }
    }

  if (angle_actual_no_message || angle_target_no_message)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         2000,
                         "node has not received messages on all subscribed topics yet\nno angle ACTUAL message for [ %s]\nno angle TARGET message for [ %s]",
                         angle_actual_no_message_list.str().c_str(),
                         angle_target_no_message_list.str().c_str());

    return State::Init;
  }

  return State::Control;
}

Node::State Node::handle_Control()
{
  for (auto leg: LEG_LIST)
    for (auto joint: HYDRAULIC_JOINT_LIST)
    {
      float const angle_actual_rad = _angle_actual_rad_map.at(make_key(leg, joint));
      float const angle_target_rad = _angle_target_rad_map.at(make_key(leg, joint));
      float const angle_err_rad    = angle_target_rad - angle_actual_rad;


      static float constexpr ANGLE_DIFF_EPSILON_rad = 2.5f * M_PI / 180.0f;
      if (fabs(angle_err_rad) < ANGLE_DIFF_EPSILON_rad)
      {
        _servo_pulse_width[LEG_JOINT_to_SERVO_NUM_MAP.at(make_key(leg, joint))] = SERVO_PULSE_WIDTH_NEUTRAL_us;
        _angle_error_sum_rad_map[make_key(leg, joint)] = 0.0f;
      }


      float const KP = 75.0f * (180.f / M_PI);
      float const P_OUT = (KP * angle_err_rad);

      float const KI = 10.0f * (180.f / M_PI);
      _angle_error_sum_rad_map[make_key(leg, joint)] += angle_err_rad;
      float const angle_error_sum_rad = _angle_error_sum_rad_map.at(make_key(leg, joint));
      float const dt_sec = static_cast<float>(CTRL_LOOP_RATE.count()) / 1000.0f;
      float const I_OUT = (KI * angle_error_sum_rad * dt_sec);


      /* Servo PWM set point calculation. */
      float pulse_width = SERVO_PULSE_WIDTH_NEUTRAL_us + P_OUT + I_OUT;

      /* Limit output to actual servo actuator limits. */
      pulse_width = std::max(pulse_width, static_cast<float>(SERVO_PULSE_WIDTH_MIN_us));
      pulse_width = std::min(pulse_width, static_cast<float>(SERVO_PULSE_WIDTH_MAX_us));

      _servo_pulse_width[LEG_JOINT_to_SERVO_NUM_MAP.at(make_key(leg, joint))] = static_cast<uint16_t>(pulse_width);
    }

  return State::Control;
}

std_msgs::msg::UInt16MultiArray Node::toServoPulseWidthMessage(ServoPulseWidth const & servo_pulse_width)
{
  std_msgs::msg::UInt16MultiArray msg;

  /* Configure dimensions. */
  msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  msg.layout.dim[0].size = servo_pulse_width.size();

  /* Copy in the data. */
  msg.data.clear();
  msg.data.insert(msg.data.end(), servo_pulse_width.begin(), servo_pulse_width.end());

  return msg;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
