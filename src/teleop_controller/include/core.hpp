#pragma once

/*  ROS2 Headers  */
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cstddef>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/utilities.hpp"
#include "rclcpp/logging.hpp"
#include <cassert>
#include <functional>
#include "std_msgs/msg/string.hpp"
#include <future>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <fstream>

/*  Sparkcan Headers    */
#include <SparkBase.hpp> 
#include <SparkFlex.hpp>
#include <SparkMax.hpp>


/* START : XBOX Teleoperation */
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"  
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "teleop_controller/srv/set_parameter.hpp"  // generated from .srv file
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

using JoyMsg = sensor_msgs::msg::Joy::SharedPtr;
using Joy = sensor_msgs::msg::Joy;
using JoySubscription = rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr;
using Twist = geometry_msgs::msg::Twist;
using TwistSubscription = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
using VelocityPublisher = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
using VelocitySubscriber = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
using Float64Publisher = rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
using Float64Subscriber = rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr

using Float64 = std_msgs::msg::Float64
using BoolPublisher = rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr;
using BoolSubscriber = rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr;
using msg_Bool = std_msgs::msg::Bool;

using ParamVector = std::vector<rclcpp::Parameter>;
using ParamDescriptor = rcl_interfaces::msg::ParameterDescriptor;
using ParamEventHandler = std::shared_ptr<rclcpp::ParameterEventHandler>;
using SetParamsRes = rcl_interfaces::msg::SetParametersResult;
using SetParameterClient = rclcpp::Client<teleop_controller::srv::SetParameter>;
using SetParameterClientSharedPtr = std::shared_ptr<SetParameterClient>;



typedef struct{
    bool emergency_stop_button;
    bool manual_mode_button;
    bool autonomous_mode_button;
    bool a_button;
    bool b_button;
    bool x_button;
    bool y_button;
    double joystick_turn_input;
    double joystick_forward_input;
    //double secondary_vertical_input;
    double left_bumper, right_bumper,throttle_backwards, throttle_forward;
    double dpad_horizontal, dpad_vertical;
}XBOX_JOYSTICK_INPUT_t;


/**
 * @note Drivebase Scaling Factor: Actuating via trigger can be wonky, so user can use x/y along with trigger to drive at certain speeds 
 * 
 */
typedef struct{
    double speed_lift_actuator;
    double speed_tilt_actuator;
    double speed_scaling_factor_drivebase; //
    double speed_multiplier_mining; 
    double speed_multiplier_dumping;
    double velocity_scaling = 0.75;
    double wheel_speed_left, wheel_speed_right;
}ROBOT_ACTUATION_t;

typedef struct{
    bool manual_enabled;
    bool robot_disabled; 
    bool outdoor_mode;
    bool XBOX;
}ROBOTSTATE_t;

/**
 * @note wheel distance value is arbitrary as of Jan 10, 2025. Waiting for final robot design
 * 
 */
typedef struct{
    double wheel_radius;
    const double wheel_distance = 16; 
    double voltage_limit;
}ROBOT_LIMITS_t;

/* Make Leadscrews/Limit switches C++ classes or this is enough? */
enum class LeadScrewMiningState {
    Extended,
    Retracted,
    Traveling,
    FullyExtended
};


enum class RobotSide{
    LEFT,
    RIGHT,
};

enum class MechanismPosition{
    TOP,
    BOTTOM
};

/*END : XBOX Teleoperation*/




