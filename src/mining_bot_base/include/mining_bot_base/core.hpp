#pragma once

/*  ROS2 Headers  */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen.h>
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
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/utilities.hpp"
#include "rclcpp/logging.hpp"
#include <cassert>


/*  Sparkcan Headers    */
#include <SparkBase.hpp> 
#include <SparkFlex.hpp>
#include <SparkMax.hpp>


/* Custom Headers */



/* 

TODO:
Seperate into (1) Drivebase (2) Digging (3) Dumping. 
Drivebase initializes the motors
Digging Initializes the linear actuators --> motors should be already running
Dumping utilizes initialized motors & linear actuators to dump on to our conveyer belt

*/

/* START : XBOX Teleoperation */
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

using JoyMsg = sensor_msgs::msg::Joy::SharedPtr;
using Joy = sensor_msgs::msg::Joy;
using JoySubscription = rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr;

using TwistMsg = geometry_msgs::msg::Twist::SharedPtr;
using Twist = geometry_msgs::msg::Twist;
using TwistSubscription = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
using BoolPublisher = rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr;
using namespace std::chrono_literals;

/*
    Should I default the bools to their
    ideal initial values?
*/

typedef struct{
    bool home, share, menu;
    bool a, b, x, y;
    bool left_bumper, right_bumper;
}XBOX_BUTTONS_t;

typedef struct{
    double left_x_axis;
    double left_y_axis;
    double right_y_axis;
    double right_x_axis;
    double left_trigger, right_trigger;
    double dpad_up, dpad_down, dpad_left, dpad_right;
}XBOX_JOYSTICK_INPUT_t;

typedef struct{
    bool manual_enabled;
    bool robot_disabled; 
    bool outdoor_mode;
    bool XBOX;
    bool PS4;
    double speed_multiplier = 0.6;
    double left_speed = 0.0;
    double right_speed = 0.0;
    double lift_actuator_speed = 0.0;
    double tilt_actuator_speed = 0.0;
}ROBOTSTATE_t;

enum class Motor{
    leftFront,
    leftRear,
    rightFront,
    rightRear
};



/*END : XBOX Teleoperation*/




