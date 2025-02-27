/**
 * @file core.hpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief Defines main Teleop : Constants / Imports / Enums & Structs
 * @version 0.1
 * @date 2025-01-13
 * @copyright Copyright (c) 2025
 */

#pragma once

/*  ROS2 Headers  */
#include <cstddef>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <cassert>
#include <functional>
#include <future>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"


/*  Sparkcan Headers    */
#include <SparkBase.hpp> 
#include <SparkFlex.hpp>
#include <SparkMax.hpp>
#include <PIDController.hpp>


/* START : XBOX Teleoperation */
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"  
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "teleop_controller/srv/set_parameter.hpp"  // generated from .srv file
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

using JoyMsg = sensor_msgs::msg::Joy::SharedPtr;
using Joy = sensor_msgs::msg::Joy;
using JoyShared = sensor_msgs::msg::Joy::SharedPtr;
using JoySubscription = rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr;
using Twist = geometry_msgs::msg::Twist;
using TwistSubscription = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
using VelocityPublisher = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using VelocitySubscriber = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
using Float64Publisher = rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr;
using Float64Subscriber = rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr;
using Float64 = std_msgs::msg::Float64;
using Float64Shared = std_msgs::msg::Float64::SharedPtr;
using BoolPublisher = rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr;
using BoolSubscriber = rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr;
using msg_Bool = std_msgs::msg::Bool;

using ParamVector = std::vector<rclcpp::Parameter>;
using ParamDescriptor = rcl_interfaces::msg::ParameterDescriptor;
using ParamEventHandler = std::shared_ptr<rclcpp::ParameterEventHandler>;
using SetParamsRes = rcl_interfaces::msg::SetParametersResult;
using SetParameterClient = rclcpp::Client<teleop_controller::srv::SetParameter>;
using SetParameterClientSharedPtr = std::shared_ptr<SetParameterClient>;
using CameraImageSub = rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr;
using CameraImagePub = rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr;


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
    double left_bumper, right_bumper,throttle_backwards, throttle_forward;
    double dpad_horizontal, dpad_vertical;
}XBOX_JOYSTICK_INPUT_t;


/**
 * @note Drivebase Scaling Factor: Precise driving via trigger can be hard. Driver can use x/y along with trigger to drive at certain speeds 
 */
typedef struct{
    double speed_lift_actuator;
    double speed_tilt_actuator;
    double speed_scaling_factor_drivebase; 
    double speed_multiplier_mining; 
    double speed_multiplier_dumping;
    double velocity_scaling = 1.0;//Was 0.75, might revert back to 0.75 later
    double wheel_speed_left, wheel_speed_right;
}ROBOT_ACTUATION_t;

typedef struct{
    bool manual_enabled;
    bool robot_disabled; 
    bool outdoor_mode;
    bool XBOX;
}ROBOTSTATE_t;

/**
 * @note wheel distance value is arbitrary as of Jan 10, 2025. Wait for final robot design to determine this value
 * @note wheel_radius might be a constant, but check FRC JAVA code & grayson's teleop usage to determine.
 */
typedef struct{
    double wheel_radius;
    const double wheel_distance = 16; 
    double voltage_limit;
}ROBOT_LIMITS_t;

const uint8_t MAX_VOLTAGE = 12;

/**
 * @todo Change CAN IDs to reflect their ACTUAL values based on the robot final configuration
 */
const uint8_t LEADSCREW_1_CAN_ID = 7;
const uint8_t LEADSCREW_2_CAN_ID = 8;
const uint8_t BELT_1_CAN_ID = 5;
const uint8_t BELT_2_CAN_ID = 6;
const double LEADSCREW_MAX_ERROR = 0.1;
const double LEADSCREW_MAX_TRAVEL = 10.0;
const uint8_t DUMPING_LEFT_CAN_ID = 13;
const uint8_t DUMPING_RIGHT_CAN_ID = 14;
const uint8_t LINEAR_LEFT_CAN_ID = 15;
const uint8_t LINEAR_RIGHT_CAN_ID = 16;

/**
 * @todo Get the ACTUAL PID constants or if they exist in some version of the FRC JAVA code, use those constants.
*/
const double BELT_VELOCITY_SCALAR = 1.0;
const double BELT_kP = 0.0; 
const double BELT_kI = 0.0;
const double BELT_kD = 0.0;
const double BELT_kIZ = 0.0;
const double BELT_kFF = 0.0;


enum class LinearActuatorState {
		Unknown, Raised, Lowered, TravelingUp, TravelingDown, Commanded
};

/**
 * @note Used to set LeadScrew Speed 
 * @todo add Leadscrew function set raw speed based on DiggingLeadscew.java in FRC JAVA code
 */
enum class RobotSide{
    LEFT,
    RIGHT,
};

enum class MechanismPosition{
    TOP,
    BOTTOM
};


typedef struct{
    std::string path;
    uint8_t cam_id;
}Camera_t;



/*END : XBOX Teleoperation*/




