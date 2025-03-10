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
#include "teleop_controller/srv/set_parameter.hpp"  // srv file
#include "teleop_controller/srv/switch_mode.hpp"    // srv file
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

using JoyMsg = sensor_msgs::msg::Joy::SharedPtr;
using Joy = sensor_msgs::msg::Joy;
using JoyShared = sensor_msgs::msg::Joy::SharedPtr;
using JoySubscription = rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr;
using Twist = geometry_msgs::msg::Twist;
using TwistSubscription = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
using TwistPublisher = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;

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


const uint8_t WHEEL_BASE = 30;                  //inches
const uint8_t WHEEL_RADIUS = 8;
const uint8_t MIN_THROTTLE_DEADZONE = 0.05;
const uint16_t SPARKMAX_RPM = 292;               //change to reflect actual value. Use GetVelocity()
const uint8_t MAX_VOLTAGE = 12;
const uint8_t SPARKMAX_RPM_AVERAGE = 63;
const uint8_t SPARKMAX_MAX_DUTY_CYCLE = 1;

/**
 * @todo Change CAN IDs to reflect their ACTUAL values based on the robot final configuration
 */

const uint8_t MOTOR_FRONT_LEFT_CAN_ID = 1;
const uint8_t MOTOR_REAR_LEFT_CAN_ID = 2;
const uint8_t MOTOR_FRONT_RIGHT_CAN_ID = 3;
const uint8_t MOTOR_REAR_RIGHT_CAN_ID = 4;


const uint8_t LEADSCREW_1_CAN_ID = 7;
const uint8_t LEADSCREW_2_CAN_ID = 8;
const uint8_t BELT_1_CAN_ID = 5;
const uint8_t BELT_2_CAN_ID = 6;
const uint8_t DUMPING_LEFT_CAN_ID = 13;
const uint8_t DUMPING_RIGHT_CAN_ID = 14;
const uint8_t LINEAR_LEFT_CAN_ID = 15;
const uint8_t LINEAR_RIGHT_CAN_ID = 16;


bool LEADSCREW_INVERT = true;
double LEADSCREW_MAX_ACCEL = 11000 * 0.5; // RPM/s
double LEADSCREW_MAX_VEL = 11000 * 0.5; // RPM
double LEADSCREW_MIN_VEL = 10; // RPM
double LEADSCREW_MAX_ERROR = 1; // Rotations
double LEADSCREW_MAX_TRAVEL = 13000; // Native Units
double LEADSCREW_MAX_SPEED = 1; // For leadscrews without PIDs
double LEADSCREW_EXTENDED_POS = 13000; //TODO: Put a real value here
double LEADSCREW_kP = 0.0000000015;
double LEADSCREW_kI = 0.000002;
double LEADSCREW_kD = 0.000005;
double LEADSCREW_kIZ = 20;
double LEADSCREW_kFF = 0.000080;
double LEADSCREW_MAX_OUTPUT = .9;
double LEADSCREW_MIN_OUTPUT = -.9;
int LEADSCREW_CURRENT_LIMIT_STALL = 20;
int LEADSCREW_CURRENT_LIMIT_FREE = 10;
int LEADSCREW_SECNDARY_CURRENT_LIMIT = 22;
//IdleMode LEADSCREW_IDLE_MODE = IdleMode.kBrake;


bool BELT_INVERT = true;
double BELT_MAX_ACCEL = 5760; // RPM/s
double BELT_MAX_VEL = 11000; // RPM
double BELT_MIN_VEL = 10; // RPM
double BELT_MAX_ERROR = 50; // Rotations
double BELT_VELOCITY_SCALAR = 1; // 6.65e-5
double BELT_kP = 0.000026;
double BELT_kI = 0.000001;
double BELT_kD = 0.0001;
double BELT_kIZ = 20;
double BELT_kFF = 0.000085;
double BELT_MAX_OUTPUT = 0.9;
double BELT_MIN_OUTPUT = -0.9;
int BELT_CURRENT_LIMIT_STALL = 20;
int BELT_CURRENT_LIMIT_FREE = 10;
int BELT_SECNDARY_CURRENT_LIMIT = 22;


// Linear Actuator
bool LINEAR_INVERT = true;
double LINEAR_DEADBAND = .01;
double LINEAR_MIN_TRAVEL = 0; //1.438; // 0.2876; // 1.438
double LINEAR_MAX_TRAVEL = 0.5; //1;//3.3; //3.3; //0.55
double LINEAR_2_ADJUSTMENT = -0.02;
double DIGGING_LINEAR_kP = 0.0;//0.1;
double DIGGING_LINEAR_kI = 0.0;//0.000002;
double DIGGING_LINEAR_kD = 0.0;//0.000005;
double DIGGING_LINEAR_kIZ = 20;
double DIGGING_LINEAR_kFF = 0.000080;

enum class LinearActuatorState {
    Unknown, Raised, Lowered, TravelingUp, TravelingDown, Commanded
};


enum class LeadscrewState {
    Extended,
    Retracted,
    Traveling,
    FullExtended,
    GivenCommand
};


enum class RobotSide{
LEFT,
RIGHT,
};

enum class MechanismPosition{
TOP,
BOTTOM
};


/**
 * @brief Fault IDs based on REV documentation. Use GetFaults() method from Sparkcan
*/
enum class FaultBits : uint16_t {
    kHardLimitFwd = 0,    // Forward limit switch
    kHardLimitRev = 1,    // Reverse limit switch
    kSoftLimitFwd = 2,    // Forward soft limit
    kSoftLimitRev = 3,    // Reverse soft limit
    kMotorFault = 4,      // Motor fault
    kSensorFault = 5,     // Sensor fault
    kStall = 6,           // Stall detected
    kEEPROMCRC = 7,       // EEPROM CRC error
    kCANTX = 8,           // CAN transmit error
    kCANRX = 9,           // CAN receive error
    kHasReset = 10,       // Has reset
    kDRVFault = 11,       // DRV fault
    kOtherFault = 12,     // Other fault
    kSoftLimitClamp = 13, // Soft limit clamp
    kBrownout = 14        // Brownout
};



typedef struct{
    bool y_button;
}XBOX_JOYSTICK_INPUT_t;


typedef struct{
    bool XBOX;
}ROBOT_ACTUATION_t;


typedef struct{
    std::string path;
    uint8_t cam_id;
}Camera_t;

//BMS




/*END : XBOX Teleoperation*/




