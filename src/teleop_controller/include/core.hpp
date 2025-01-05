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

using namespace std::chrono_literals;
using std::placeholders::_1;

using JoyMsg = sensor_msgs::msg::Joy::SharedPtr;
using Joy = sensor_msgs::msg::Joy;
using JoySubscription = rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr;
using Twist = geometry_msgs::msg::Twist;
using TwistSubscription = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;

using BoolPublisher = rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr;
using BoolSubscriber = rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr;
using msg_Bool = std_msgs::msg::Bool;

using ParamVector = std::vector<rclcpp::Parameter>;
using ParamDescriptor = rcl_interfaces::msg::ParameterDescriptor;
using ParamEventHandler = std::shared_ptr<rclcpp::ParameterEventHandler>;
using SetParamsRes = rcl_interfaces::msg::SetParametersResult;
using SetParameterClient = rclcpp::Client<teleop_controller::srv::SetParameter>;
using SetParameterClientSharedPtr = std::shared_ptr<SetParameterClient>;

// Drivebase diff drive Motor Groups
class MotorControllerGroup {
private:
    SparkMax& motor1;
    SparkMax& motor2;

public:
    MotorControllerGroup(SparkMax& m1, SparkMax& m2) : motor1(m1), motor2(m2) {}
    
    void setSpeed(double speed) {
        motor1.SetDutyCycle(speed);
        motor2.SetDutyCycle(speed);
    }
    
    void setInverted(bool inverted) {
        motor1.SetInverted(inverted);
        motor2.SetInverted(inverted);
    }

    void stop_motor_groups() {
        motor1.SetDutyCycle(0.0);
        motor1.SetVoltage(0);
        motor2.SetDutyCycle(0.0);
        motor2.SetVoltage(0);
    }
};


//COMMENT & UNCOMMENT the define below --> CMAKE LISTS IS IDEAL, BUT THIS FOR NOW

#define HARDWARE_ENABLED

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
    double secondary_vertical_input;
    double left_bumper, right_bumper,throttle_backwards, throttle_forward;
    double dpad_horizontal, dpad_vertical;
}XBOX_JOYSTICK_INPUT_t;

typedef struct{
    double speed_lift_actuator;
    double speed_tilt_actuator;
    double velocity_scaling;
    double wheel_speed_left, wheel_speed_right;
}ROBOT_ACTUATION_t;

typedef struct{
    bool manual_enabled;
    bool robot_disabled; 
    bool outdoor_mode;
    bool XBOX;
    bool PS4;
}ROBOTSTATE_t;

typedef struct{
    double wheel_radius;
    const double wheel_distance = 0.5;
    double max_velocity;
    double min_velocity;
    double max_angular_velocity;
    double voltage_limit;
}ROBOT_LIMITS_t;

//Not sure if I'll use this. For now it is unused
enum class RobotSide{
    LEFT,
    RIGHT,
};



/*END : XBOX Teleoperation*/




