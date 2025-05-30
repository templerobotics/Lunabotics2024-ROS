#pragma once

#include "core.hpp"

class Digging : public rclcpp::Node {

public:
    Digging();
    bool isRunning();
protected:
    SparkMax m_belt_left;
    SparkMax m_belt_right;
    SparkMax m_linear_left;
    SparkMax m_linear_right;  
    SparkMax m_leadscrew_left;
    SparkMax m_leadscrew_right;
    

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    void joy_callback_digging(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void initMotors();

    /**
     * @brief Digging Belt
     * @todo PID Implementation
     */
    void setBeltSpeedForward(double speed);         
    void setBeltSpeedReverse(double speed);
    void stopDiggingBeltMotors();     
    void setBeltSpeed(double belt_speed);                        
    bool belt_running = false;  
    double belt_speed = 0.0;       
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr digging_speed_sub;             
    

    /**
     * @brief Limit Switch / Leadscrew
    */
    void configureLimitSwitches();                  
    bool isTopLimitPressed();
    bool isBottomLimitPressed();
    void setLeadscrewSpeed(double speed);
    LeadscrewState leadscrew_state = LeadscrewState::Traveling;
    LeadscrewState getLeadscrewState();               

    bool leadscrew_initialized{false};
    bool checkFault(uint16_t faults, FaultBits bit);
    void periodic();
    void checkLeadscrewLimits();
    void publishState();
    void stopLeadScrew();
    std::string stateToStringActuatorLeft(LinearActuatorStateLeft state);
    std::string stateToStringActuatorRight(LinearActuatorStateRight state);
    std::string stateToStringLeadScrew(LeadscrewState state);
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr actuator_right_state_pub, actuator_left_state_pub, mode_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr digging_right_speed_pub, digging_left_speed_pub, digging_right_temp_pub, digging_left_temp_pub, leadscrew_right_speed_pub, leadscrew_left_speed_pub, leadscrew_right_temp_pub, leadscrew_left_temp_pub, leadscrew_right_position_pub, leadscrew_left_position_pub, actuator_right_position_pub, actuator_left_position_pub, drivetrain_right_pub, drivetrain_left_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_digging_running_pub, actuator_running_right_pub, actuator_running_left_pub;
    rclcpp::TimerBase::SharedPtr timer_diagnostics;
    rclcpp::TimerBase::SharedPtr timer_linear_actuators;
    rclcpp::TimerBase::SharedPtr telemetry_timer;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leadscrew_speed_sub;  
    

    // LINEAR ACTUATOR
    LinearActuatorStateRight linear_actuator_state_right = LinearActuatorStateRight::Unknown;
    LinearActuatorStateLeft linear_actuator_state_left = LinearActuatorStateLeft::Unknown;
    void checkLinearActuatorLimits();
    void actuatorCommand(double actuator_cmd);
    void commandUpRight();
    void commandUpLeft();
    void commandDownRight();
    void commandDownLeft();
    void commandStopRight();
    void commandStopLeft();
    void commandStop();
    void linearUpRight();
    void linearUpLeft();
    void linearDownRight();
    void linearDownLeft();
    LinearActuatorStateRight getLinearActuatorStateRight();
    LinearActuatorStateLeft getLinearActuatorStateLeft();
    double getLinearActuatorLeftPosition();
    double getLinearActuatorRightPosition();
    void stopLinearActuatorMotorsRight();       
    void stopLinearActuatorMotorsLeft();                       
    void periodicLinearActuatorCheck();
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr actuator_pos_sub;
    void actuatorCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void diggingCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void leadscrewCallback(const std_msgs::msg::Float64::SharedPtr msg);
    double actuator_cmd = 0.0, digging_cmd = 0.0, leadscrew_cmd = 0.0;


    bool left_trigger = false;      // Increase Speed of Leadscrew extension
    bool right_trigger = false;     // Increase Speed of Leadscrew retraction
    bool last_a_state = false;
    bool last_y_state = false;
    bool last_b_state = false;
    bool last_x_state = false;
    bool actuators_running_right = false;
    bool actuators_running_left = false;
};