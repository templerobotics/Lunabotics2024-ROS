#pragma once

#include "core.hpp"

class Digging : public rclcpp::Node {
public:
    Digging();

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
    bool belt_running{false};                      
    bool isRunning() const { return belt_running; }

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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr actuator_right_state_pub, actuator_left_state_pub, leadscrew_right_state_pub, leadscrew_left_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr digging_right_speed_pub, digging_left_speed_pub, digging_right_temp_pub, digging_left_temp_pub, leadscrew_right_speed_pub, leadscrew_left_speed_pub, leadscrew_right_temp_pub, leadscrew_left_temp_pub, actuator_right_position_pub, actuator_left_position_pub;
    rclcpp::TimerBase::SharedPtr timer_diagnostics;
    rclcpp::TimerBase::SharedPtr timer_linear_actuators;
    rclcpp::TimerBase::SharedPtr telemetry_timer;

    // LINEAR ACTUATOR
    LinearActuatorStateRight linear_actuator_state_right = LinearActuatorStateRight::Unknown;
    LinearActuatorStateLeft linear_actuator_state_left = LinearActuatorStateLeft::Unknown;
    void checkLinearActuatorLimits();
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

    bool x_button = false;          //raise linear actuator
    bool b_button = false;          // lower linear actuator
    bool left_bumper = false;       // Extend Leadscrew
    bool right_bumper = false;      //Retract Leadscrew
    bool left_trigger = false;      // Increase Speed of Leadscrew extension
    bool right_trigger = false;     // Increase Speed of Leadscrew retraction
    bool last_a_state = false;
    bool last_y_state = false;
    bool last_b_state = false;
    bool last_x_state = false;
    bool actuators_running_right = false;
    bool actuators_running_left = false;
};