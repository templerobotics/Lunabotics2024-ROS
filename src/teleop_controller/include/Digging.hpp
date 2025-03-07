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
    std::string stateToString(LeadscrewState state);
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
    rclcpp::TimerBase::SharedPtr timer_diagnostics;
    rclcpp::TimerBase::SharedPtr timer_linear_actuators;

    // LINEAR ACTUATOR
    LinearActuatorState linear_actuator_state = LinearActuatorState::Unknown;
    void checkLinearActuatorLimits();
    void commandUp();
    void commandDown();
    void commandStop();
    void linearUp();
    void linearDown();
    LinearActuatorState getLinearActuatorState();
    double getLinearActuatorLeftPosition();
    double getLinearActuatorRightPosition();
    void stopLinearActuatorMotors();                              
    void periodicLinearActuatorCheck();

   bool x_button = false;          //raise linear actuator
   bool b_button = false;          // lower linear actuator
   bool left_bumper = false;       // Extend Leadscrew
   bool right_bumper = false;      //Retract Leadscrew
   bool left_trigger = false;      // Increase Speed of Leadscrew extension
   bool right_trigger = false;     // Increase Speed of Leadscrew retraction

};