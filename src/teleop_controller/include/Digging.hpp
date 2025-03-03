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
    void init_motors();

    /**
     * @brief Digging Belt
     * @todo PID Implementation
     */
    void configure_belts();//initialization
    void setBeltSpeed(); //USE SetDutyCycle() --> set(double speed) is [-1,1]
    void stop_motor(); // set DutyCycle() and voltage for both belt Sparkmaxes to 0.0
    bool belt_running{false};  // Keep only one declaration
    bool isRunning() const { return belt_running; }

    /**
     * @brief Limit Switch / Leadscrew
    */
    void configure_limit_switches();// in leadscrew.cpp
    bool leadscrewGetRawLimitSwitch(RobotSide side, MechanismPosition pos)
    bool isTopLimitPressed();
    bool isBottomLimitPressed();
    void setLeadscrewSpeed(double speed);
    LeadscrewState leadscrew_state = LeadscrewState::Traveling;
    LeadscrewState getState() const;//return leadscrew_state

    bool leadscrew_initialized{false};
    bool checkFault(uint16_t faults, FaultBits bit);
    void periodic();
    void checkLimits();
    void publishState();
    std::string stateToString(LeadscrewState state);
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;

    /**
     * @brief Linear Actuator
    */

    //TODO : FIGURE THESE OUT
    double current_speed{0.0};
    double belt_speed{0.5};
    const double SPEED_INCREMENT{0.1};


};