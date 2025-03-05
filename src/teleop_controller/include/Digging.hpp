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
    void configure_belts();//initialization
    void setBeltSpeed(double speed); //USE SetDutyCycle() --> set(double speed) is [-1,1]
    void stopMotors(); // set DutyCycle() and voltage for both belt Sparkmaxes to 0.0
    bool belt_running{false};  // Keep only one declaration
    bool isRunning() const { return belt_running; }

    /**
     * @brief Limit Switch / Leadscrew
    */
    void configureLimitSwitches();// in leadscrew.cpp
    bool leadscrewGetRawLimitSwitch(RobotSide side, MechanismPosition pos);
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
       
    /*
    Left Trigger : joy_msg->axes[4] 
    Right Trigger : joy_msg->axes[5] 
    msg->buttons[5];    Right Bumper 
    msg->buttons[4];    Left Bumper 
    msg->buttons[1];    B Button
    msg->buttons[2];    X Button        
    */
   bool x_button = false;          //raise linear actuator
   bool b_button = false;          // lower linear actuator
   bool left_bumper = false;       // Extend Leadscrew
   bool right_bumper = false;      //Retract Leadscrew
   bool left_trigger = false;      // Increase Speed of Leadscrew extension
   bool right_trigger = false;     // Increase Speed of Leadscrew retraction
    
   
   /**
    * @brief Linear Actuator
    */


};