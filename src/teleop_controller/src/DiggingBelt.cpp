/**
 * @file DiggingBelt.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief Based on DiggingBelt FRC Java code
 * @version 0.1
 * @date 2025-01-12
 * @todo PID --> Wait for Grayson & look into it myself as well
 * @copyright Copyright (c) 2025
 * 
 */


#include "core.hpp"


class DiggingBelt : public rclcpp::Node{

public: 
    DiggingBelt()
    : Node("digging_belt")
    , m_belt1("can0", BELT_1_CAN_ID)
    , m_belt2("can0", BELT_2_CAN_ID)
{
    configureBelts();
    configurePID();
    
    velocity_pub = create_publisher<Float64>("belt/velocity", 10);
    temperature_pub = create_publisher<Float64>("belt/temperature", 10);
    timer_diagnostics = create_wall_timer(100ms, std::bind(&DiggingBelt::periodic, this));
    mining_belt_speed_sub = create_subscription<std_msgs::msg::Float64>("mining/belt_speed", 10,std::bind(&DiggingBelt::handleSpeedCommand, this, std::placeholders::_1));
    alter_mining_belt_speed = create_subscription<Joy>("joy", 10, std::bind(&DiggingBelt::OperatorDigging, this, std::placeholders::_1))
    RCLCPP_INFO(get_logger(), "Digging Belt initialized");
}

private:
    rclcpp::TimerBase::SharedPtr timer_diagnostics;
    VelocityPublisher velocity_pub;
    Float64Publisher temperature_pub;
    Float64Subscriber mining_belt_speed_sub;
    XBOX_JOYSTICK_INPUT_t xbox_input;
    double current_speed = 0.0; 
    const double SPEED_INCREMENT = 0.1;
    JoySubscription alter_mining_belt_speed;

    /**
     * @brief Increase/Decrease belt speed based on right/left bumpers while user is already digging w/ Y or A
     * @param msg Incoming Joystick msg data stream 
     */

    void OperatorDigging(const JoyMsg& msg){
        xbox_input.left_bumper = msg->buttons[4];
        xbox_input.right_bumper = msg->buttons[5];

        if(xbox_input.right_bumper && xbox_input.y_button){
            current_speed = std::min(1.0, current_speed + SPEED_INCREMENT);

        }else if(xbox_input.left_bumper && xbox_input.y_button){
            current_speed = std::max(-1.0, current_speed - SPEED_INCREMENT);

        }else if(xbox_input.a_button && xbox_input.right_bumper){
            current_speed = std::min(1.0, current_speed + SPEED_INCREMENT);

        }else if(xbox_input.a_button && xbox_input.left_bumper){
            current_speed = std::max(-1.0, current_speed - SPEED_INCREMENT);

        }

        m_belt_1.SetBeltSpeed(current_speed);
        m_belt_2.SetBeltSpeed(current_speed);

    }



    /**
     * @brief Sets Mining Belt Speed
     * @param msg 
     */
    void handleSpeedCommand(const std_msgs::msg::Float64::SharedPtr msg) {
        setBeltSpeed(msg->data);  
    }

    /**
     * @brief Mining Belt Config
     * 
     */
    void DiggingBelt::configureBelts() {
        // Configure follower
        m_belt2.SetFollowerID(BELT_1_CAN_ID);
        m_belt2.SetFollowerConfig(1);  // Basic follower mode
        m_belt2.SetInverted(true);  // Invert follower

        m_belt1.SetMotorType(MotorType::kBrushless);
        m_belt2.SetMotorType(MotorType::kBrushless);
        
        m_belt1.BurnFlash();
        m_belt2.BurnFlash();
    }

    /**
     * @brief Set PID values
     * @note Grayson says PID coming soon. Hopefully asap
     */
    void DiggingBelt::configurePID() {
        p_belt = std::make_unique<PIDController>(m_belt1);
        
        // Configure PID settings
        p_belt->SetP(0, BELT_kP);
        p_belt->SetI(0, BELT_kI);
        p_belt->SetD(0, BELT_kD);
        p_belt->SetIZone(0, BELT_kIZ);
        p_belt->SetF(0, BELT_kFF);
    }

    /**
     * @brief Periodically prints sensor data
     */
    void DiggingBelt::periodic() {
        reportSensors();
    }

    void DiggingBelt::setBeltSpeed(double speed) {
        m_belt1.SetDutyCycle( std::clamp(speed, -1, 1) );
        belt_running = true;
        RCLCPP_INFO(get_logger(), "Belt speed set to: %f", speed);
    }

    void DiggingBelt::runBelt(bool reverse) {
        double target_speed = reverse ? -belt_speed : belt_speed;
        p_belt->SetReference(target_speed, CtrlType::kVelocity);
        belt_running = true;
        RCLCPP_INFO(get_logger(), "Belt running at speed: %f", target_speed);
    }

    /**
     * @brief Stops belt 
     */
    void DiggingBelt::stopBelt() {
        m_belt1.SetDutyCycle(0.0);
        belt_running = false;
        RCLCPP_INFO(get_logger(), "Belt stopped");
    }

    /**
     * @brief Logging function 
    */
    void DiggingBelt::reportSensors() {
        auto velocity_msg = std_msgs::msg::Float64();
        velocity_msg.data = m_belt1.GetVelocity();
        velocity_publisher->publish(velocity_msg);

        auto temp_msg = std_msgs::msg::Float64();
        temp_msg.data = m_belt1.GetTemperature();
        temperature_publisher->publish(temp_msg);
        RCLCPP_DEBUG(get_logger(), "Belt Velocity: %f, Temperature: %f", velocity_msg.data, temp_msg.data);
    }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiggingBelt>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
