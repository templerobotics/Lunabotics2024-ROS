/**
 * @file DiggingBelt.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief Based on DiggingBelt FRC Java code
 * @version 0.1
 * @date 2025-01-12
 * @todo PID --> Wait for Grayson & look into it myself as well
 * @copyright Copyright (c) 2025
 */

#include "core.hpp"
#include "DiggingBelt.hpp"

DiggingBelt::DiggingBelt()
    : Node("digging_belt")
    , m_belt1("can0", BELT_1_CAN_ID)
    , m_belt2("can0", BELT_2_CAN_ID)
{
    velocity_pub = this->create_publisher<Float64>("belt/velocity", 10);
    temperature_pub = this->create_publisher<Float64>("belt/temperature", 10);
    
    timer_diagnostics = this->create_wall_timer(100ms, std::bind(&DiggingBelt::periodic, this));
    
    mining_belt_speed_sub = this->create_subscription<Float64>(
        "mining/belt_speed", 
        10,std::bind(&DiggingBelt::handleSpeedCommand, this, std::placeholders::_1)
    );
    
    alter_mining_belt_speed = this->create_subscription<Joy>(
        "joy", 
        10, 
        std::bind(&DiggingBelt::OperatorDigging, this, std::placeholders::_1)
    );

    configureBelts();
    configurePID();
    
    RCLCPP_INFO(this->get_logger(), "Digging Belt initialized");
}

void DiggingBelt::configureBelts() {
    // Config follower
    m_belt2.SetFollowerID(BELT_1_CAN_ID);
    m_belt2.SetFollowerConfig(1);  // follower mode
    m_belt2.SetInverted(true);  // invert follower

    m_belt1.SetMotorType(MotorType::kBrushless);
    m_belt2.SetMotorType(MotorType::kBrushless);
    
    m_belt1.BurnFlash();
    m_belt2.BurnFlash();
}

void DiggingBelt::configurePID() {
    p_belt = std::make_unique<PIDController>(m_belt1);
    
    // Config PID settings
    p_belt->SetP(0, BELT_kP);
    p_belt->SetI(0, BELT_kI);
    p_belt->SetD(0, BELT_kD);
    p_belt->SetIZone(0, BELT_kIZ);
    p_belt->SetF(0, BELT_kFF);
}

void DiggingBelt::periodic() {
    reportSensors();
}

void DiggingBelt::OperatorDigging(const JoyShared msg) {
    xbox_input.left_bumper = msg->buttons[4];
    xbox_input.right_bumper = msg->buttons[5];
    xbox_input.y_button = msg->buttons[3];
    xbox_input.a_button = msg->buttons[0];

    if(xbox_input.right_bumper && xbox_input.y_button) {
        current_speed = std::min(1.0, current_speed + SPEED_INCREMENT);
    } else if(xbox_input.left_bumper && xbox_input.y_button) {
        current_speed = std::max(-1.0, current_speed - SPEED_INCREMENT);
    } else if(xbox_input.a_button && xbox_input.right_bumper) {
        current_speed = std::min(1.0, current_speed + SPEED_INCREMENT);
    } else if(xbox_input.a_button && xbox_input.left_bumper) {
        current_speed = std::max(-1.0, current_speed - SPEED_INCREMENT);
    }
    setBeltSpeed(current_speed);
}

void DiggingBelt::handleSpeedCommand(const Float64Shared msg) {
    setBeltSpeed(msg->data);  
}

void DiggingBelt::setBeltSpeed(double speed) {
    m_belt1.SetDutyCycle(std::clamp(speed, -1.0, 1.0));
    belt_running = true;
    RCLCPP_INFO(this->get_logger(), "Belt speed set to: %f", speed);
}

void DiggingBelt::runBelt(bool reverse) {
    double target_speed = reverse ? -belt_speed : belt_speed;
    p_belt->SetReference(target_speed, CtrlType::kVelocity);
    belt_running = true;
    RCLCPP_INFO(this->get_logger(), "Belt running at speed: %f", target_speed);
}

void DiggingBelt::stopBelt() {
    m_belt1.SetDutyCycle(0.0);
    belt_running = false;
    RCLCPP_INFO(this->get_logger(), "Belt stopped");
}

void DiggingBelt::reportSensors() {
    auto velocity_msg = std_msgs::msg::Float64();
    velocity_msg.data = m_belt1.GetVelocity();
    velocity_pub->publish(velocity_msg);

    auto temp_msg = std_msgs::msg::Float64();
    temp_msg.data = m_belt1.GetTemperature();
    temperature_pub->publish(temp_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Belt Velocity: %f, Temperature: %f", 
                 velocity_msg.data, temp_msg.data);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiggingBelt>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}