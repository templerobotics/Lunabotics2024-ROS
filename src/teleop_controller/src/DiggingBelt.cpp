/**
 * @file DiggingBelt.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief Based on DiggingBelt FRC Java code
 * @version 0.1
 * @date 2025-01-12
 * @todo PID --> Wait for Grayson & look into it myself as well
 * @todo What to do with functions : (1)runBelt() & (2) stopBelt()
 * @copyright Copyright (c) 2025
 */
#include "core.hpp"
#include "DiggingBelt.hpp"

DiggingBelt::DiggingBelt()
    : Node("digging_belt")
    , m_belt1("can0", BELT_1_CAN_ID)
    , m_belt2("can0", BELT_2_CAN_ID)
    , m_linear_left("can0", LINEAR_LEFT_CAN_ID)
    , m_linear_right("can0", LINEAR_RIGHT_CAN_ID)
{
    velocity_pub = this->create_publisher<Float64>("belt/velocity", 10);
    temperature_pub = this->create_publisher<Float64>("belt/temperature", 10);
    timer_diagnostics = this->create_wall_timer(100ms, std::bind(&DiggingBelt::periodic, this));
    
    /**
     * @brief sub to published topic from Drivebase Control
     */
    mining_belt_speed_sub = this->create_subscription<Float64>("mining/belt_speed", 10,
        std::bind(&DiggingBelt::handleDiggingSpeed, this, std::placeholders::_1));
    
    configureBelts();
    //configurePID();
    
    RCLCPP_INFO(this->get_logger(), "Digging Belt initialized");
}

void DiggingBelt::configureBelts() {
    // Config follower
    m_belt2.SetFollowerID(BELT_1_CAN_ID);
    m_belt2.SetFollowerConfig(1); // follower mode
    m_belt2.SetInverted(true); // invert follower
    m_belt1.SetMotorType(MotorType::kBrushless);
    m_belt2.SetMotorType(MotorType::kBrushless);
    m_belt1.BurnFlash();
    m_belt2.BurnFlash();
}

/*
void DiggingBelt::configurePID() {
    p_belt = std::make_unique<PIDController>(m_belt1);
    // Config PID settings
    p_belt->SetP(0, BELT_kP);
    p_belt->SetI(0, BELT_kI);
    p_belt->SetD(0, BELT_kD);
    p_belt->SetIZone(0, BELT_kIZ);
    p_belt->SetF(0, BELT_kFF);
}
*/

void DiggingBelt::periodic() {
    reportSensors();
}

void DiggingBelt::handleDiggingSpeed(const Float64Shared msg) {
    setBeltSpeed(msg->data);
}

void DiggingBelt::setBeltSpeed(double speed) {
    m_belt1.SetDutyCycle(std::clamp(speed, -1.0, 1.0));
    belt_running = true;
    RCLCPP_INFO(this->get_logger(), "Belt speed set to: %f", speed);
}

/*
void DiggingBelt::runBelt(bool reverse) {
    double target_speed = reverse ? -belt_speed : belt_speed;
    p_belt->SetReference(target_speed, CtrlType::kVelocity);
    belt_running = true;
    RCLCPP_INFO(this->get_logger(), "Belt running at speed: %f", target_speed);
}
*/

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