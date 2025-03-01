/**
 * @file drivebase_control.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief ROS2 node for managing the robot's drivebase and coordinating subsystem commands
 * @version 0.2
 * @date 2025-01-11
 */

#include "core.hpp"

class DrivebaseControl : public rclcpp::Node {
public:
    DrivebaseControl() 
        : Node("drivebase_control"), 
        left_motor("can0", 2),
        right_motor("can0", 3),
        manual_enabled(true), 
        robot_disabled(false)
    {
        joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&DrivebaseControl::joy_callback, this, std::placeholders::_1));
        
        try {
            RCLCPP_INFO(get_logger(), "Configuring Drivebase Motors");
            
            left_motor.SetIdleMode(IdleMode::kCoast);
            left_motor.SetMotorType(MotorType::kBrushless);
            left_motor.SetDutyCycle(0.0);
            left_motor.BurnFlash();
            
            right_motor.SetIdleMode(IdleMode::kCoast);
            right_motor.SetMotorType(MotorType::kBrushless);
            right_motor.SetInverted(true);  // Invert right motor
            right_motor.SetDutyCycle(0.0);
            right_motor.BurnFlash();
            
            RCLCPP_INFO(get_logger(), "Drivebase Motors configured successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to configure Drivebase motors: %s", e.what());
        }
        
        heartbeat_timer = create_wall_timer(std::chrono::milliseconds(500),std::bind(&DrivebaseControl::heartbeat, this));    
        RCLCPP_INFO(get_logger(), "DrivebaseControl initialized!");
    }

private:
    SparkMax left_motor;
    SparkMax right_motor;
    
    // State
    bool manual_enabled;
    bool robot_disabled;
    double speed_multiplier = 0.6;
    double left_speed = 0.0;
    double right_speed = 0.0;
    
    // Inputs
    double left_joystick_x = 0.0;
    double right_trigger = 0.0;
    double left_trigger = 0.0;
    bool x_button = false;
    bool y_button = false;
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    
    /**
     * @brief Process joystick input and control motors
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        if (robot_disabled) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Robot is disabled!");
            return;
        }
        
        left_joystick_x = joy_msg->axes[0];
        right_trigger = joy_msg->axes[5];
        left_trigger = joy_msg->axes[2];
        x_button = joy_msg->buttons[2];
        y_button = joy_msg->buttons[3];
        
        // Transform triggers from 1.0 to -1.0 range to 0.0 to 1.0 range
        right_trigger = (1.0 - right_trigger) / 2.0;
        left_trigger = (1.0 - left_trigger) / 2.0;
        
        speed_multiplier = (x_button && y_button) ? 0.3 : (x_button ? 0.1 : 0.6);
        //Differential Drive Equations, but Teleop
        if (right_trigger > 0.05) {  // Forward
            left_speed = right_trigger - left_joystick_x;
            right_speed = right_trigger + left_joystick_x;
            RCLCPP_INFO(get_logger(), "Forward: RT=%f, LX=%f -> L=%f, R=%f", right_trigger, left_joystick_x, left_speed, right_speed);
        
        } else if (left_trigger > 0.05) {  // Backward
            left_speed = -(left_trigger - left_joystick_x);
            right_speed = -(left_trigger + left_joystick_x);
            RCLCPP_INFO(get_logger(), "Backward: LT=%f, LX=%f -> L=%f, R=%f", left_trigger, left_joystick_x, left_speed, right_speed);
        
        } else if (std::abs(left_joystick_x) > 0.05) {  // Turn in place
            left_speed = -left_joystick_x;
            right_speed = left_joystick_x;
            RCLCPP_INFO(get_logger(), "Turn: LX=%f -> L=%f, R=%f", left_joystick_x, left_speed, right_speed);
        } else {
            left_speed = 0.0;
            right_speed = 0.0;
        }
        
        left_speed = std::clamp(left_speed * speed_multiplier, -1.0, 1.0);
        right_speed = std::clamp(right_speed * speed_multiplier, -1.0, 1.0);
        
        try {
            left_motor.SetDutyCycle(left_speed);
            right_motor.SetDutyCycle(right_speed);
            double left_voltage = left_speed * MAX_VOLTAGE;
            double right_voltage = right_speed * MAX_VOLTAGE;
            left_motor.SetVoltage(left_voltage);
            right_motor.SetVoltage(right_voltage);
            RCLCPP_DEBUG(get_logger(), "Motors set: L=%f V, R=%f V", left_voltage, right_voltage);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to set motor speeds: %s", e.what());
        }
    }
    
    /**
     * @brief Send heartbeat to keep NEO Motors running
     */
    void heartbeat() {
        try {
            SparkMax::Heartbeat();
            RCLCPP_DEBUG(get_logger(), "Heartbeat sent successfully!");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to send heartbeat!: %s", e.what());
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrivebaseControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}