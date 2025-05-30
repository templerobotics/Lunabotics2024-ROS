#pragma once

#include "core.hpp"

class DrivebaseControl : public rclcpp::Node {
public:
    DrivebaseControl();
protected:
    SparkMax left_front, left_rear, right_front, right_rear;
private:
    
    void joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void initMotors();
    void calculate_motor_speeds(double motor_cmd_left, double motor_cmd_right);
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_drive_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_front_speed_pub, right_rear_speed_pub,
        left_front_speed_pub, left_rear_speed_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_digging_running_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drivetrain_right_speed, drivetrain_left_speed;
    rclcpp::TimerBase::SharedPtr telemetry_timer;

    double linear_x = 0.0, angular_z = 0.0, motor_cmd_left = 0.0, motor_cmd_right = 0.0;
    bool controller_teleop_enabled, autonomy_enabled;
    bool is_digging_running;
};
