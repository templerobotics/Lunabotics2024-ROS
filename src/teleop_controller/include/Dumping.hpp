#pragma once
#include "core.hpp"

class Dumping : public rclcpp::Node {
public:
    Dumping();

protected:
    SparkMax m_dumping_left;
    SparkMax m_dumping_right;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dumping_right_speed_pub, dumping_left_speed_pub, dumping_right_temp_pub, dumping_left_temp_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_digging_running;
    rclcpp::TimerBase::SharedPtr telemetry_timer;
    void joy_callback_dumping(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void initMotors();  
    void cmd_open_dumplatch(double cmd_open_dumplatch);
    void cmd_close_dumplatch(double cmd_close_dumplatch);
    void move_belt_forward();
    void move_belt_reverse();
    void stop_dumping_belt();

    bool last_dpad_right = false;
    bool last_dpad_left = false;
    bool dumping_belt_running = false;    
};