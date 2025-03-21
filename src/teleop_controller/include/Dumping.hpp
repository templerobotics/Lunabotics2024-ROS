#pragma once
#include "core.hpp"

class Dumping : public rclcpp::Node {

protected:
    Dumping();
    SparkMax m_dumping_left;
    SparkMax m_dumping_right;
    void joy_callback_dumping(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void initMotors();  
    void cmd_open_dumplatch(double cmd_open_dumplatch);
    void cmd_close_dumplatch(double cmd_close_dumplatch);
    
    
};