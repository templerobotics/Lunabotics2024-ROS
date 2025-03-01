/**
 * @file drivebase_control.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief ROS2 node for managing the robot's drivebase and coordinating subsystem commands
 * @todo Simplify Info print commands to only say Forward/Backwards/Turning
 * @todo For drivebase, don't even need bool x/y buttons. Can use them for another Subsystem. Something to consider
 * @todo Put in some work to get SparkMax temps in celsius to print/other profiling data. SparkMax has method/built-in temp sensor
 * @version 0.3
 * @date 2025-03-01
 */

 #include "core.hpp"

 class DrivebaseControl : public rclcpp::Node {
 public:
     DrivebaseControl() 
         : Node("drivebase_control"), 
         left_front("can0", 1),
         left_rear("can0", 2),
         right_front("can0", 3),
         right_rear("can0", 4),
         manual_enabled(true), 
         robot_disabled(false)
     {
        joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&DrivebaseControl::joy_callback, this, std::placeholders::_1));
        try {
            RCLCPP_INFO(get_logger(), "Configuring Drivebase Motors");
            left_front.SetIdleMode(IdleMode::kCoast);
            left_front.SetMotorType(MotorType::kBrushless);
            left_front.SetDutyCycle(0.0);
            left_front.BurnFlash();
            
            left_rear.SetIdleMode(IdleMode::kCoast);
            left_rear.SetMotorType(MotorType::kBrushless);
            left_rear.SetDutyCycle(0.0);
            left_rear.BurnFlash();
            
            right_front.SetIdleMode(IdleMode::kCoast);
            right_front.SetMotorType(MotorType::kBrushless);
            right_front.SetInverted(true);  // Invert right motors
            right_front.SetDutyCycle(0.0);
            right_front.BurnFlash();
            
            right_rear.SetIdleMode(IdleMode::kCoast);
            right_rear.SetMotorType(MotorType::kBrushless);
            right_rear.SetInverted(true);  // Invert right motors
            right_rear.SetDutyCycle(0.0);
            right_rear.BurnFlash();
            
            RCLCPP_INFO(get_logger(), "Drivebase Motors configured successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to configure Drivebase motors: %s", e.what());
        }
        
        RCLCPP_INFO(get_logger(), "DrivebaseControl initialized!");
    }

private:
    SparkMax left_front;
    SparkMax left_rear;
    SparkMax right_front;
    SparkMax right_rear;

    
    
    bool manual_enabled;
    bool robot_disabled;
    double speed_multiplier = 0.8;
    double left_speed = 0.0;
    double right_speed = 0.0;
    
    double left_joystick_x = 0.0;
    double right_trigger = 0.0;
    double left_trigger = 0.0;
    
    bool x_button = false;
    bool y_button = false;
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    
 
    /**
     * @brief Teleoperate the robot via XBOX Controller
     * @details left/right speed = Wheel Speed for DIFF Drive mobile robot kinematics
    */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        if (robot_disabled) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Robot is disabled!");
            return;
        }
        SparkMax::Heartbeat();
        
        left_joystick_x = joy_msg->axes[0];
        right_trigger = joy_msg->axes[5];
        left_trigger = joy_msg->axes[2];
        x_button = joy_msg->buttons[2];
        y_button = joy_msg->buttons[3];
        
        // Drive triggers range from 0 to 1 instead of 1 to -1
        right_trigger = (1.0 - right_trigger) / 2.0;
        left_trigger = (1.0 - left_trigger) / 2.0;
        
        speed_multiplier = (x_button && y_button) ? 0.3 : (x_button ? 0.1 : 0.6);

        //Differential Drive Equations, but Teleop
        if (right_trigger > 0.05) {  // Forward
            left_speed = right_trigger - left_joystick_x;
            right_speed = right_trigger + left_joystick_x;
            //RCLCPP_INFO(get_logger(), "Forward: RT=%f, LX=%f -> L=%f, R=%f", right_trigger, left_joystick_x, left_speed, right_speed);
        
        } else if (left_trigger > 0.05) {  // Backward
            left_speed = -(left_trigger - left_joystick_x);
            right_speed = -(left_trigger + left_joystick_x);
            //RCLCPP_INFO(get_logger(), "Backward: LT=%f, LX=%f -> L=%f, R=%f", left_trigger, left_joystick_x, left_speed, right_speed);
        
        } else if (std::abs(left_joystick_x) > 0.05) {  // Turn in place
            left_speed = -left_joystick_x;
            right_speed = left_joystick_x;
            //RCLCPP_INFO(get_logger(), "Turn: LX=%f -> L=%f, R=%f", left_joystick_x, left_speed, right_speed);
        } else {
            left_speed = 0.0;
            right_speed = 0.0;
        }
        
        left_speed = std::clamp(left_speed * speed_multiplier, -1.0, 1.0);
        right_speed = std::clamp(right_speed * speed_multiplier, -1.0, 1.0);
        
        try {
            left_front.SetDutyCycle(left_speed);
            left_rear.SetDutyCycle(left_speed);
            right_front.SetDutyCycle(right_speed);
            right_rear.SetDutyCycle(right_speed);
            
            double left_voltage = left_speed * MAX_VOLTAGE;
            double right_voltage = right_speed * MAX_VOLTAGE;
            left_front.SetVoltage(left_voltage);
            left_rear.SetVoltage(left_voltage);
            right_front.SetVoltage(right_voltage);
            right_rear.SetVoltage(right_voltage);
            
            RCLCPP_DEBUG(get_logger(), "Motor Voltages Set: [L=%f V], [R=%f V]", left_voltage, right_voltage);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to set motor speeds: %s", e.what());
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