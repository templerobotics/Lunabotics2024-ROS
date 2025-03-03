/**
 * @file drivebase_control.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief ROS2 node for managing the robot's drivebase and coordinating subsystem commands
 * @todo Simplify Info print commands to only say Forward/Backwards/Turning
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
    /**
     * @brief Delete this variable if during testing proves to be pointless. From test bench Triggers handle speed well enough
    */
    double speed_multiplier = 0.8;

    double left_speed = 0.0;
    double right_speed = 0.0;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    
    /**
     * @brief Teleoperate the robot via XBOX Controller
     * @details Left stick Y-axis (forward/backward) &  Right stick X-axis (turning)
     * @var deadband --> Prevents small joystick movements from causing unwanted motor activation
     * @var left/right speed = Calculate left and right side power using arcade drive formula
     * @var maxMagnitude Normalize values to keep them between -1.0 and 1.0

    */
   void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    if (robot_disabled) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Robot is disabled!");
        return;
    }
    SparkMax::Heartbeat();
    
    double forward = joy_msg->axes[1];  
    double turn = joy_msg->axes[2];     
    
    const double deadband = 0.05;
    if (std::abs(forward) < deadband) forward = 0.0;
    if (std::abs(turn) < deadband) turn = 0.0;
    
    left_speed = forward + turn;
    right_speed = forward - turn;

    double maxMagnitude = std::max(std::abs(left_speed), std::abs(right_speed));
    if (maxMagnitude > 1.0) {
        left_speed /= maxMagnitude;
        right_speed /= maxMagnitude;
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
        
        RCLCPP_DEBUG(get_logger(), "Drivebase Motor Voltages Set: [L=%f V], [R=%f V]", left_voltage, right_voltage);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to set Drivebase Motor speeds: %s", e.what());
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