/**
 * @file drivebase_control.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief ROS2 node for managing the robot's drivebase and coordinating subsystem commands
 * @todo 2 Options for MUX : Global Bool states vs Wrapping service call w/ ros2 params. Try both
 * @version 0.4
 * @date 2025-03-10
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
         controller_teleop_enabled(true),
         autonomy_enabled(false)
     {
        joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&DrivebaseControl::joy_callback, this, std::placeholders::_1));
        cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("teleop/cmd_vel", 10);
        metrics_timer = create_wall_timer(std::chrono::milliseconds(500),std::bind(&DrivebaseControl::publish_motor_metrics, this));          
        mode_sub = create_subscription<std_msgs::msg::String>("current_mode", 10, 
            [this](const std_msgs::msg::String::SharedPtr msg) {
                controller_teleop_enabled = (msg->data == "teleop"); // Update global
                autonomy_enabled = (msg->data == "autonomy"); // Update global
                RCLCPP_INFO(get_logger(), "Mode enabled = %s", msg->data.c_str());
            });
            
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
            right_rear.SetInverted(true);  
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

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::TimerBase::SharedPtr metrics_timer;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub; 
    double linear_x, angular_z;
    bool controller_teleop_enabled, autonomy_enabled;

    /**
     * @brief Periodically logs motor metrics
     * @details Downgrade all Sparkmaxes to Firmware Version 24 
     * @details CAN ID 1 Sparkmax downgraded to version 24 already & worked perfectly
     */
    void publish_motor_metrics()
    {
        try {
           // float temperature = left_front.GetTemperature();
            //float voltage = left_front.GetVoltage();
            //float velocity = left_front.GetVelocity();
            //float duty_cycle = left_front.GetDutyCycle();
           // float pos = left_front.GetPosition();

            //RCLCPP_INFO(get_logger(), "Motor Metrics [Left Front] - Temp: %.6f C, Voltage: %.6f C ",temperature, voltage);
            //RCLCPP_INFO(get_logger(), "Motor Metrics [Left Front] - Duty Cycle: %.6f C, Velocity: %.6f C | Position =%.6f ", duty_cycle , velocity , pos);

            //if (temperature > 40.0) {RCLCPP_WARN(get_logger(), "Motor temperature high: %.1f°C", temperature);}

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to read motor metrics: %s", e.what());
        }
    }

    /**
     * @brief Interprets XBOX joystick Joy msgs
     * @details Crucial Part = Deadzone check. Makes sure that joy ONLY publishes velocities when the joysticks are moved.
     *         
    */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        left_front.Heartbeat();
        left_rear.Heartbeat();
        right_front.Heartbeat();
        right_rear.Heartbeat();

        if(controller_teleop_enabled){
            linear_x = joy_msg->axes[1];  
            angular_z = joy_msg->axes[3];
            
            if (std::abs(linear_x) < MIN_THROTTLE_DEADZONE && std::abs(angular_z) < MIN_THROTTLE_DEADZONE) {
                left_front.SetDutyCycle(0.0);
                left_rear.SetDutyCycle(0.0);
                right_front.SetDutyCycle(0.0);
                right_rear.SetDutyCycle(0.0);
                return;  
            }

            //RCLCPP_INFO(get_logger(), "Joy input: linear_x=%.2f, angular_z=%.2f", linear_x, angular_z);
            
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = linear_x;
            twist_msg.angular.z = angular_z;
            cmd_vel_pub->publish(twist_msg);
            calculate_motor_speeds(linear_x, angular_z);
        }
    }

    /**
     * @brief Calculates hardware motor speeds based on Twist msg
     * @details Applies Forward Kinematics
     * @var wheel_speed_left/right --> in m/s
     * @var rpm_left/right = m/s to RPM
     */
    void calculate_motor_speeds(double linear_x_velocity, double angular_z_velocity) {
        if (std::abs(linear_x_velocity) < MIN_THROTTLE_DEADZONE) { 
            linear_x_velocity = 0.0; 
        }
        if (std::abs(angular_z_velocity) < MIN_THROTTLE_DEADZONE) { 
            angular_z_velocity = 0.0; 
        }
  
        // Calculate wheel speeds based on differential drive kinematics
        double wheel_speed_left = linear_x_velocity - (angular_z_velocity * WHEEL_BASE/2);
        double wheel_speed_right = linear_x_velocity + (angular_z_velocity * WHEEL_BASE/2);
       
        // Convert to RPM
        double rpm_left = (wheel_speed_left / (2*M_PI * WHEEL_RADIUS)) * 60;
        double rpm_right = (wheel_speed_right / (2*M_PI * WHEEL_RADIUS)) * 60;
        
        // Convert RPM to duty cycle command
        double motor_cmd_left = rpm_left * (SPARKMAX_MAX_DUTY_CYCLE / SPARKMAX_RPM_AVERAGE);
        double motor_cmd_right = rpm_right * (SPARKMAX_MAX_DUTY_CYCLE / SPARKMAX_RPM_AVERAGE);
        
        //RCLCPP_INFO(get_logger(), "PRE CLAMP MOTOR DUTY: left=%.2f, right=%.2f", motor_cmd_left, motor_cmd_right);

        motor_cmd_left = std::clamp(motor_cmd_left, -1.0, 1.0);
        motor_cmd_right = std::clamp(motor_cmd_right, -1.0, 1.0);

        left_front.SetDutyCycle(motor_cmd_left);
        left_rear.SetDutyCycle(motor_cmd_left);
        right_front.SetDutyCycle(motor_cmd_right);
        right_rear.SetDutyCycle(motor_cmd_right);
    }


};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrivebaseControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}