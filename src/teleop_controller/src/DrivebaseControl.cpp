/**
 * @file drivebase_control.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief ROS2 node for managing the robot's drivebase and coordinating subsystem commands
 * @todo UTILIZE CALC MOTOR SPEEDS! I recently deleted cmd_vel sub due to implementing Twist MUX
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
         controller_teleop_enabled(true),
         autonomy_enabled(false), 
         robot_disabled(false)
     {
        joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&DrivebaseControl::joy_callback, this, std::placeholders::_1));
        cmd_vel_pub = create_publisher<Twist>("cmd_vel/teleop", 10);    
        cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                this->calculate_motor_speeds(msg->linear.x, msg->angular.z);
            });

        mode_client = create_client<teleop_controller::srv::SwitchMode>("activate_mode");
        
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

    JoySubscription joy_sub;
    TwistPublisher cmd_vel_pub;
    TwistSubscription cmd_vel_sub;

    bool controller_teleop_enabled;
    bool autonomy_enabled;
    bool robot_disabled;

    rclcpp::Client<teleop_controller::srv::SwitchMode>::SharedPtr mode_client;

    void request_mode_change(const std::string& mode, bool activate) {
        while (!mode_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for mode service");
                return;
            }
            RCLCPP_INFO(get_logger(), "Waiting for activate_mode service...");
        }
        
        auto request = std::make_shared<teleop_controller::srv::SwitchMode::Request>();
        request->mode_name = mode;
        request->activate = activate;
        auto future = mode_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            RCLCPP_INFO(get_logger(), "Mode change result: %s", response->message.c_str());
        }

    }

    /**
     * @brief Interprets XBOX joystick Joy msgs
    */
   void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        if (robot_disabled) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Robot is disabled!");return;}
        SparkMax::Heartbeat();
        
        if (joy_msg->buttons[5]) { // Guide button for teleop
            request_mode_change("teleop", true);
        } else if (joy_msg->buttons[6]) { // Start button for autonomy
            request_mode_change("autonomy", true);
        }

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = joy_msg->axes[1];  
        twist_msg.angular.z  = joy_msg->axes[2];
        cmd_vel_pub->publish(twist_msg);
    
    }
    /**
     * @brief Calculates hardware motor speeds based on Twist msg
     * @details Applies Forward Kinematics
     * @var wheel_speed_left/right --> in m/s
     * @var rpm_left/right = m/s to RPM
     */
    void calculate_motor_speeds(double linear_x_velocity, double angular_z_velocity){
        if( abs(linear_x_velocity) < MIN_THROTTLE_DEADZONE ){ linear_x_velocity = 0.0; }
        if( abs(angular_z_velocity) < MIN_THROTTLE_DEADZONE ){ angular_z_velocity = 0.0; }
        
        double wheel_speed_left = linear_x_velocity - (angular_z_velocity * WHEEL_BASE/2);
        double wheel_speed_right = linear_x_velocity + (angular_z_velocity * WHEEL_BASE/2);
       
        double rpm_left = (wheel_speed_left / (2*M_PI * WHEEL_RADIUS) ) * 60;
        double rpm_right = (wheel_speed_right / (2*M_PI * WHEEL_RADIUS) ) * 60;
        
        double motor_cmd_left = rpm_left * ( SPARKMAX_MAX_DUTY_CYCLE / SPARKMAX_RPM_AVERAGE);
        double motor_cmd_right = rpm_right * ( SPARKMAX_MAX_DUTY_CYCLE / SPARKMAX_RPM_AVERAGE);
        motor_cmd_left = std::clamp(motor_cmd_left, -1.0,1.0);
        motor_cmd_right = std::clamp(motor_cmd_right, -1.0,1.0);

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