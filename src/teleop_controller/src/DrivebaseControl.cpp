/**
 * @file drivebase_control.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief ROS2 node for managing the robot's drivebase and coordinating subsystem commands
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
         autonomy_enabled(false), 
         robot_disabled(false),
         last_heartbeat_time(this->now())
     {
        // Subscribe to joystick for direct control
        joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&DrivebaseControl::joy_callback, this, std::placeholders::_1));
        
        // Publish teleop commands for the multiplexer
        cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel/teleop", 10);
        
        // Subscribe to final multiplexed commands (for autonomy mode)
        cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&DrivebaseControl::cmd_vel_callback, this, std::placeholders::_1));

        // Client for changing control modes
        mode_client = create_client<teleop_controller::srv::SwitchMode>("activate_mode");
        
        // Subscribe to the current mode topic to stay in sync with the multiplexer
        mode_sub = create_subscription<std_msgs::msg::String>(
            "current_mode", 10, 
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "teleop") {
                    controller_teleop_enabled = true;
                    autonomy_enabled = false;
                } else if (msg->data == "autonomy") {
                    controller_teleop_enabled = false;
                    autonomy_enabled = true;
                }
            });
            
        // Create a timer for regular heartbeats and diagnostics
        heartbeat_timer = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DrivebaseControl::heartbeat_callback, this));
        
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

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub;
    rclcpp::Client<teleop_controller::srv::SwitchMode>::SharedPtr mode_client;
    rclcpp::TimerBase::SharedPtr heartbeat_timer;

    bool controller_teleop_enabled;
    bool autonomy_enabled;
    bool robot_disabled;
    
    rclcpp::Time last_heartbeat_time;
    
    /**
     * @brief Timer callback for regular heartbeat and diagnostics
     */
    void heartbeat_callback() {
        SparkMax::Heartbeat();
        auto now = this->now();
        auto elapsed = now - last_heartbeat_time;
        
        if (elapsed.seconds() > 5.0) {
            RCLCPP_DEBUG(get_logger(), "Drivebase heartbeat - Motors running");
            RCLCPP_INFO(get_logger(), "Motor temps: LF: %.1f°C, LR: %.1f°C, RF: %.1f°C, RR: %.1f°C", left_front.GetTemperature(), left_rear.GetTemperature(),right_front.GetTemperature(), right_rear.GetTemperature());
        }
    }

    void request_mode_change(const std::string& mode, bool activate) {
        while (!mode_client->wait_for_service(std::chrono::seconds(1))) {
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
     * @brief Handles multiplexed cmd_vel messages 
     * @details This callback handles commands from the multiplexer, which will be either from autonomy or from teleop (via the other node)
     * @note Only use these commands in autonomy mode or if they're coming back from the mux. This avoids double-processing our own teleop commands
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
       
        if (autonomy_enabled || !controller_teleop_enabled) {
            RCLCPP_DEBUG(get_logger(), "Cmd_vel input: linear_x=%.2f, angular_z=%.2f", msg->linear.x, msg->angular.z);
            calculate_motor_speeds(msg->linear.x, msg->angular.z);
        }
    }

    /**
     * @brief Interprets XBOX joystick Joy msgs
    */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        if (robot_disabled) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Robot is disabled!");
            return;
        }
        
        SparkMax::Heartbeat();
        
        if (joy_msg->buttons[5]) { // Guide button for teleop
            request_mode_change("teleop", true);
            controller_teleop_enabled = true;
            autonomy_enabled = false;
            RCLCPP_INFO(get_logger(), "Teleop mode activated");
        } else if (joy_msg->buttons[6]) { // Start button for autonomy
            request_mode_change("autonomy", true);
            controller_teleop_enabled = false;
            autonomy_enabled = true;
            RCLCPP_INFO(get_logger(), "Autonomy mode activated");
        }

        if (controller_teleop_enabled) {
            double linear_x = joy_msg->axes[1];  
            double angular_z = joy_msg->axes[2];

            RCLCPP_INFO(get_logger(), "Joy input: linear_x=%.2f, angular_z=%.2f", linear_x, angular_z);
            
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
    /**
     * @brief Calculates hardware motor speeds based on Twist msg
     * @details Applies Forward Kinematics
     * @var wheel_speed_left/right --> in m/s
     * @var rpm_left/right = m/s to RPM
     */
    void calculate_motor_speeds(double linear_x_velocity, double angular_z_velocity) {
        // Add constants if they're not defined in core.hpp
        const double MIN_THROTTLE_DEADZONE = 0.05;  // Adjust as needed
        const double WHEEL_BASE = 0.5;  // Distance between wheels in meters
        const double WHEEL_RADIUS = 0.1;  // Wheel radius in meters
        const double SPARKMAX_MAX_DUTY_CYCLE = 1.0;
        const double SPARKMAX_RPM_AVERAGE = 5676.0;  // Max RPM of your motors
        
        // Update last heartbeat time when receiving commands
        last_heartbeat_time = this->now();
        
        // Apply deadzone
        if (std::abs(linear_x_velocity) < MIN_THROTTLE_DEADZONE) { 
            linear_x_velocity = 0.0; 
        }
        if (std::abs(angular_z_velocity) < MIN_THROTTLE_DEADZONE) { 
            angular_z_velocity = 0.0; 
        }
        
        // If both inputs are zero after deadzone, just stop the motors directly
        if (linear_x_velocity == 0.0 && angular_z_velocity == 0.0) {
            left_front.SetDutyCycle(0.0);
            left_rear.SetDutyCycle(0.0);
            right_front.SetDutyCycle(0.0);
            right_rear.SetDutyCycle(0.0);
            return;
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
        
        //motor_cmd_left = (motor_cmd_left, -1.0, 1.0);
        //motor_cmd_right = (motor_cmd_right, -1.0, 1.0);

        // Use INFO level during troubleshooting so we can see these messages
        //RCLCPP_INFO(get_logger(), "Motor commands: left=%.2f, right=%.2f from input: linear=%.2f, angular=%.2f", motor_cmd_left, motor_cmd_right, linear_x_velocity, angular_z_velocity);

        // Apply to motors
        printf("MOTOR CMD LEFT = [%lf]\n",motor_cmd_left);
        printf("MOTOR CMD RIGHT = [%lf]\n",motor_cmd_right);
        //NOTE: TEMPORARY SOLUTION is x100 so the motors actually spin
        //NOTE: RUNS ON STARTUP??? SHOULD NOT HAPPEN. ALSO CLAMP DUTY CYCLE AND THE CODE IS GOOD TO GO!
        left_front.SetDutyCycle(motor_cmd_left*100);
        left_rear.SetDutyCycle(motor_cmd_left*100);
        right_front.SetDutyCycle(motor_cmd_right*100);
        right_rear.SetDutyCycle(motor_cmd_right*100);
        printf("AFTERMOTOR CMD LEFT = [%lf]\n",motor_cmd_left);
        printf("AFTER MOTOR CMD RIGHT = [%lf]\n",motor_cmd_right);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrivebaseControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}