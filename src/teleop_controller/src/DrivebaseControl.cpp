/**
 * @file drivebase_control.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief ROS2 node for managing the robot's drivebase and coordinating subsystem commands
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
        cmd_vel_pub = create_publisher<Twist>("cmd_vel", 10);
        cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&DrivebaseControl::cmd_vel_callback, this, std::placeholders::_1));
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
    bool robot_disabled;
    bool autonomy_enabled;

    /**
     * @brief Interprets XBOX joystick Joy msgs
    */
   void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        if (robot_disabled) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Robot is disabled!");return;}
        SparkMax::Heartbeat();
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = joy_msg->axes[1];  
        twist_msg.angular.z  = joy_msg->axes[2];
        cmd_vel_pub.publish(twist_msg);
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
        motor_cmd_left = std::clamp(motor_cmd_left, -1,1);
        motor_cmd_right = std::clamp(motor_cmd_right, -1,1);

        left_front.SetDutyCycle(motor_cmd_left);
        left_rear.SetDutyCycle(motor_cmd_left);
        right_front.SetDutyCycle(motor_cmd_right);
        right_rear.SetDutyCycle(motor_cmd_right);

    }

    /**
     * @brief Interprets publishes Twist msg to actuate the Robot's Motors via XBOX Controller Teleop
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg){
        double linear_x_velocity = twist_msg->linear.x;
        double angular_z_velocity = twist_msg->angular.z;
        calculate_motor_speeds(linear_x_velocity, angular_z_velocity);
    }

   

    
};
 
 int main(int argc, char* argv[]) {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<DrivebaseControl>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
 }