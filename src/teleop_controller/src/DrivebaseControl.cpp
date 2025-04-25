#include "core.hpp"

class DrivebaseControl : public rclcpp::Node {
public:
    DrivebaseControl() 
        : Node("drivebase_control"), 
          left_front("can0", MOTOR_FRONT_LEFT_CAN_ID), left_rear("can0", MOTOR_REAR_LEFT_CAN_ID), 
          right_front("can0", MOTOR_FRONT_RIGHT_CAN_ID), right_rear("can0", MOTOR_REAR_RIGHT_CAN_ID),
          controller_teleop_enabled(true), autonomy_enabled(false) {
        joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&DrivebaseControl::joy_callback, this, std::placeholders::_1));
        //ALL defined for Foxglove to visualize them in real-time. 
        cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("teleop/cmd_vel", 10);
        right_front_speed_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/right_front/speed", 10);
        right_rear_speed_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/right_rear/speed", 10);
        left_front_speed_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/left_front/speed", 10);
        left_rear_speed_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/left_rear/speed", 10);
        right_front_temp_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/right_front/temp", 10);
        right_rear_temp_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/right_rear/temp", 10);
        left_front_temp_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/left_front/temp", 10);
        left_rear_temp_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/left_rear/temp", 10);
        mode_sub = create_subscription<std_msgs::msg::String>("current_mode", 10, 
            [this](const std_msgs::msg::String::SharedPtr msg) {
                controller_teleop_enabled = (msg->data == "teleop");
                autonomy_enabled = (msg->data == "autonomy");
                RCLCPP_INFO(get_logger(), "Mode enabled = %s", msg->data.c_str());
            });
            //Todo : Put inside a function called initMotors()
        try {
            RCLCPP_INFO(get_logger(), "Configuring Drivebase Motors");
            left_front.SetIdleMode(IdleMode::kCoast);
            left_front.SetMotorType(MotorType::kBrushless);
            left_front.SetDutyCycle(0.0);
            left_front.SetInverted(false);
            left_front.ClearStickyFaults();
            left_front.ResetFaults();
            left_front.SetPeriodicStatus3Period(0);
            left_front.SetPeriodicStatus4Period(0);
            left_front.BurnFlash();

            left_rear.SetIdleMode(IdleMode::kCoast);
            left_rear.SetMotorType(MotorType::kBrushless);
            left_rear.SetDutyCycle(0.0);
            left_rear.SetInverted(false);
            left_rear.ClearStickyFaults();
            left_rear.ResetFaults();
            left_rear.SetPeriodicStatus3Period(0);
            left_rear.SetPeriodicStatus4Period(0);
            left_rear.BurnFlash();

            right_front.SetIdleMode(IdleMode::kCoast);
            right_front.SetMotorType(MotorType::kBrushless);
            right_front.SetDutyCycle(0.0);
            right_front.SetInverted(true);
            right_front.ClearStickyFaults();
            right_front.ResetFaults();
            right_front.SetPeriodicStatus3Period(0);
            right_front.SetPeriodicStatus4Period(0);
            right_front.BurnFlash();

            right_rear.SetIdleMode(IdleMode::kCoast);
            right_rear.SetMotorType(MotorType::kBrushless);
            right_rear.SetInverted(true);
            right_rear.SetDutyCycle(0.0);
            right_rear.ClearStickyFaults();
            right_rear.ResetFaults();
            right_rear.SetPeriodicStatus3Period(0);
            right_rear.SetPeriodicStatus4Period(0);
            right_rear.BurnFlash();
            RCLCPP_INFO(get_logger(), "Drivebase Motors configured successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to configure Drivebase motors: %s", e.what());
        }
        
        RCLCPP_INFO(get_logger(), "DrivebaseControl initialized!");


        // // Foxglove does not publish exactly real-time, so im messing w/ "optimal" spot in code that allows metrics to be published w/ 0 delay
        telemetry_timer = this->create_wall_timer(
            100ms, [this]() {
        try {
            auto msg = std_msgs::msg::Float64();
            msg.data = right_front.GetVelocity();
            right_front_speed_pub->publish(msg);
            msg.data = right_rear.GetVelocity();
            right_rear_speed_pub->publish(msg);
            msg.data = left_front.GetVelocity();
            left_front_speed_pub->publish(msg);
            msg.data = left_rear.GetVelocity();
            left_rear_speed_pub->publish(msg);
            msg.data = right_front.GetTemperature();
            right_front_temp_pub->publish(msg);
            msg.data = right_rear.GetTemperature();
            right_rear_temp_pub->publish(msg);
            msg.data = left_front.GetTemperature();
            left_front_temp_pub->publish(msg);
            msg.data = left_rear.GetTemperature();
            left_rear_temp_pub->publish(msg);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Failed to read motor metrics: %s", e.what());
            }
        });
    }

private:
    SparkMax left_front, left_rear, right_front, right_rear;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_front_speed_pub, right_rear_speed_pub, left_front_speed_pub, left_rear_speed_pub, right_front_temp_pub, right_rear_temp_pub, left_front_temp_pub, left_rear_temp_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub; 
    rclcpp::TimerBase::SharedPtr telemetry_timer;
    double linear_x, angular_z;
    bool controller_teleop_enabled, autonomy_enabled;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        left_front.Heartbeat();
        
        if (controller_teleop_enabled) {
            linear_x = joy_msg->axes[1];  
            angular_z = joy_msg->axes[2];
            
            if (std::abs(linear_x) < MIN_THROTTLE_DEADZONE && std::abs(angular_z) < MIN_THROTTLE_DEADZONE) {
                left_front.SetDutyCycle(0.0);
                left_rear.SetDutyCycle(0.0);
                right_front.SetDutyCycle(0.0);
                right_rear.SetDutyCycle(0.0);
            } else {
                auto twist_msg = geometry_msgs::msg::Twist();
                twist_msg.linear.x = linear_x;
                twist_msg.angular.z = angular_z;
                cmd_vel_pub->publish(twist_msg);
                calculate_motor_speeds(linear_x, angular_z);
            }
        }
        
    }

    void calculate_motor_speeds(double linear_x_velocity, double angular_z_velocity) {
        if (std::abs(linear_x_velocity) < MIN_THROTTLE_DEADZONE) linear_x_velocity = 0.0;
        if (std::abs(angular_z_velocity) < MIN_THROTTLE_DEADZONE) angular_z_velocity = 0.0;
        double wheel_speed_left = linear_x_velocity - (angular_z_velocity * WHEEL_BASE / 2);
        double wheel_speed_right = linear_x_velocity + (angular_z_velocity * WHEEL_BASE / 2);
        //double rpm_left = (wheel_speed_left / (2 * M_PI * WHEEL_RADIUS)) * 60;
        //double rpm_right = (wheel_speed_right / (2 * M_PI * WHEEL_RADIUS)) * 60;
        double rpm_left = ((wheel_speed_left / (2 * M_PI * WHEEL_RADIUS)) * 60); //test to make sure shit still works fine
        double rpm_right = ((wheel_speed_right / (2 * M_PI * WHEEL_RADIUS)) * 60);

        double motor_cmd_left = rpm_left * (SPARKMAX_MAX_DUTY_CYCLE / SPARKMAX_RPM_AVERAGE);
        double motor_cmd_right = rpm_right * (SPARKMAX_MAX_DUTY_CYCLE / SPARKMAX_RPM_AVERAGE);
        motor_cmd_left = clamp(motor_cmd_left, -1.0, 1.0);
        motor_cmd_right = clamp(motor_cmd_right, -1.0, 1.0);
        
        left_front.SetDutyCycle(-motor_cmd_left);
        left_rear.SetDutyCycle(-motor_cmd_left);
        right_front.SetDutyCycle(-motor_cmd_right);
        right_rear.SetDutyCycle(-motor_cmd_right);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrivebaseControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}