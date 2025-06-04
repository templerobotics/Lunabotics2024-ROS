#include "core.hpp"
#include "DriveBase.hpp"
#include "SparkMax.hpp"
using namespace std::chrono_literals;

DrivebaseControl::DrivebaseControl()
    : Node("drivebase_control"),
      left_front("can0", MOTOR_FRONT_LEFT_CAN_ID),
      left_rear("can0", MOTOR_REAR_LEFT_CAN_ID),
      right_front("can0", MOTOR_FRONT_RIGHT_CAN_ID),
      right_rear("can0", MOTOR_REAR_RIGHT_CAN_ID),
      controller_teleop_enabled(true), autonomy_enabled(false), is_digging_running(false)
{
    // joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    //     "joy", 10, std::bind(&DrivebaseControl::joy_callback, this, std::placeholders::_1));

    // cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("teleop/cmd_vel", 10);

    // right_front_speed_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/right_front/speed", 50);
    // right_rear_speed_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/right_rear/speed", 50);
    // left_front_speed_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/left_front/speed", 50);
    // left_rear_speed_pub = create_publisher<std_msgs::msg::Float64>("/drivetrain/left_rear/speed", 50);

    // is_digging_running_sub = create_subscription<std_msgs::msg::Bool>(
    //     "/is_digging_running", 10,
    //     [this](const std_msgs::msg::Bool::SharedPtr msg) {
    //         is_digging_running = msg->data;
    //     });
    mode_sub = create_subscription<std_msgs::msg::String>(
        "current_mode", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            controller_teleop_enabled = (msg->data == "teleop");
            autonomy_enabled = (msg->data == "autonomy");
            RCLCPP_INFO(get_logger(), "Mode changed to: %s", msg->data.c_str());
        });
    cmd_vel_drive_sub = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_drivebase", 10, std::bind(&DrivebaseControl::joy_callback, this, std::placeholders::_1));
    // telemetry_timer = this->create_wall_timer(100ms, [this]() {
    //     try {
    //         auto msg = std_msgs::msg::Float64();
    //         msg.data = right_front.GetVelocity();
    //         right_front_speed_pub->publish(msg);
    //         msg.data = right_rear.GetVelocity();
    //         right_rear_speed_pub->publish(msg);
    //         msg.data = left_front.GetVelocity();
    //         left_front_speed_pub->publish(msg);
    //         msg.data = left_rear.GetVelocity();
    //         left_rear_speed_pub->publish(msg);
    //     } catch (const std::exception &e) {
    //         RCLCPP_ERROR(get_logger(), "Telemetry error: %s", e.what());
    //     }
    // });

    initMotors();
    RCLCPP_INFO(get_logger(), "DrivebaseControl initialized");
}

void DrivebaseControl::joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    left_front.Heartbeat();
    left_rear.Heartbeat();
    right_front.Heartbeat();
    right_rear.Heartbeat();
    linear_x = msg->linear.x;
    angular_z = msg->angular.z;

    if (std::abs(linear_x) < MIN_THROTTLE_DEADZONE && std::abs(angular_z) < MIN_THROTTLE_DEADZONE) {
        left_front.SetDutyCycle(0.0);
        left_rear.SetDutyCycle(0.0);
        right_front.SetDutyCycle(0.0);
        right_rear.SetDutyCycle(0.0);
    } else {
        // auto twist_msg = geometry_msgs::msg::Twist();
        // twist_msg.linear.x = linear_x;
        // twist_msg.angular.z = angular_z;
        // cmd_vel_pub->publish(twist_msg);
        calculate_motor_speeds(linear_x, angular_z);
    }
}

void DrivebaseControl::calculate_motor_speeds(double linear_x_velocity, double angular_z_velocity) {
    if (std::abs(linear_x_velocity) < MIN_THROTTLE_DEADZONE) linear_x_velocity = 0.0;
    if (std::abs(angular_z_velocity) < MIN_THROTTLE_DEADZONE) angular_z_velocity = 0.0;

    double wheel_speed_left = linear_x_velocity - (angular_z_velocity * WHEEL_BASE / 2);
    double wheel_speed_right = linear_x_velocity + (angular_z_velocity * WHEEL_BASE / 2);

    double rpm_left = ((wheel_speed_left / (2 * M_PI * WHEEL_RADIUS)) * 60);
    double rpm_right = ((wheel_speed_right / (2 * M_PI * WHEEL_RADIUS)) * 60);

    double motor_cmd_left = rpm_left * (SPARKMAX_MAX_DUTY_CYCLE / SPARKMAX_RPM_AVERAGE);
    double motor_cmd_right = rpm_right * (SPARKMAX_MAX_DUTY_CYCLE / SPARKMAX_RPM_AVERAGE);

    motor_cmd_left = clamp(motor_cmd_left, -1.0, 1.0);
    motor_cmd_right = clamp(motor_cmd_right, -1.0, 1.0);

    // float multiplier = is_digging_running ? 0.05 : 1;

    // motor_cmd_left *= multiplier;
    // motor_cmd_right *= multiplier;
    left_front.SetDutyCycle(-motor_cmd_left);
    left_rear.SetDutyCycle(-motor_cmd_left);
    right_front.SetDutyCycle(-motor_cmd_right);
    right_rear.SetDutyCycle(-motor_cmd_right);
}

void DrivebaseControl::initMotors() {
    try {
        RCLCPP_INFO(get_logger(), "Configuring drivetrain motors");
        auto setup_motor = [this](SparkMax &motor, bool inverted = false) {
            motor.SetIdleMode(IdleMode::kCoast);
            motor.SetMotorType(MotorType::kBrushless);
            motor.SetDutyCycle(0.0);
            motor.SetInverted(inverted);
            motor.ClearStickyFaults();
            motor.ResetFaults();
            motor.SetPeriodicStatus3Period(0);
            motor.SetPeriodicStatus4Period(0);
            motor.BurnFlash();
            motor.Heartbeat();
        };

        setup_motor(left_front, false);
        setup_motor(left_rear, false);
        setup_motor(right_front, true);
        setup_motor(right_rear, true);

        RCLCPP_INFO(get_logger(), "Drivetrain motors configured");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Motor init error: %s", e.what());
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrivebaseControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}