#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "SparkMax.hpp"

class drivebase : public rclcpp::Node {
  public:
    drivebase() : Node("drivebase"),
    left_front("can0", 5),
    left_rear("can0", 3),
    right_front("can0", 4),
    right_rear("can0", 2) 
    {
      velocity_subscriber = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&drivebase::velocity_callback, this, std::placeholders::_1));
    }
  private:
    
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr velocity_msg)
    {
      right_front.Heartbeat();
      // right_rear.Heartbeat();
      // left_front.Heartbeat();
      // left_rear.Heartbeat();
      double linear_velocity = velocity_msg->linear.x;
      double angular_velocity = velocity_msg->angular.z;
      
      if (std::abs(linear_velocity) < MIN_THROTTLE_DEADZONE) linear_velocity = 0.0;
      if (std::abs(angular_velocity) < MIN_THROTTLE_DEADZONE) angular_velocity = 0.0;
  
      double wheel_speed_left = linear_velocity - (angular_velocity * WHEEL_BASE / 2);
      double wheel_speed_right = linear_velocity + (angular_velocity * WHEEL_BASE / 2);
  
      double rpm_left = ((wheel_speed_left / (2 * M_PI * WHEEL_RADIUS)) * 60);
      double rpm_right = ((wheel_speed_right / (2 * M_PI * WHEEL_RADIUS)) * 60);
  
      double motor_cmd_left = rpm_left * (SPARKMAX_MAX_DUTY_CYCLE / SPARKMAX_RPM_AVERAGE);
      double motor_cmd_right = rpm_right * (SPARKMAX_MAX_DUTY_CYCLE / SPARKMAX_RPM_AVERAGE);
  
      motor_cmd_left = std::clamp(motor_cmd_left, -1.0, 1.0);
      motor_cmd_right = std::clamp(motor_cmd_right, -1.0, 1.0);

      left_front.SetDutyCycle(-motor_cmd_left);
      left_rear.SetDutyCycle(-motor_cmd_left);
      right_front.SetDutyCycle(-motor_cmd_right);
      right_rear.SetDutyCycle(-motor_cmd_right);
    }

    SparkMax left_front, left_rear, right_front,right_rear;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber;


    const double WHEEL_BASE = 0.6761581854;                
    const double WHEEL_RADIUS = 0.1183231568;                  
    const double MIN_THROTTLE_DEADZONE = 0.05;
    const double SPARKMAX_RPM = 292;               
    const double MAX_VOLTAGE = 12;
    const double SPARKMAX_RPM_AVERAGE = 63;
    const double SPARKMAX_MAX_DUTY_CYCLE = 1;
    const double GEAR_RATIO = 100.0;

  };

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drivebase>());
  rclcpp::shutdown();
  return 0;
}
