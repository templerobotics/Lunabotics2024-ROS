#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/float64.hpp"

#include "SparkMax.hpp"


class dumping : public rclcpp::Node {
  public:
    dumping() : Node("dumping"),
    dumping_left("can0", 13),
    dumping_right("can0", 12)
    {
        initMotors();
        cmd_vel_dumping_subscriber = create_subscription<std_msgs::msg::Float64>(
        "/cmd_vel_dumping", 10, std::bind(&dumping::joy_callback_dumping, this, std::placeholders::_1));
    }
  private:
    SparkMax dumping_left, dumping_right;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_vel_dumping_subscriber;

    void joy_callback_dumping(const std_msgs::msg::Float64::SharedPtr velocity_msgs){
        double velocity = velocity_msgs->data;
        dumping_left.SetDutyCycle(velocity);
        dumping_right.SetDutyCycle(-velocity);
        // RCLCPP_INFO(get_logger(), "dumping cmd: %lf", velocity);
    }
    void initMotors(){
        RCLCPP_INFO(get_logger(), "Starting Dumping system initialization");
        // Leadscrews
        dumping_left.SetIdleMode(IdleMode::kCoast);
        dumping_left.SetMotorType(MotorType::kBrushless);
        dumping_left.ClearStickyFaults();
        dumping_left.SetDutyCycle(0.0);
        dumping_left.ResetFaults();
        dumping_left.BurnFlash();

        dumping_right.SetIdleMode(IdleMode::kCoast);
        dumping_right.SetMotorType(MotorType::kBrushless);
        dumping_right.ClearStickyFaults();
        dumping_right.SetDutyCycle(0.0);
        dumping_right.ResetFaults();
        dumping_right.BurnFlash();
        RCLCPP_INFO(get_logger(), "Finished Dumping system initialization");

    }
  };

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dumping>());
  rclcpp::shutdown();
  return 0;
}
