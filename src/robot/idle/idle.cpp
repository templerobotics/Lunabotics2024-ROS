#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <geometry_msgs/msg/twist.hpp>


#include "SparkMax.hpp"


class idle : public rclcpp::Node {
  public:
    idle() : Node("idle")
    {
        cmd_state_subscriber = create_subscription<std_msgs::msg::String>(
        "/cmd_state", 10, std::bind(&idle::joy_callback_dumping, this, std::placeholders::_1));
        cmd_vel_drivebase_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        cmd_vel_leadscrew_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_leadscrew", 10);
        cmd_vel_actuator_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_actuator", 10);
        cmd_vel_digging_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_digging", 10);
        cmd_vel_dumping_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_dumping", 10);
        cmd_rest_pub = create_publisher<std_msgs::msg::Bool>("reset", 10);
    }
  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_state_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_vel_digging_pub, cmd_vel_dumping_pub, cmd_vel_actuator_pub, cmd_vel_leadscrew_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_drivebase_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_rest_pub;
    void joy_callback_dumping(const std_msgs::msg::String::SharedPtr command){
        if(command->data == "idle"){
            RCLCPP_INFO(get_logger(), "Starting idle");
            auto msg = std_msgs::msg::Float64();
            msg.data = 0.0;
            cmd_vel_actuator_pub->publish(msg);
            cmd_vel_digging_pub->publish(msg);
            cmd_vel_dumping_pub->publish(msg);
            cmd_vel_leadscrew_pub->publish(msg);

            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            cmd_vel_drivebase_pub->publish(twist);

            rclcpp::Time start_time = this->get_clock()->now();
            while ((this->get_clock()->now() - start_time).seconds() < 2.0){
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }

            auto msgReset = std_msgs::msg::Bool();
            msgReset.data = false;
            cmd_rest_pub->publish(msgReset);
        }
    }
  };

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<idle>());
  rclcpp::shutdown();
  return 0;
}