#include <algorithm>
 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>
 
 
#include "SparkMax.hpp"
 
 
class autonomy : public rclcpp::Node {
public:
  autonomy() : Node("autonomy"), in_autonomy_mode(false)
  {
    cmd_vel_drivebase_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    cmd_vel_leadscrew_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_leadscrew", 10);
    cmd_vel_actuator_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_actuator", 10);
    cmd_vel_digging_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_digging", 10);
    cmd_vel_dumping_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_dumping", 10);
    cmd_state = create_subscription<std_msgs::msg::String>(
      "/cmd_state", 10, std::bind(&autonomy::joy_callback_autonomy, this, std::placeholders::_1));
    actuator_state_subscriber = create_subscription<std_msgs::msg::String>(
      "/actuator_state", 10, std::bind(&autonomy::actuator_state_callback, this, std::placeholders:: _1));
    leadscrew_timer = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&autonomy::leadscrew_timer_callback, this));
    leadscrew_timer->cancel();  // Don’t run at start
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_drivebase_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_vel_leadscrew_pub, cmd_vel_actuator_pub, cmd_vel_digging_pub, cmd_vel_dumping_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_state, actuator_state_subscriber;
  rclcpp::TimerBase::SharedPtr leadscrew_timer;
  bool in_autonomy_mode;
  std::string actuator_state = "";

  void actuator_state_callback(const std_msgs::msg::String::SharedPtr msg){
    actuator_state = msg->data;
    if (in_autonomy_mode && actuator_state == "Raised" && !leadscrew_timer->is_ready()) {
      RCLCPP_INFO(this->get_logger(), "Actuator raised, starting leadscrew...");
      leadscrew_timer->reset();  // Start leadscrew movement
      auto msg = std_msgs::msg::Float64();
      msg.data = 0.2;
      cmd_vel_dumping_pub->publish(msg);
    }
  }
  void joy_callback_autonomy(const std_msgs::msg::String::SharedPtr command_msg){
    if(command_msg->data == "autonomy" && !in_autonomy_mode){
      RCLCPP_INFO(get_logger(), "Starting auto");
      in_autonomy_mode = true;
      
      auto msg = std_msgs::msg::Float64();
      msg.data = 1.0;
      cmd_vel_digging_pub->publish(msg);

      msg.data = -1.0;
      cmd_vel_actuator_pub->publish(msg);
    } else {
      in_autonomy_mode = false;
      leadscrew_timer->cancel();  // Stop timer if leaving autonomy
    }
  }

  void leadscrew_timer_callback() {
    if (!in_autonomy_mode) return;
    auto msg = std_msgs::msg::Float64();
    msg.data = -0.5;
    cmd_vel_leadscrew_pub->publish(msg);
  }
};

 
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy>());
  rclcpp::shutdown();
  return 0;
}