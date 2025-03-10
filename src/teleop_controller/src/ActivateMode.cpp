/**
 * @brief Activates : Controller Teleoperation or Autonomous Control via ROS2 Twist Multiplexer(mux)
 * @todo Determine if I need TwistStamped or just Twist
 * @note Consider booleans & Parameters set by Robot Drivebase
 * @note Robot default control mode is Teleoperation via XBOX Controller
 */
#include "core.hpp"

class ActivateMode : public rclcpp::Node {
public:
    ActivateMode() : Node("twist_mux"), controller_teleop_enabled(true), autonomy_enabled(false)
    {
        cmd_vel_sub_teleop = create_subscription<geometry_msgs::msg::Twist>("cmd_vel/teleop", 10, std::bind(&ActivateMode::callback_teleop_cmdvel, this, std::placeholders::_1));
        cmd_vel_sub_autonomy = create_subscription<geometry_msgs::msg::Twist>( "cmd_vel/autonomy", 10, std::bind(&ActivateMode::callback_autonomy_cmdvel, this, std::placeholders::_1));
        cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&ActivateMode::joy_callback, this, std::placeholders::_1));
        
        mode_service = create_service<teleop_controller::srv::SwitchMode>("activate_mode", std::bind(&ActivateMode::handle_mode_change, this, std::placeholders::_1, std::placeholders::_2));                     
        mode_publisher = create_publisher<std_msgs::msg::String>("current_mode", 10);
        current_mode_pub = create_wall_timer(1s, std::bind(&ActivateMode::publish_current_mode, this));
    }

    
private:
    rclcpp::Service<teleop_controller::srv::SwitchMode>::SharedPtr mode_service;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher;
    rclcpp::TimerBase::SharedPtr current_mode_pub;
    
    
    void handle_mode_change(const std::shared_ptr<teleop_controller::srv::SwitchMode::Request> request,std::shared_ptr<teleop_controller::srv::SwitchMode::Response> response)
    {
        if (request->mode_name == "teleop") {
            controller_teleop_enabled = request->activate;
            autonomy_enabled = !request->activate;
            response->success = true;
            response->message = "Teleop mode " + std::string(request->activate ? "activated" : "deactivated");
        } 
        else if (request->mode_name == "autonomy") {
            autonomy_enabled = request->activate;
            controller_teleop_enabled = !request->activate;
            response->success = true;
            response->message = "Autonomy mode " + std::string(request->activate ? "activated" : "deactivated");
        }
        else {
            response->success = false;
            response->message = "Unknown mode: " + request->mode_name;
        }
        
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }
    /**
     * @brief Prints current Control Mode every 1 second
     */
    void publish_current_mode() {
        auto msg = std_msgs::msg::String();
        msg.data = controller_teleop_enabled ? "teleop" : "autonomy";
        mode_publisher->publish(msg);
    }

    void callback_teleop_cmdvel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (controller_teleop_enabled) { cmd_vel_pub->publish(*msg); }
    }

    void callback_autonomy_cmdvel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (autonomy_enabled && !controller_teleop_enabled) { cmd_vel_pub->publish(*msg); }
    }

    /**
     * @brief 
     * @var joy_msg->buttons[5]
     * XBOX Guide Button
     * @var joy_msg->buttons[6]
     * XBOX Start Button
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        if (joy_msg->buttons[5]) { 
            controller_teleop_enabled = true;
            autonomy_enabled = false;
            RCLCPP_INFO(get_logger(), "Teleop mode activated");
        } else if (joy_msg->buttons[6]) { 
            controller_teleop_enabled = false;
            autonomy_enabled = true;
            RCLCPP_INFO(get_logger(), "Autonomy mode activated");
        }

    }




};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActivateMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}