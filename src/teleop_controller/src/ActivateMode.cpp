/**
 * @brief Activates : Controller Teleoperation or Autonomous Control via ROS2 Twist Multiplexer(mux)
 */
#include "core.hpp"

class ActivateMode : public rclcpp::Node {
public:
    ActivateMode() : Node("twist_mux"), 
                     controller_teleop_enabled(true), 
                     autonomy_enabled(false) {
        cmd_vel_sub_teleop = create_subscription<geometry_msgs::msg::Twist>("teleop/cmd_vel", 10, std::bind(&ActivateMode::callback_teleop_cmdvel, this, std::placeholders::_1));
        cmd_vel_sub_autonomy = create_subscription<geometry_msgs::msg::Twist>("autonomy/cmd_vel", 10, std::bind(&ActivateMode::callback_autonomy_cmdvel, this, std::placeholders::_1));
        cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&ActivateMode::joy_callback, this, std::placeholders::_1));

        service_switch_operation_mode = create_service<teleop_controller::srv::SwitchMode>("switch_operation_mode", 
            std::bind(&ActivateMode::service_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        client_switch_operation_mode = create_client<teleop_controller::srv::SwitchMode>("switch_operation_mode");
        
        mode_publisher = create_publisher<std_msgs::msg::String>("current_mode", 10);
        
        current_mode_pub = create_wall_timer(1s, std::bind(&ActivateMode::publish_current_mode, this));
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher;
    rclcpp::TimerBase::SharedPtr current_mode_pub;
    rclcpp::Service<teleop_controller::srv::SwitchMode>::SharedPtr service_switch_operation_mode;
    rclcpp::Client<teleop_controller::srv::SwitchMode>::SharedPtr client_switch_operation_mode;
    TwistPublisher cmd_vel_pub;
    TwistSubscription cmd_vel_sub_teleop, cmd_vel_sub_autonomy;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    bool controller_teleop_enabled, autonomy_enabled;
    rclcpp::Time last_switch = rclcpp::Time(0);

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        rclcpp::Time now = this->get_clock()->now();
        if ((now - last_switch).seconds() < 2.0) return;

        if (joy_msg->buttons[7] && joy_msg->buttons[0]) {
            auto request = std::make_shared<teleop_controller::srv::SwitchMode::Request>();
            request->mode_name = "teleop";
            client_switch_operation_mode->async_send_request(
                request, std::bind(&ActivateMode::response_callback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Switching to teleop mode!");
            last_switch = now;
        } else if (joy_msg->buttons[6] && joy_msg->buttons[1]) {
            auto request = std::make_shared<teleop_controller::srv::SwitchMode::Request>();
            request->mode_name = "autonomy";
            client_switch_operation_mode->async_send_request(
                request, std::bind(&ActivateMode::response_callback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Switching to autonomous mode!");
            last_switch = now;
        }
    }

    void service_callback(const std::shared_ptr<teleop_controller::srv::SwitchMode::Request> request,
                          std::shared_ptr<teleop_controller::srv::SwitchMode::Response> response) {
        if (request->mode_name == "teleop") {
            controller_teleop_enabled = true;
            autonomy_enabled = false;
            response->success = true;
            response->message = "Switched to teleop mode";
        } else if (request->mode_name == "autonomy") {
            controller_teleop_enabled = false;
            autonomy_enabled = true;
            response->success = true;
            response->message = "Switched to autonomous mode";
        } else {
            response->success = false;
            response->message = "Unknown mode: " + request->mode_name;
        }
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        publish_current_mode(); // Update immediately
    }

    void response_callback(rclcpp::Client<teleop_controller::srv::SwitchMode>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(get_logger(), "Control Mode switch response: %s", response->message.c_str());
    }

    void publish_current_mode() {
        auto msg = std_msgs::msg::String();
        msg.data = controller_teleop_enabled ? "teleop" : "autonomy";
        mode_publisher->publish(msg);
    }

    void callback_teleop_cmdvel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (controller_teleop_enabled) cmd_vel_pub->publish(*msg);
    }

    void callback_autonomy_cmdvel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (autonomy_enabled) cmd_vel_pub->publish(*msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActivateMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}