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
        
        mode_publisher = create_publisher<std_msgs::msg::String>("current_mode", 10);
        current_mode_pub = create_wall_timer(5s, std::bind(&ActivateMode::publish_current_mode, this));
        
        // Initialize with the node's clock
        last_switch = this->get_clock()->now();
        
        RCLCPP_INFO(get_logger(), "Activate Mode initialized - Press Guide + B button to switch robot control modes");
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher;
    rclcpp::TimerBase::SharedPtr current_mode_pub;
    TwistPublisher cmd_vel_pub;
    TwistSubscription cmd_vel_sub_teleop, cmd_vel_sub_autonomy;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    bool controller_teleop_enabled, autonomy_enabled;
    rclcpp::Time last_switch;

    /**
     * @brief Toggles between Teleop & Autonomous Modes
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        rclcpp::Time now = this->get_clock()->now();
        if ((now - last_switch).seconds() < 2.0) return;
       
        if (joy_msg->buttons[1] && joy_msg->buttons[6]) {  
            RCLCPP_INFO(get_logger(), "Toggling Control Mode!");
            
            // Toggle modes 

            if(!controller_teleop_enabled && !autonomy_enabled){
                controller_teleop_enabled = true;
            }else{
                controller_teleop_enabled = !controller_teleop_enabled;
                autonomy_enabled = !autonomy_enabled;
            }
         
            
            if (controller_teleop_enabled) {
                RCLCPP_INFO(get_logger(), "Teleoperation Mode Activated!");
            } else if (autonomy_enabled){
                RCLCPP_INFO(get_logger(), "Autonomous Mode Activated!");
            }
            else{
                RCLCPP_INFO(get_logger(), "Communications Killed!");
            }
            
            publish_current_mode(); 
            last_switch = now;
        }
        if(joy_msg->buttons[7] && joy_msg->buttons[5]){
            controller_teleop_enabled = false;
            autonomy_enabled = false;
        }
    }

    void publish_current_mode() {
        auto msg = std_msgs::msg::String();
        if(!controller_teleop_enabled  && !autonomy_enabled ){
            msg.data = "Communications Killed";
        } else{
            msg.data = controller_teleop_enabled ? "teleop" : "autonomy";
        }
       
         
        mode_publisher->publish(msg);
        RCLCPP_INFO(get_logger(), "Mode enabled = %s", msg.data.c_str());
    }

    void callback_teleop_cmdvel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (controller_teleop_enabled) {
            cmd_vel_pub->publish(*msg);
        }
    }

    void callback_autonomy_cmdvel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (autonomy_enabled) {
            cmd_vel_pub->publish(*msg);
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