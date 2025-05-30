/**
 * @brief Activates : Controller Teleoperation or Autonomous Control via ROS2 Twist Multiplexer(mux)
 */
#include "core.hpp"

class ActivateMode : public rclcpp::Node {
public:
    ActivateMode() : Node("twist_mux"), 
                     controller_teleop_enabled(true), 
                     autonomy_enabled(false) {
        cmd_vel_drive_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_drivebase", 10);
        cmd_pos_actuator_pub = create_publisher<std_msgs::msg::Float64>("cmd_pos_actuator", 10);
        cmd_vel_diggingBelt_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_digging", 10);
        cmd_vel_leadscrew_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_leadscrew", 10);
        cmd_vel_dumpingBelt_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_dumping", 10);
        is_dumping_running_pub = create_publisher<std_msgs::msg::Bool>("is_dumping_running", 10);
        joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&ActivateMode::joy_callback, this, std::placeholders::_1));
        
        mode_publisher = create_publisher<std_msgs::msg::String>("current_mode", 10);
        current_mode_pub = create_wall_timer(5s, std::bind(&ActivateMode::publish_current_mode, this));
        
        // Initialize with the node's clock
        last_switch = this->get_clock()->now();
        is_digging_running_sub = create_subscription<std_msgs::msg::Bool>(
            "/is_digging_running", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                is_digging_running = msg->data;
            });
        actuator_running_left_sub = create_subscription<std_msgs::msg::Bool>(
            "/actuator_running_left", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                actuator_running_left= msg->data;
            });
        actuator_running_right_sub = create_subscription<std_msgs::msg::Bool>(
            "/actuator_running_right", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                actuator_running_right = msg->data;
            });
        is_dumping_running_sub = create_subscription<std_msgs::msg::Bool>(
            "/is_dumping_running" , 10, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                dumping_belt_running = msg->data;
        });
        RCLCPP_INFO(get_logger(), "Activate Mode initialized - Press Guide + B button to switch robot control modes");
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_vel_diggingBelt_pub, cmd_pos_actuator_pub, cmd_vel_leadscrew_pub, cmd_vel_dumpingBelt_pub, cmd_vel_drivetrain_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_dumping_running_pub;
    rclcpp::TimerBase::SharedPtr current_mode_pub;
    TwistPublisher cmd_vel_drive_pub;
    TwistSubscription cmd_vel_sub_teleop, cmd_vel_sub_autonomy;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_digging_running_sub, actuator_running_left_sub, actuator_running_right_sub, is_dumping_running_sub;
    bool controller_teleop_enabled, autonomy_enabled, actuator_command_up = false, actuator_command_down = false, actuator_running_right = false, actuator_running_left = false, is_digging_running = false, current_a, last_a_state = false, current_y,
    last_y_state = false, current_b, last_b_state = false, current_x, last_x_state = false, dpad_right, last_dpad_right = false, dpad_left, last_dpad_left = false, dumping_belt_running = false;
    double cmd_vel_diggingBelt = 0.0, cmd_vel_actuator = 0.0, cmd_vel_leadscrew = 0.0, cmd_vel_dumpingBelt = 0.0;
    rclcpp::Time last_switch;

    /**
     * @brief Toggles between Teleop & Autonomous Modes
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        rclcpp::Time now = this->get_clock()->now();
        if ((now - last_switch).seconds() < 2.0) return;
        if (joy_msg->buttons[9] && joy_msg->buttons[10]) {  
            
            RCLCPP_INFO(get_logger(), "Toggling Control Mode!");
            // RCLCPP_INFO(get_logger(), "Teleop mode: %s", controller_teleop_enabled);
            
            // Toggle modes 
            controller_teleop_enabled = !controller_teleop_enabled;
            autonomy_enabled = !autonomy_enabled;
        }
        if (controller_teleop_enabled) {
            RCLCPP_INFO(get_logger(), "Teleoperation Mode Activated!");

            current_b = joy_msg->buttons[1];
            if (current_b && !last_b_state) {
                if (actuator_running_right || actuator_running_left) {
                    cmd_vel_actuator = 0.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_actuator;
                    cmd_pos_actuator_pub->publish(msg);
                } else {
                    cmd_vel_actuator = 1.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_actuator;
                    cmd_pos_actuator_pub->publish(msg);
                }
            }
            last_b_state = current_b;

            current_x = joy_msg->buttons[2];
            if (current_x && !last_x_state){
                if (actuator_running_right || actuator_running_left) {
                    cmd_vel_actuator = 0.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_actuator;
                    cmd_pos_actuator_pub->publish(msg);
                } else {
                    cmd_vel_actuator = -1.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_actuator;
                    cmd_pos_actuator_pub->publish(msg);
                }
            }
            last_x_state = current_x;


            current_a = joy_msg->buttons[0];
            if (current_a && !last_a_state) {  // Button A just pressed
                if (is_digging_running) {
                    cmd_vel_diggingBelt = 0.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_diggingBelt;
                    cmd_vel_diggingBelt_pub->publish(msg);
                } else {
                    cmd_vel_diggingBelt = 1.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_diggingBelt;
                    cmd_vel_diggingBelt_pub->publish(msg);
                }  
            }
            last_a_state = current_a;


            current_y = joy_msg->buttons[3];
            if (current_y && !last_y_state) {  // Button Y just pressed
                if (is_digging_running) {
                    cmd_vel_diggingBelt = 0.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_diggingBelt;
                    cmd_vel_diggingBelt_pub->publish(msg);
                } else {
                    cmd_vel_diggingBelt = -1.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_diggingBelt;
                    cmd_vel_diggingBelt_pub->publish(msg);
                }
            }
            last_y_state = current_y;

            double rightTrigger = joy_msg->axes[4];
            double leftTrigger = joy_msg->axes[5]; //2 when on the nuc
            double leadscrewSpeed;
            if(rightTrigger >= -1 && rightTrigger <= 1 && leftTrigger >= -1 && leftTrigger <= 1){
                if (leftTrigger > MIN_THROTTLE_DEADZONE && rightTrigger > MIN_THROTTLE_DEADZONE) {
                    leadscrewSpeed = 0;
                }
                else if (leftTrigger > MIN_THROTTLE_DEADZONE){
                    leadscrewSpeed = -1*leftTrigger;
                }
                else if (rightTrigger > MIN_THROTTLE_DEADZONE){
                    leadscrewSpeed = rightTrigger;
                }
                cmd_vel_leadscrew = clamp(leadscrewSpeed, -1.0, 1.0);
                auto msg = std_msgs::msg::Float64();
                msg.data = cmd_vel_leadscrew;
                cmd_vel_leadscrew_pub->publish(msg);
            }

            dpad_right = joy_msg->buttons[14];
            if (dpad_right && !last_dpad_right) {
                auto msg = std_msgs::msg::Float64();
                auto msgBool = std_msgs::msg::Bool();
                if (dumping_belt_running) {
                    cmd_vel_dumpingBelt = 0.0;
                    msg.data = cmd_vel_dumpingBelt;
                    cmd_vel_dumpingBelt_pub->publish(msg);
                    dumping_belt_running = false;
                    msgBool.data = dumping_belt_running;
                    is_dumping_running_pub->publish(msgBool);
                } else {
                    cmd_vel_dumpingBelt = 0.25;
                    msg.data = cmd_vel_dumpingBelt;
                    cmd_vel_dumpingBelt_pub->publish(msg);
                    dumping_belt_running = true;
                    msgBool.data = dumping_belt_running;
                    is_dumping_running_pub->publish(msgBool);
                }
            }
            last_dpad_right = dpad_right;
            dpad_left = joy_msg->buttons[13];
            if (dpad_left && !last_dpad_left) {
                auto msg = std_msgs::msg::Float64();
                auto msgBool = std_msgs::msg::Bool();
                if (dumping_belt_running) {
                    cmd_vel_dumpingBelt = 0.0;
                    msg.data = cmd_vel_dumpingBelt;
                    cmd_vel_dumpingBelt_pub->publish(msg);
                    dumping_belt_running = false;
                    msgBool.data = dumping_belt_running;
                    is_dumping_running_pub->publish(msgBool);
                } else {
                    cmd_vel_dumpingBelt = -0.25;
                    msg.data = cmd_vel_dumpingBelt;
                    cmd_vel_dumpingBelt_pub->publish(msg);
                    dumping_belt_running = true;
                    msgBool.data = dumping_belt_running;
                    is_dumping_running_pub->publish(msgBool);
                }
            }
            last_dpad_left = dpad_left;
            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = joy_msg->axes[1];
            twist.angular.z = joy_msg->axes[2];
            cmd_vel_drive_pub->publish(twist);
            
        } else {
            RCLCPP_INFO(get_logger(), "Autonomous Mode Activated!");
        }
        
        publish_current_mode(); 
        last_switch = now;
    }

    void publish_current_mode() {
        auto msg = std_msgs::msg::String();
        msg.data = controller_teleop_enabled ? "teleop" : "autonomy";
        mode_publisher->publish(msg);
        RCLCPP_INFO(get_logger(), "Mode enabled = %s", msg.data.c_str());
    }

    void callback_teleop_cmdvel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (controller_teleop_enabled) {
            cmd_vel_drive_pub->publish(*msg);
        }
    }

    void callback_autonomy_cmdvel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (autonomy_enabled) {
            cmd_vel_drive_pub->publish(*msg);
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