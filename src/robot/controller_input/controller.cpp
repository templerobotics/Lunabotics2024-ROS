#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"




class controller : public rclcpp::Node {
    public:
    controller() : Node("controller"){
        cmd_vel_drivebase_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        cmd_vel_leadscrew_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_leadscrew", 10);
        cmd_vel_actuator_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_actuator", 10);
        cmd_vel_digging_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_digging", 10);
        cmd_vel_dumping_pub = create_publisher<std_msgs::msg::Float64>("cmd_vel_dumping", 10);
        cmd_state_pub = create_publisher<std_msgs::msg::String>("cmd_state", 10);
        joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&controller::joy_callback, this, std::placeholders::_1));
        cmd_reset_subscriber = create_subscription<std_msgs::msg::Bool>("/reset", 10, [this](const std_msgs::msg::Bool::SharedPtr msg){
            at_rest = msg->data;
        });
        }
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_drivebase_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_vel_leadscrew_pub, cmd_vel_actuator_pub, cmd_vel_digging_pub, cmd_vel_dumping_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_state_pub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_reset_subscriber;
        double cmd_vel_leadscrew = 0.0, cmd_vel_actuator = 0.0, cmd_vel_digging = 0.0, cmd_vel_dumping = 0.0, MIN_THROTTLE_DEADZONE = 0.05;
        int state = 0; //idle
        bool current_b = false, last_b_state = false, current_x = false, last_x_state = false, actuator_running = false, current_a = false, last_a_state = false,
        digging_running = false, dumping_running = false, dpad_right = false, dpad_left = false, last_dpad_right = false, last_dpad_left = false, teleopControl = false, 
        autoControl = false, autoState = false, last_auto_state = false, at_rest = true;
        
        
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
            autoState = joy_msg->buttons[9] && joy_msg->buttons[10];
            if ((autoState && !last_auto_state) && at_rest){
                auto msg = std_msgs::msg::String();
                if(state == 0){ //teleop
                    state = 1;
                    msg.data = "teleop";
                    teleopControl = true;
                    autoControl = false;
                }else if(state == 1){//idle
                    state = 2;
                    msg.data ="idle";
                    teleopControl = false;
                    autoControl = false;
                }
                else if(state == 2){//auto
                    state = 3;
                    msg.data="autonomy";
                    teleopControl = false;
                    autoControl = true;
                }else{
                    msg.data = "idle";
                    state = 0;
                    teleopControl = false;
                    autoControl = false;
                }
                
                cmd_state_pub->publish(msg);
            }
            last_auto_state = autoState;

            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = joy_msg->axes[1];
            twist.angular.z = joy_msg->axes[2];
            cmd_vel_drivebase_pub->publish(twist);


            double rightTrigger = joy_msg->axes[4];
            double leftTrigger = joy_msg->axes[5];
            double leadscrewSpeed;
            if(teleopControl){
                if((rightTrigger > -1 && rightTrigger <= 1) || (leftTrigger > -1 && leftTrigger <= 1)){
                    if (leftTrigger > MIN_THROTTLE_DEADZONE && rightTrigger > MIN_THROTTLE_DEADZONE) {
                        leadscrewSpeed = 0;
                    }
                    else if (leftTrigger > MIN_THROTTLE_DEADZONE){
                        leadscrewSpeed = -1*leftTrigger;
                    }
                    else if (rightTrigger > MIN_THROTTLE_DEADZONE){
                        leadscrewSpeed = rightTrigger;
                    }
                }else{
                    leadscrewSpeed = 0.0;
                }
                cmd_vel_leadscrew = std::clamp(leadscrewSpeed, -1.0, 1.0);
                auto msg = std_msgs::msg::Float64();
                msg.data = cmd_vel_leadscrew;
                cmd_vel_leadscrew_pub->publish(msg);
            
            
            current_b = joy_msg->buttons[1]; //actuator up
            if (current_b && !last_b_state) {
                if (actuator_running) {
                    cmd_vel_actuator = 0.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_actuator;
                    cmd_vel_actuator_pub->publish(msg);
                    actuator_running = false;
                } else {
                    cmd_vel_actuator = -1.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_actuator;
                    cmd_vel_actuator_pub->publish(msg);
                    actuator_running = true;
                }
            }
            last_b_state = current_b;

            current_x = joy_msg->buttons[2]; //actuator down
            if (current_x && !last_x_state){
                if (actuator_running) {
                    cmd_vel_actuator = 0.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_actuator;
                    cmd_vel_actuator_pub->publish(msg);
                    actuator_running = false;
                } else {
                    cmd_vel_actuator = 1.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_actuator;
                    cmd_vel_actuator_pub->publish(msg);
                    actuator_running = true;
                }
            }
            last_x_state = current_x;

            current_a = joy_msg->buttons[0]; //belt forward for digging
            if (current_a && !last_a_state){
                if (digging_running) {
                    cmd_vel_digging = 0.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_digging;
                    cmd_vel_digging_pub->publish(msg);
                    digging_running = false;
                } else {
                    cmd_vel_digging = 1.0;
                    auto msg = std_msgs::msg::Float64();
                    msg.data = cmd_vel_digging;
                    cmd_vel_digging_pub->publish(msg);
                    digging_running = true;
                }
            }
            last_a_state = current_a;

            dpad_right = joy_msg->buttons[14];
            if (dpad_right && !last_dpad_right) {
                auto msg = std_msgs::msg::Float64();
                if (dumping_running) {
                    cmd_vel_dumping = 0.0;
                    msg.data = cmd_vel_dumping;
                    cmd_vel_dumping_pub->publish(msg);
                    dumping_running = false;
                } else {
                    cmd_vel_dumping = 0.25;
                    msg.data = cmd_vel_dumping;
                    cmd_vel_dumping_pub->publish(msg);
                    dumping_running = true;
                }
            }
            last_dpad_right = dpad_right;

            
            dpad_left = joy_msg->buttons[13];
            if (dpad_left && !last_dpad_left) {
                auto msg = std_msgs::msg::Float64();
                if (dumping_running) {
                    cmd_vel_dumping = 0.0;
                    msg.data = cmd_vel_dumping;
                    cmd_vel_dumping_pub->publish(msg);
                    dumping_running = false;
                } else {
                    cmd_vel_dumping = -0.25;
                    msg.data = cmd_vel_dumping;
                    cmd_vel_dumping_pub->publish(msg);
                    dumping_running = true;
                }
            }
            last_dpad_left = dpad_left;
            }
        }
    };
    
    int main(int argc, char * argv[]) {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<controller>());
      rclcpp::shutdown();
      return 0;
    }
    