/*  
    Make sure I load the publishing data into the publisher.
    Look at the ROS2 Publisher C++ Documentation

    Note:: Could I declare RobotState_t robot_state variable in the header file
    AND all the different bools as static, then use those globals from core.hpp in 
    all of my action nodes? Make it static so that state changes are retained until 
    the nodes stop?
    Answer: Don't do the above statement, it's bad practice
    Answer: Try Behavior Trees --> Grayson using behavior trees for global planning


    TODO
        Set the actual parameter server values in constructor, not just the topic names
*/
#include "core.hpp"


class Teleop_State_Manager : public rclcpp::Node{

public:
    Teleop_State_Manager() : Node("Teleop_State_Manager"){
    robot_state.robot_disabled = true;
    robot_state.manual_enabled = false;
    robot_state.outdoor_mode = false;
    robot_state.XBOX = true;
    robot_state.PS4 = false;

    pub_robot_enabled = this->create_publisher<std_msgs::msg::Bool>("robot_state/enabled", 10);
    pub_manual_mode = this->create_publisher<std_msgs::msg::Bool>("robot_state/manual_mode", 10);
    pub_xbox = this->create_publisher<std_msgs::msg::Bool>("robot_state/XBOX", 10);
    pub_ps4 = this->create_publisher<std_msgs::msg::Bool>("robot_state/PS4", 10);
    pub_outdoor_mode = this->create_publisher<std_msgs::msg::Bool>("robot_state/outdoor_mode", 10);
    timer = this->create_wall_timer(500ms, std::bind(&Teleop_State_Manager::callback_publish_states, this));
}

private:
    ROBOTSTATE_t robot_state;
    BoolPublisher pub_robot_enabled;
    BoolPublisher pub_manual_mode;
    BoolPublisher pub_xbox;
    BoolPublisher pub_ps4;
    BoolPublisher pub_outdoor_mode;
    rclcpp::TimerBase::SharedPtr timer;

void init_parameters() {
    this->declare_parameter("XBOX", true);
    this->declare_parameter("PS4", false);
    this->declare_parameter("manual_enabled", true);
    this->declare_parameter("outdoor_mode", false);
    this->declare_parameter("robot_disabled",false);

    robot_state.outdoor_mode = this->get_parameter("outdoor_mode").as_bool();
    robot_state.XBOX = this->get_parameter("XBOX").as_bool();
    robot_state.PS4 = this->get_parameter("PS4").as_bool();
    robot_state.manual_enabled = this->get_parameter("manual_enabled").as_bool();  
    robot_state.outdoor_mode = this->get_parameter("outdoor_mode").as_bool();
    robot_state.robot_disabled = this->get_parameter("robot_disabled").as_bool();
}
//Publishers publish continuously while node spins, so yes, I do need all the publishers
void callback_publish_states() {
    auto msg = std_msgs::msg::Bool();
    msg.data = robot_state.robot_disabled;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    pub_robot_enabled->publish(msg);

    msg.data = robot_state.robot_disabled;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    pub_robot_enabled->publish(msg);
    
}
    


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop_StateManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


}