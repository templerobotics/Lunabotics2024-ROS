#include "core.hpp"


class Teleop_State_Manager : public rclcpp::Node{

public:
    Teleop_State_Manager() : Node("Teleop_State_Manager"){
    robot_state.robot_disabled = true;
    robot_state.manual_enabled = false;
    robot_state.outdoor_mode = false;
    robot_state.XBOX = true;
    robot_state.PS4 = false;
}

private:
    ROBOTSTATE_t robot_state;




}