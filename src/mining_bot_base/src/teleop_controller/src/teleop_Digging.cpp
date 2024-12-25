#include "core.hpp"

/*
    Each teleop subsystem should (1) Declare and (2) Fetch current
    robot state parameters, and save their state in private variables.
    Do this so the subsystem retains the full current context of the robot in storage
    so that the robot behaves accordingly


*/


class Teleop_Digging : public rclcpp::Node {

    public:
        Teleop_Digging() : Node("Digging_Subsystem"),
        m_left_actuator("can0",5),
        m_right_actuator("can0",6),
        {
            init_parameters();
            apply_control_mode();
        }

    private:
        SparkMax m_left_actuator;
        SparkMax m_right_actuator;
        ROBOTSTATE_t robot_state; 
        XBOX_BUTTONS_t buttons;
        XBOX_JOYSTICK_INPUT_t joystick;

        void init_parameters(){
            robot_state.robot_disabled = this->get_parameter("robot_disabled").as_bool();    
        }

        void apply_control_mode(){

        }



}