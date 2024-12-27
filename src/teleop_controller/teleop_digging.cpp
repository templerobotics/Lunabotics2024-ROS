#include "core.hpp"

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
        void init_parameters(){}
        void apply_control_mode(){}



};