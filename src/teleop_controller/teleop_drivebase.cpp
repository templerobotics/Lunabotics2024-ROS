#include "core.hpp"

class Teleop_Drivebase : public rclcpp::Node{

public:
    Teleop_Drivebase() : Node("Drivebase"),m_left_front("can0", 1),m_left_rear("can0", 2),m_right_front("can0", 3),m_right_rear("can0", 4),
    {
        init_parameters();
        apply_control_mode();
    }

private:
    XBOX_BUTTONS_t buttons;
    XBOX_JOYSTICK_INPUT_t joystick;
    ROBOTSTATE_t robot_state;

    SparkMax m_left_front;
    SparkMax m_left_rear;
    SparkMax m_right_front;
    SparkMax m_right_rear;

    TwistSubscription velocity_subscriber;
    JoySubscription joy_subscriber;

void init_parameters() {
    robot_state.outdoor_mode = this->get_parameter("outdoor_mode").as_bool();
    robot_state.XBOX = this->get_parameter("XBOX").as_bool();
    robot_state.PS4 = this->get_parameter("PS4").as_bool();
    robot_state.manual_enabled = this->get_parameter("manual_enabled").as_bool();  
    robot_state.outdoor_mode = this->get_parameter("outdoor_mode").as_bool();
    robot_state.robot_disabled = this->get_parameter("robot_disabled").as_bool();
}

/*Change this function to poll the current robot_state struct for values*/
void apply_control_mode() {
    bool xbox_enabled = this->get_parameter("XBOX").as_bool();
    bool ps4_enabled = this->get_parameter("PS4").as_bool();
    if( robot_state.robot_disabled == true){robot_state.robot_disabled = false;}
    if (xbox_enabled)
        RCLCPP_INFO(get_logger(), "XBOX MODE");
    else if (ps4_enabled)
        RCLCPP_INFO(get_logger(), "PS4 MODE");
    else
        RCLCPP_ERROR(get_logger(), "NO CONTROLLER SELECTED, CHECK LAUNCH PARAMETERS");
}

int get_button(const JoyMsg &joy_msg, const std::initializer_list<int> &mappings) {
    bool xbox_enabled = this->get_parameter("XBOX").as_bool();
    bool ps4_enabled = this->get_parameter("PS4").as_bool();
    size_t index = xbox_enabled ? 2 : ps4_enabled ? 1 : 0;
    return joy_msg->buttons[*(mappings.begin() + index)];
}

double get_axis(const JoyMsg &joy_msg, const std::initializer_list<int> &mappings) {
    bool xbox_enabled = this->get_parameter("XBOX").as_bool();
    bool ps4_enabled = this->get_parameter("PS4").as_bool();
    size_t index = xbox_enabled ? 2 : ps4_enabled ? 1 : 0;
    return joy_msg->axes[*(mappings.begin() + index)];
}

void control_robot() {
    if (buttons.home)
        robot_state.robot_disabled = true;
    robot_state.speed_multiplier = (buttons.x && buttons.y) ? 0.3 : (buttons.x ? 0.1 : 0.6);
    // Modify joystick triggers for better control
    joystick.right_trigger = (1.0 - joystick.right_trigger) / 2.0;
    joystick.left_trigger = (1.0 - joystick.left_trigger) / 2.0;

    if (joystick.right_trigger != 0.0) {
        robot_state.left_speed = joystick.right_trigger - joystick.left_x_axis;
        robot_state.right_speed = joystick.right_trigger + joystick.left_x_axis;
    }
    else if (joystick.left_trigger != 0.0) {
        robot_state.left_speed = -(joystick.left_trigger - joystick.left_x_axis);
        robot_state.right_speed = -(joystick.left_trigger + joystick.left_x_axis);
    }
    else {
        robot_state.left_speed = -joystick.left_x_axis;
        robot_state.right_speed = joystick.left_x_axis;
    }
    // Apply motor commands
    m_left_front.SetDutyCycle(std::clamp(robot_state.left_speed * robot_state.speed_multiplier, -1.0, 1.0));
    m_left_rear.SetDutyCycle(std::clamp(robot_state.left_speed * robot_state.speed_multiplier, -1.0, 1.0));
    m_right_front.SetDutyCycle(std::clamp(robot_state.right_speed * robot_state.speed_multiplier, -1.0, 1.0));
    m_right_rear.SetDutyCycle(std::clamp(robot_state.right_speed * robot_state.speed_multiplier, -1.0, 1.0));
}

void callback_joy(const JoyMsg joy_msg) {
    auto clock = rclcpp::Clock();

    buttons.share = get_button(joy_msg, {9, 8, 6});
    buttons.menu = get_button(joy_msg, {10, 9, 7});
    buttons.home = get_button(joy_msg, {11, 10, 8});
    buttons.a = get_button(joy_msg, {1, 0, 0});
    buttons.b = get_button(joy_msg, {0, 2, 1});
    buttons.x = get_button(joy_msg, {2, 3, 2});
    buttons.y = get_button(joy_msg, {3, 2, 3});
    buttons.left_bumper = get_button(joy_msg, {4, 4, 4});
    buttons.right_bumper = get_button(joy_msg, {5, 5, 5});

    if (buttons.share) {
        robot_state.manual_enabled = true;
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "MANUAL CONTROL:ENABLED");
    }

    if (buttons.menu) {
        robot_state.manual_enabled = false;
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "AUTONOMOUS CONTROL:ENABLED");
    }

    if (robot_state.manual_enabled) {
        // Update joystick states
        joystick.left_x_axis = joy_msg->axes[0];
        joystick.left_y_axis = joy_msg->axes[1];
        joystick.right_y_axis = joy_msg->axes[4];
        joystick.left_trigger = joy_msg->axes[2];
        joystick.right_trigger = joy_msg->axes[5];
        joystick.dpad_up = get_axis(joy_msg, {5, 7, 7});
        joystick.dpad_down = get_axis(joy_msg, {5, 7, 7});

        if (robot_state.robot_disabled) {
            RCLCPP_ERROR(get_logger(), "ROBOT DISABLED");
        } else {
            SparkMax::Heartbeat();
            control_robot();
        }
    }
}

void callback_velocity(const TwistMsg velocity_msg) {
    if (!robot_state .manual_enabled) {
        SparkMax::Heartbeat();
        double linear_velocity = velocity_msg->linear.x;
        double angular_velocity = velocity_msg->angular.z;
        double wheel_radius = robot_state.outdoor_mode ? 0.2 : 0.127;
        double wheel_distance = 0.5;

        double velocity_left_cmd = -0.1 * (linear_velocity - angular_velocity * wheel_distance / 2.0) / wheel_radius;
        double velocity_right_cmd = -0.1 * (linear_velocity + angular_velocity * wheel_distance / 2.0) / wheel_radius;

        m_left_front.SetDutyCycle(std::clamp(velocity_left_cmd, -1.0, 1.0));
        m_left_rear.SetDutyCycle(std::clamp(velocity_left_cmd, -1.0, 1.0));
        m_right_front.SetDutyCycle(std::clamp(velocity_right_cmd, -1.0, 1.0));
        m_right_rear.SetDutyCycle(std::clamp(velocity_right_cmd, -1.0, 1.0));
    }
}


};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop_Drivebase>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


