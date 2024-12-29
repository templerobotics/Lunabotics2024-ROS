#include "core.hpp"

class Teleop_State_Manager : public rclcpp::Node {
public:
    Teleop_State_Manager() : Node("Teleop_State_Manager") {
    
        declare_parameters();
        init_param_event_subscriber();
        
        // Set up parameter callback for validation
        param_callback_handle = this->add_on_set_parameters_callback([this](const ParamVector &parameters) {
            return validate_parameters(parameters);
        });

        init_state_from_parameters();
        
        pub_robot_enabled = this->create_publisher<msg_Bool>("robot_state/robot_disabled", 10);
        pub_manual_enabled = this->create_publisher<msg_Bool>("robot_state/manual_enabled", 10);
        pub_xbox = this->create_publisher<msg_Bool>("robot_state/XBOX", 10);
        pub_ps4 = this->create_publisher<msg_Bool>("robot_state/PS4", 10);
        pub_outdoor_mode = this->create_publisher<msg_Bool>("robot_state/outdoor_mode", 10);
        
        timer = this->create_wall_timer(1000ms, std::bind(&Teleop_State_Manager::callback_publish_states, this));
    }

private:
    ROBOTSTATE_t robot_state;
    BoolPublisher pub_robot_enabled;
    BoolPublisher pub_manual_enabled;
    BoolPublisher pub_xbox;
    BoolPublisher pub_ps4;
    BoolPublisher pub_outdoor_mode;
    rclcpp::TimerBase::SharedPtr timer;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
    ParamEventHandler param_sub;

void init_param_event_subscriber(){
    
    // Subscribe to parameter events for actual state updates. Allows all params to be able to be triggered when set(), byassigning each param a callback() 
    param_sub = std::make_shared<rclcpp::ParameterEventHandler>(this);
    
    auto xbox_cb = [this](const rclcpp::Parameter & p) {
        robot_state.XBOX = p.as_bool();
        RCLCPP_INFO(get_logger(), "XBOX state updated to: %s", robot_state.XBOX ? "true" : "false");
    };
    param_sub->add_parameter_callback("XBOX", xbox_cb);
    
    
    auto ps4_cb = [this](const rclcpp::Parameter & p) {
        robot_state.PS4 = p.as_bool();
        RCLCPP_INFO(get_logger(), "PS4 state updated to: %s", robot_state.PS4 ? "true" : "false");
    };
    param_sub->add_parameter_callback("PS4", ps4_cb);
    
    
    auto robot_disabled_cb = [this](const rclcpp::Parameter & p) {
        robot_state.robot_disabled = p.as_bool();
        RCLCPP_INFO(get_logger(), "Robot disabled state updated to: %s", 
                    robot_state.robot_disabled ? "true" : "false");
    };
    param_sub->add_parameter_callback("robot_disabled", robot_disabled_cb);

    
    auto manual_enabled_cb = [this](const rclcpp::Parameter & p) {
        robot_state.manual_enabled = p.as_bool();
        RCLCPP_INFO(get_logger(), "Manual mode state updated to: %s", 
                    robot_state.manual_enabled ? "true" : "false");
    };
    param_sub->add_parameter_callback("manual_enabled", manual_enabled_cb);

    
    auto outdoor_mode_cb = [this](const rclcpp::Parameter & p) {
        robot_state.outdoor_mode = p.as_bool();
        RCLCPP_INFO(get_logger(), "Outdoor mode state updated to: %s", 
                    robot_state.outdoor_mode ? "true" : "false");
    };
    param_sub->add_parameter_callback("outdoor_mode", outdoor_mode_cb);
}

void declare_parameters() {
    ParamDescriptor xbox_descriptor;
    xbox_descriptor.description = "Enable/disable XBOX controller";
    xbox_descriptor.additional_constraints = "Only one controller can be active at a time";
    xbox_descriptor.read_only = false;
    
    ParamDescriptor ps4_descriptor;
    ps4_descriptor.description = "Enable/disable PS4 controller";
    ps4_descriptor.additional_constraints = "Only one controller can be active at a time";
    ps4_descriptor.read_only = false;
    
    ParamDescriptor robot_disabled_descriptor;
    robot_disabled_descriptor.description = "Enable/disable robot operation";
    robot_disabled_descriptor.additional_constraints = "Safety parameter - disables all robot operations";
    robot_disabled_descriptor.read_only = false;
    
    ParamDescriptor manual_enabled_descriptor;
    manual_enabled_descriptor.description = "Enable/disable manual control mode";
    manual_enabled_descriptor.additional_constraints = "Switches between manual and autonomous control";
    manual_enabled_descriptor.read_only = false;
    
    ParamDescriptor outdoor_mode_descriptor;
    outdoor_mode_descriptor.description = "Enable/disable outdoor operation mode";
    outdoor_mode_descriptor.additional_constraints = "Adjusts robot behavior for outdoor operation";
    outdoor_mode_descriptor.read_only = false;
    
    // Initialize each parameter to T/F respectively ---> In Drivebase these will be correctly set in control logic function
    this->declare_parameter("XBOX", true, xbox_descriptor);
    this->declare_parameter("PS4", false, ps4_descriptor);
    this->declare_parameter("robot_disabled", true, robot_disabled_descriptor);
    this->declare_parameter("manual_enabled", true, manual_enabled_descriptor);
    this->declare_parameter("outdoor_mode", false, outdoor_mode_descriptor);
}

void init_state_from_parameters() {
    robot_state.XBOX = this->get_parameter("XBOX").as_bool();
    robot_state.PS4 = this->get_parameter("PS4").as_bool();
    robot_state.manual_enabled = this->get_parameter("manual_enabled").as_bool();
    robot_state.outdoor_mode = this->get_parameter("outdoor_mode").as_bool();
    robot_state.robot_disabled = this->get_parameter("robot_disabled").as_bool();
}

rcl_interfaces::msg::SetParametersResult validate_parameters(const ParamVector &parameters) {
    
    SetParamsRes result;
    result.successful = true;
    
    /*robot HAS to enabled for anything to work ie -----> robot_disabled = FALSE for anything on the robot to function*/
    for (const auto &param : parameters) {
        if (param.get_name() == "XBOX" && param.as_bool() && this->get_parameter("PS4").as_bool()) {
            result.successful = false;
            result.reason = "Cannot enable XBOX while PS4 is active";
            return result;
        }
        else if (param.get_name() == "PS4" && param.as_bool() && this->get_parameter("XBOX").as_bool()) {
            result.successful = false;
            result.reason = "Cannot enable PS4 while XBOX is active";
            return result;
        }   //If set manual mode to T & the robot_disabled = T --> error
        
        // Validate manual mode constraints if needed
        // Add any other validation logic here
    }
    return result;
}

void callback_publish_states() {
    //if( robot_state_changed() ){
        auto msg = msg_Bool();
        msg.data = robot_state.robot_disabled;
        RCLCPP_INFO(this->get_logger(), "Publishing: ROBOT DISABLED [ %s ]", msg.data ? "true" : "false");
        pub_robot_enabled->publish(msg);

        msg.data = robot_state.manual_enabled;
        RCLCPP_INFO(this->get_logger(), "Publishing: MANUAL ENABLED [ %s ]", msg.data ? "true" : "false");
        pub_manual_enabled->publish(msg);

        msg.data = robot_state.outdoor_mode;
        RCLCPP_INFO(this->get_logger(), "Publishing: OUTDOOR MODE [ %s ]", msg.data ? "true" : "false");
        pub_outdoor_mode->publish(msg);

        msg.data = robot_state.XBOX;
        RCLCPP_INFO(this->get_logger(), "Publishing: XBOX [ %s]", msg.data ? "true" : "false");
        pub_xbox->publish(msg);

        msg.data = robot_state.PS4;
        RCLCPP_INFO(this->get_logger(), "Publishing: PS4 [ %s ]", msg.data ? "true" : "false");
        pub_ps4->publish(msg);
    //}
    
}

/*
bool robot_state_changed() {
    static ROBOTSTATE_t last_state = robot_state;
    bool changed = (robot_state != last_state);
    last_state = robot_state;
    return changed;
}

bool equal_states(ROBOTSTATE_t states1, ROBOT_STATE_t states2 ){
    for(bool state : states1){  if(state!=states2){ return false; } }
    return true;
}



*/
    
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop_State_Manager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}