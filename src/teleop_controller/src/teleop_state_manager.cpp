#include "core.hpp"

class Teleop_State_Manager : public rclcpp::Node {
public:
    Teleop_State_Manager() : Node("Teleop_State_Manager") {
    
        declare_parameters();
        init_param_event_subscriber();
        init_internal_state_from_params();

        ready_service = create_service<std_srvs::srv::Trigger>("state_manager_ready",std::bind(&Teleop_State_Manager::handle_ready_check, this,std::placeholders::_1, std::placeholders::_2));       
        param_service = create_service<teleop_controller::srv::SetParameter>("set_parameter",std::bind(&Teleop_State_Manager::handle_set_parameter, this,std::placeholders::_1, std::placeholders::_2));

        // parameter callback for param validation --> can try to find a way to incorporate robot_disabled if need be. But other nodes prob don't need that
        param_callback_handle = this->add_on_set_parameters_callback([this](const ParamVector &parameters) {
            return validate_parameters(parameters);
        });

        pub_robot_enabled = this->create_publisher<msg_Bool>("robot_state/robot_disabled", 10);
        pub_manual_enabled = this->create_publisher<msg_Bool>("robot_state/manual_enabled", 10);
        pub_xbox = this->create_publisher<msg_Bool>("robot_state/XBOX", 10);
        pub_ps4 = this->create_publisher<msg_Bool>("robot_state/PS4", 10);
        pub_outdoor_mode = this->create_publisher<msg_Bool>("robot_state/outdoor_mode", 10);
        
        timer = this->create_wall_timer(5000ms, std::bind(&Teleop_State_Manager::callback_publish_states, this));
    }

private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service;
    rclcpp::Service<teleop_controller::srv::SetParameter>::SharedPtr param_service;

    ROBOTSTATE_t robot_state;
    BoolPublisher pub_robot_enabled;
    BoolPublisher pub_manual_enabled;
    BoolPublisher pub_xbox;
    BoolPublisher pub_ps4;
    BoolPublisher pub_outdoor_mode;
    rclcpp::TimerBase::SharedPtr timer;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
    ParamEventHandler param_sub;


void handle_ready_check(const std::shared_ptr<std_srvs::srv::Trigger::Request>request,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    RCLCPP_INFO(get_logger(), "Received ready check request (pointer: %p)", static_cast<void*>(request.get()));
    response->success = true;
    response->message = "State Manager initialized and ready";
    RCLCPP_INFO(get_logger(), "State Manager ready service called");
}

void handle_set_parameter(const std::shared_ptr<teleop_controller::srv::SetParameter::Request> request,std::shared_ptr<teleop_controller::srv::SetParameter::Response> response){
    try {
        this->set_parameter(rclcpp::Parameter(request->param_name, request->new_value));
        // Update internal state
        if (request->param_name == "robot_disabled") {
            robot_state.robot_disabled = request->new_value;
        } else if (request->param_name == "XBOX") {
            robot_state.XBOX = request->new_value;
        } else if (request->param_name == "PS4") {
            robot_state.PS4 = request->new_value;
        } else if (request->param_name == "manual_enabled") {
            robot_state.manual_enabled = request->new_value;
        } else if (request->param_name == "outdoor_mode") {
            robot_state.outdoor_mode = request->new_value;
        }
        response->success = true;
        response->message = "Parameter set successfully";
        RCLCPP_INFO(get_logger(), "Parameter %s set to %s", 
                    request->param_name.c_str(), 
                    request->new_value ? "true" : "false");
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to set parameter: ") + e.what();
        RCLCPP_ERROR(get_logger(), "Failed to set parameter %s: %s", 
                    request->param_name.c_str(), e.what());
    }

}


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
   
    this->declare_parameter("XBOX", true, xbox_descriptor);
    this->declare_parameter("PS4", false, ps4_descriptor);
    this->declare_parameter("robot_disabled", true, robot_disabled_descriptor);
    this->declare_parameter("manual_enabled", true, manual_enabled_descriptor);
    this->declare_parameter("outdoor_mode", false, outdoor_mode_descriptor);
}

void init_internal_state_from_params() {
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
        }   
        
        // Validate manual mode constraints if needed
        // Add any other validation logic here
    }
    return result;
}



void callback_publish_states() {
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
    
}




};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop_State_Manager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}