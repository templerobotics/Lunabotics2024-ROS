#include "core.hpp"

class Teleop_Drivebase : public rclcpp::Node{
public:
    Teleop_Drivebase() : Node("drivebase"),m_left_front("can0", 1),m_left_rear("can0", 2),m_right_front("can0", 3),m_right_rear("can0", 4),
    {
       
        param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/Teleop_State_Manager");// Create client to modify state manager parameters 
        
        while (!param_client->wait_for_service(1s)) { // Wait for state manager to be available. If not ready, error
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for state manager.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for state manager...");
        }


        sub_xbox = this->create_subscription<msg_Bool>("robot_state/XBOX", 10,std::bind(&Teleop_Drivebase::callback_xbox, this, std::placeholders::_1)); 
        sub_robot_enabled = this->create_subscription<msg_Bool>("robot_state/enabled", 10,std::bind(&Teleop_Drivebase::callback_robot_enabled, this, std::placeholders::_1));
        sub_manual_mode_enabled = this->create_subscription<msg_Bool>("robot_state/manual_mode", 10,std::bind(&Teleop_Drivebase::callback_manual_mode, this, std::placeholders::_1));
        
        config_motor(m_left_front);
        config_motor(m_left_rear);
        config_motor(m_right_front);
        config_motor(m_right_front);
        m_right_front.SetInverted(true);    // Diff drive robot
        m_right_rear.SetInverted(true);     // Diff drive robot

        timer = this->create_wall_timer(0.1ms, std::bind(&Teleop_Drivebase::callback_motor_heartbeat, this));
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Teleop_Drivebase::callback_cmd_vel, this, std::placeholders::_1));

        //TODO: Load physical robot parameters from URDF file OR make them parameters. As for right now, make them hard-coded values until further runtime parameter testing
    }

private:
    SetParamClient param_client;
    
    BoolSubscriber sub_xbox;
    BoolSubscriber sub_robot_enabled;
    BoolSubscriber sub_manual_mode_enabled;
    TwistSubscription cmd_vel_sub;
    rclcpp::TimerBase::SharedPtr timer;
    
    const ParamVector all_params;//holds fetched parameters. 1-time fetch?

    //Should be fine to have in every class. Focus is specific states being true/false for drivebase, mining, dumping to check before utilizing XBOX controls
    XBOX_BUTTONS_t buttons;
    XBOX_JOYSTICK_INPUT_t joystick;

    ROBOTSTATE_t robot_state;
    SparkMax m_left_front;
    SparkMax m_left_rear;
    SparkMax m_right_front;
    SparkMax m_right_rear;
    TwistSubscription velocity_subscriber;
    JoySubscription joy_subscriber;

/*
    TODO: "Action Nodes" : Drivebase, Digging, Dumping --> Handle their own responses & send requests to SET() parameters at runtime
    Overall Goal.Revise as needed
    if state is T/F then do something --> SET NEW PARAMETER STATE VALUES USING SERVICE REQUEST --> Make sure that the state is RETAINED in state_manager node fields
*/

//TODO: Poll current PUBLISHED Parameters & fetch() and store them. Needs to function at ANY time. Subscribing should work 
void fetch_store_current_parameters() {
    all_params = param_client.get( {"XBOX","PS4","manual_mode","robot_disabled"} );
    for(const auto &param : all_params){
        if(param.get_name().compare("XBOX")==0 && param_client->service_is_ready()){
           
        }
    }

}


rcl_interfaces::msg::Parameter create_bool_parameter(const std::string &name, bool value) {
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = name;
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    parameter.value.bool_value = value;
    return parameter;
}

void set_parameter(const std::string &param_name, bool new_value){
    
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(create_bool_parameter(param_name, new_value));
    auto future = param_client_->async_send_request(request);
    this->handle_response(future);

}

void handle_response(rclcpp::Client<rcl_interfaces::srv::SetParameters>::Future future){
     // Wait for the response
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        if (future.valid()) {
            const auto &response = future.get();
            for (const auto &result : response.results) {
                if (result.successful) {
                    RCLCPP_INFO(this->get_logger(), "Successfully set parameter '%s'.", result.name.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set parameter '%s': %s", result.name.c_str(), result.reason.c_str());
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive response from SetParameters service.");
        }
    }
}

//What we "hear" from the topic is of type msg_Bool
void callback_xbox(const msg_Bool &state_XBOX){
    robot_state.XB0X = state_XBOX.data;
    RCLCPP_INFO(this->get_logger(), "XBOX state: %s", robot_state.XBOX ? "true" : "false");
    if (robot_state.XBOX) {
        RCLCPP_INFO(this->get_logger(), "XBOX controller active, enabling drive operations");

    } else {
        RCLCPP_INFO(this->get_logger(), "XBOX controller inactive, disabling drive operations");
    }

}

void callback_robot_enabled(const msg_Bool &state_robot_disabled){
    robot_state.robot_disabled = state_robot_disabled.data;
    RCLCPP_INFO(this->get_logger(), "Robot state: %s", robot_state.robot_disabled ? "true" : "false");
    if (robot_state.robot_disabled == false) {
        RCLCPP_INFO(this->get_logger(), "Robot is ACTIVE, enabling drive operations");
    } else {
        RCLCPP_INFO(this->get_logger(), "Robot is INACTIVE, disabling drive operations");
    }
}

void callback_manual_mode(const msg_Bool &state_manual_enabled){
    robot_state.manual_enabled = state_manual_enabled.data;
    RCLCPP_INFO(this->get_logger(), "Robot state: %s", robot_state.manual_enabled ? "true" : "false");
    if (robot_state.manual_enabled == true) {
        RCLCPP_INFO(this->get_logger(), "Robot MANUAL MODE IS TRUE, enabling drive operations");
    } else {
        RCLCPP_INFO(this->get_logger(), "Robot MANUAL MODE is FALSE, disabling drive operations");
    }
}
void callback_motor_heartbeat(){
    try{
        m_left_front.Heartbeat();
        m_left_back.Heartbeat();
        m_right_front.Heartbeat();
        m_right_back.Heartbeat();
    }catch(const std::exception &e){
        std::cerr << "Motor Heartbeat Error: " << e.what() << std::endl;
        return -1;
    }
}

void callback_cmd_vel(){

}


void config_motor(SparkMax& motor){
    motor.SetIdleMode(IdleMode::kBrake);
    motor.SetMotorType(MotorType::kBrushless);
    motor.BurnFlash();
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








};




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop_Drivebase>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


