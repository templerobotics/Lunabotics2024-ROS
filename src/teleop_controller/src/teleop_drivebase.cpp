#include "core.hpp"

class Teleop_Drivebase : public rclcpp::Node{
public:
    Teleop_Drivebase() : Node("drivebase")
    {
        /*  Put this after drivebase node above when testing HARDWARE 
            ,m_left_front("can0", 1),m_left_rear("can0", 2),m_right_front("can0", 3),m_right_rear("can0", 4)
        */
        param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/Teleop_State_Manager");// Create client to modify state manager parameters 
        while (!param_client->wait_for_service(1s)) { // Wait for state manager to be available. If not ready, error
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for state manager.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Waiting for state manager...");
        }


        sub_xbox = create_subscription<msg_Bool>("robot_state/XBOX", 10,std::bind(&Teleop_Drivebase::callback_xbox, this, std::placeholders::_1)); 
        sub_robot_enabled = create_subscription<msg_Bool>("robot_state/enabled", 10,std::bind(&Teleop_Drivebase::callback_robot_enabled, this, std::placeholders::_1));
        sub_manual_enabled_enabled = create_subscription<msg_Bool>("robot_state/manual_enabled", 10,std::bind(&Teleop_Drivebase::callback_manual_enabled, this, std::placeholders::_1));
        
        /*
        CANT RUN WITHOUT PHYSICAL HARDWARE SETUP RUNNING AS WELL

        config_motor(m_left_front);
        config_motor(m_left_rear);
        config_motor(m_right_front);
        config_motor(m_right_front);
        m_right_front.SetInverted(true);   ---->  Diff drive robot
        m_right_rear.SetInverted(true);    ----> Diff drive robot
        */

        /* Make this 0.1ms when testing on actual hardware = 10HZ = 10x per second */
        timer = create_wall_timer(1s, std::bind(&Teleop_Drivebase::callback_motor_heartbeat, this));
        cmd_vel_sub = create_subscription<Twist>("cmd_vel", 10, std::bind(&Teleop_Drivebase::callback_cmd_vel, this, std::placeholders::_1));
        //TODO: Load physical robot parameters from URDF file OR make them parameters in TELEOP_STATE MANAGER
        //              . As for right now, make them hard-coded values until further runtime parameter testing
    
    }
    
private:

    //Should be fine to have in every class. Focus is specific states being true/false for drivebase, mining, dumping to check before utilizing XBOX controls
    XBOX_BUTTONS_t buttons;
    XBOX_JOYSTICK_INPUT_t joystick;

    ROBOTSTATE_t robot_state;
    
    //SparkMax m_left_front;
    //SparkMax m_left_rear;
    //SparkMax m_right_front;
    //SparkMax m_right_rear;

    std::shared_ptr<rclcpp::SyncParametersClient> param_client;
    BoolSubscriber sub_xbox;
    BoolSubscriber sub_robot_enabled;
    BoolSubscriber sub_manual_enabled_enabled;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::TimerBase::SharedPtr timer;
    
    const ParamVector all_params;//holds fetched parameters. 1-time fetch?

    TwistSubscription velocity_subscriber;
    JoySubscription joy_subscriber;


rcl_interfaces::msg::Parameter create_bool_parameter(const std::string &name, bool value) {
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = name;
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    parameter.value.bool_value = value;
    return parameter;
}

/*param is type msg_Bool --> but upon calling this function, I can cast to string if need be. Ideally keep as bool though*/
void set_parameter(const std::string &param_name, bool new_value){
    ParamVector parameters;
    parameters.push_back(rclcpp::Parameter(param_name, new_value));
    param_client->set_parameters(parameters);
    
}

//  callback calls this function 
//  lets say XBOX is true, what do we do to set XBOX to false AND send that val to state manager to be consistently published?
//  since we're subscribing, sending a value back upstream to state manager is not possible, so NEED a service?
//IM DUMB --> CONVERT THE DATA INSIDE THE PARAM aka ".data" field
//I DELETED "const" --> ideally keep the const, but in order to bypass it would need to use PTR(*) and not Ref(&) --> doing too much w/ that. SO just remove "const" probably best bet
void set_param(msg_Bool &current_param_val, bool new_val){
    if( !param_client->has_parameter(std::to_string(current_param_val.data))){
        RCLCPP_ERROR(get_logger(),"ERROR! State Manager node does NOT contain param: [%s]", std::to_string(current_param_val.data));
    }
    current_param_val.data = new_val; //DELETE THIS LINE, TEMPORARY SO WE CAN COLCON BUILD SUCCESSFULLY. BELOW IS CORRECT FUNCTION CALL
    //param_client->set_parameters_atomically()
}

void handle_response(rclcpp::Client<rcl_interfaces::srv::SetParameters>::Future future){
     // Wait for the response
        rclcpp::spin_until_future_complete(get_node_base_interface(), future);
        if (future.valid()) {
            const auto &response = future.get();
            for (const auto &result : response->results) {
                if (result.successful) {
                    RCLCPP_INFO(get_logger(), "Successfully set parameter '%d'.", result.successful);
                } else {
                    RCLCPP_ERROR(get_logger(), "Failed to set parameter '%d': %s", result.successful, result.reason.c_str());
                }
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to receive response from SetParameters service.");
        }
    }


void callback_xbox(const msg_Bool &state_XBOX){
    robot_state.XBOX = state_XBOX.data;
    RCLCPP_INFO(get_logger(), "XBOX Controller State: %s", robot_state.XBOX ? "true" : "false");
    if (robot_state.XBOX) {
        RCLCPP_INFO(get_logger(), "XBOX Controller is Active, enabling drive operations");

    } else {
        RCLCPP_INFO(get_logger(), "XBOX Controller is Inactive, disabling drive operations");
    }

}

void callback_robot_enabled(const msg_Bool &state_robot_disabled){
    robot_state.robot_disabled = state_robot_disabled.data;
    RCLCPP_INFO(get_logger(), "Robot Disabled State:[%s]", robot_state.robot_disabled ? "true" : "false");
    if (robot_state.robot_disabled == false) {
        RCLCPP_INFO(get_logger(), "Robot is ACTIVE, enabling drive operations");
    } else {
        RCLCPP_INFO(get_logger(), "Robot is INACTIVE, disabling drive operations");
    }
    
}

void callback_manual_enabled(const msg_Bool &state_manual_enabled){
    robot_state.manual_enabled = state_manual_enabled.data;
    RCLCPP_INFO(get_logger(), "Robot Manual Enabled State: %s", robot_state.manual_enabled ? "true" : "false");
    if (robot_state.manual_enabled == true) {
        RCLCPP_INFO(get_logger(), "Robot Manual Enabled IS [%s], enabling drive operations",robot_state.manual_enabled ? "true" : "false");
    } else {
        RCLCPP_INFO(get_logger(), "Robot Manual Enabled is [%s], disabling drive operations",robot_state.manual_enabled ? "true" : "false");
    }
}
void callback_motor_heartbeat(){
    /*
        Can't test this without physical hardware, since there's 
        no CAN protocol info being transmitted without hardware
    try{
        m_left_front.Heartbeat();
        m_left_rear.Heartbeat();
        m_right_front.Heartbeat();
        m_right_rear.Heartbeat();
    }catch(const std::exception &e){
        std::cerr << "Motor Heartbeat Error: " << e.what() << std::endl;
    }
    */
      RCLCPP_INFO(get_logger(), "Current States - XBOX: %s, Manual Enabled: %s, Robot Disabled: %s",
        robot_state.XBOX ? "true" : "false",
        robot_state.manual_enabled ? "true" : "false",
        robot_state.robot_disabled ? "true" : "false");
}

//TODO: Implement this function. Code below is temporary to avoid warnings in building
void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
    double ang_z = msg.get()->angular.z;
    double total = 5 + ang_z;
    RCLCPP_INFO(get_logger(),"TOTAL = %lf",total);
}


void config_motor(SparkMax& motor){
    motor.SetIdleMode(IdleMode::kBrake);
    motor.SetMotorType(MotorType::kBrushless);
    motor.BurnFlash();
}

int get_button(const JoyMsg &joy_msg, const std::initializer_list<int> &mappings) {
    bool xbox_enabled = get_parameter("XBOX").as_bool();
    bool ps4_enabled = get_parameter("PS4").as_bool();
    size_t index = xbox_enabled ? 2 : ps4_enabled ? 1 : 0;
    return joy_msg->buttons[*(mappings.begin() + index)];
}

double get_axis(const JoyMsg &joy_msg, const std::initializer_list<int> &mappings) {
    bool xbox_enabled = get_parameter("XBOX").as_bool();
    bool ps4_enabled = get_parameter("PS4").as_bool();
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


