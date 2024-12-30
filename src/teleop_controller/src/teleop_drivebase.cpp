#include "core.hpp"

class Teleop_Drivebase : public rclcpp::Node{
public:
    Teleop_Drivebase() : Node("drivebase")
    {
        /*  
            For Constructor initialization --> NEED HARDWARE SETUP TO TEST
        ,m_left_front("can0", 1),m_left_rear("can0", 2),m_right_front("can0", 3),m_right_rear("can0", 4)
        */
        param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/Teleop_State_Manager");// Create client to modify state manager params at runtime, while this node spins 
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
        CAN'T RUN CODE PERTAINING TO PHYSICAL HARDWARE WITHOUT PHYSICAL HARDWARE ACTUALLY SETUP 
        config_motor(m_left_front);
        config_motor(m_left_rear);
        config_motor(m_right_front);
        config_motor(m_right_front);
        m_right_front.SetInverted(true);   ---->  Diff drive robot
        m_right_rear.SetInverted(true);    ----> Diff drive robot
        */

        timer = create_wall_timer(5s, std::bind(&Teleop_Drivebase::callback_motor_heartbeat, this));// Make timer tick 0.1ms for actual hardware: 10hZ = 10x per second 
        prep_robot();
        cmd_vel_sub = create_subscription<Twist>("cmd_vel", 10, std::bind(&Teleop_Drivebase::callback_cmd_vel_control_logic, this, std::placeholders::_1));
        
    }
    
private:
    //Validate states --> Utilize XBOX controls in control logic functions
    XBOX_BUTTONS_t buttons;
    XBOX_JOYSTICK_INPUT_t joystick;
    ROBOTSTATE_t robot_state;
    
    /*
    SparkMax m_left_front;
    SparkMax m_left_rear;
    SparkMax m_right_front;
    SparkMax m_right_rear;
    */

    std::shared_ptr<rclcpp::SyncParametersClient> param_client;


    
    BoolSubscriber sub_xbox;
    BoolSubscriber sub_robot_enabled;
    BoolSubscriber sub_manual_enabled_enabled;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::TimerBase::SharedPtr timer;

    TwistSubscription velocity_subscriber;
    JoySubscription joy_subscriber;


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
        Note:Can't test this without physical hardware, since there's no CAN protocol info being transmitted without hardware
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

void set_param(const std::string &param_name, bool new_val) {
    if (!param_client->has_parameter(param_name)) {
        RCLCPP_ERROR(get_logger(), 
            "State Manager node does not contain parameter: [%s]. Attempted value: [%s]",
            param_name.c_str(), new_val ? "true" : "false");
        return;
    }
    
    std::vector<rclcpp::Parameter> parameters;
    parameters.emplace_back(rclcpp::Parameter(param_name, new_val));
    
    try {
        auto result = param_client->set_parameters_atomically(parameters);
        if (result.successful) {
            RCLCPP_INFO(get_logger(), "Successfully set parameter %s", param_name.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter %s: %s", 
                        param_name.c_str(), result.reason.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error setting parameter: %s", e.what());
    }
    
}

//Init robot params prior to executing control logic
void prep_robot(){
    set_param("robot_disabled", false);  
    set_param("XBOX", true);            
    set_param("PS4", false);           
    set_param("manual_enabled", true);   
    set_param("outdoor_mode", false); 

    RCLCPP_INFO(get_logger(), "Initializing robot parameters...");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    RCLCPP_INFO(get_logger(), "Robot parameters initialized: XBOX=true, PS4=false, robot_disabled=false, manual_enabled=true");
}


void callback_cmd_vel_control_logic(const geometry_msgs::msg::Twist::SharedPtr msg){
    if(robot_state.robot_disabled == true || robot_state.manual_enabled==false || robot_state.PS4==true || robot_state.XBOX==false){
        std::cout << "ERROR! Robot must be FULLY ENABLED CORRECTLY in order to execute control logic!" << std::endl; 
    }
    printf("INSIDE CMD VEL CALLBACK!\nLINEAR VELOCITY = [%.2lf]\n",msg->linear.x);

}





};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop_Drivebase>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


