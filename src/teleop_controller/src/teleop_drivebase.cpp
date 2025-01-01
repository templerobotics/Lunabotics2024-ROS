#include "core.hpp"
/*  
            For Constructor initialization --> NEED HARDWARE SETUP TO TEST
        ,m_left_front("can0", 1),m_left_rear("can0", 2),m_right_front("can0", 3),m_right_rear("can0", 4)
*/

class Teleop_Drivebase : public rclcpp::Node{
public:
    Teleop_Drivebase() : Node("drivebase")
    {
        ready_client = create_client<std_srvs::srv::Trigger>("state_manager_ready");
        param_client = create_client<teleop_controller::srv::SetParameter>("set_parameter");
        wait_for_state_manager();                               
        prep_robot();


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

        timer = create_wall_timer(5s, std::bind(&Teleop_Drivebase::callback_motor_heartbeat, this)); 
        cmd_vel_sub = create_subscription<Twist>("cmd_vel", 10, std::bind(&Teleop_Drivebase::callback_cmd_vel, this, std::placeholders::_1) );
        joy_sub = create_subscription<Joy>("joy", 10, std::bind(&Teleop_Drivebase::callback_joy, this, std::placeholders::_1) );

    }
    
private:

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ready_client;
    rclcpp::Client<teleop_controller::srv::SetParameter>::SharedPtr param_client;

    
    ROBOT_ACTUATION_t robot_actuation;
    XBOX_JOYSTICK_INPUT_t xbox_input;
    ROBOTSTATE_t robot_state;
    ROBOT_LIMITS_t robot_dimensions; //Determine if I want to use robot_limits
    
    /*
    
    Might have to make MOTOR CONTROLLER GROUP a class honestly
    
    MOTOR_CONTROLLER_GROUP_t left_motors;
    MOTOR_CONTROLLER_GROUP_t right_motors;
    SparkMax left;
    SparkMax left2;
    left_motors(left,left2);
    left_motors.LEFT_SIDE()
    
    */

    /*
    SparkMax m_left_front;
    SparkMax m_left_rear;
    SparkMax m_right_front;
    SparkMax m_right_rear;
    */

    BoolSubscriber sub_xbox;
    BoolSubscriber sub_robot_enabled;
    BoolSubscriber sub_manual_enabled_enabled;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::TimerBase::SharedPtr timer;

    TwistSubscription velocity_subscriber;
    JoySubscription joy_sub;


void wait_for_state_manager() {
    while (!ready_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for state manager.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Waiting for state manager to be ready...");
    }
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = ready_client->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(get_logger(), "State Manager is ready. Message: %s", 
                        response->message.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to confirm State Manager readiness");
        }
    }
}

void set_param(const std::string& param_name, bool new_val) {
    if (!param_client->wait_for_service(1s)) {
        RCLCPP_ERROR(get_logger(), "Parameter service not available");
        return;
    }
    
    auto request = std::make_shared<teleop_controller::srv::SetParameter::Request>();
    request->param_name = param_name;
    request->new_value = new_val;
    
    auto future = param_client->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(get_logger(), "Set parameter %s: %s", 
                        param_name.c_str(), response->message.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter %s: %s", 
                        param_name.c_str(), response->message.c_str());
        }
    }
}


void prep_robot() {
    RCLCPP_INFO(get_logger(), "Initializing robot parameters...");
    set_param("robot_disabled", false);
    set_param("XBOX", true);
    set_param("PS4", false);
    set_param("manual_enabled", true);
    set_param("outdoor_mode", false);
    RCLCPP_INFO(get_logger(), "Robot parameters initialized");
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


void control_robot(){
    if(xbox_input.emergency_stop_button){ set_param("robot_disabled",true) ; robot_state.robot_disabled = true;}
    robot_actuation.speed_lift_actuator = (xbox_input.dpad_vertical == 1.0) ? -0.3 : (xbox_input.dpad_vertical == -1.0) ? 0.3 : 0.0;
    robot_actuation.speed_tilt_actuator = (xbox_input.secondary_vertical_input > 0.1) ? -0.3 : (xbox_input.secondary_vertical_input < -0.1) ? 0.3 : 0.0;
    robot_actuation.velocity_scaling = (xbox_input.x_button && xbox_input.y_button) ? 0.3 : (xbox_input.x_button ? 0.1 : 0.6);

    xbox_input.throttle_forward = (1.0 - xbox_input.throttle_forward) / 2.0;
    xbox_input.throttle_backwards = (1.0 - xbox_input.throttle_backwards) / 2.0;

    if (xbox_input.throttle_forward != 0.0){
        robot_actuation.wheel_speed_left = xbox_input.throttle_forward - xbox_input.joystick_turn_input;
        robot_actuation.wheel_speed_right = xbox_input.throttle_forward + xbox_input.joystick_turn_input;

    }else if(xbox_input.throttle_backwards != 0.0){
        robot_actuation.wheel_speed_left = -(xbox_input.throttle_backwards - xbox_input.joystick_turn_input);
        robot_actuation.wheel_speed_right = -(xbox_input.throttle_backwards + xbox_input.joystick_turn_input);
    }else{
        robot_actuation.wheel_speed_left = -xbox_input.joystick_turn_input;
        robot_actuation.wheel_speed_right = xbox_input.joystick_turn_input;
    }

    /*
    left_wheel_motor_.SetDutyCycle(std::clamp(left_speed_ * speed_multiplier_, -1.0, 1.0));
    right_wheel_motor_.SetDutyCycle(std::clamp(right_speed_ * speed_multiplier_, -1.0, 1.0));
    lift_actuator_motor_.SetDutyCycle(std::clamp(lift_actuator_speed_, -1.0, 1.0));
    tilt_actuator_left_motor_.SetDutyCycle(std::clamp(tilt_actuator_speed_, -1.0, 1.0));
    tilt_actuator_right_motor_.SetDutyCycle(std::clamp(tilt_actuator_speed_, -1.0, 1.0));
    */
    
}

void callback_joy(const JoyMsg msg){
    if(robot_state.robot_disabled == true || robot_state.manual_enabled==false || robot_state.PS4==true || robot_state.XBOX==false){
        RCLCPP_ERROR(get_logger(),"Error in Joy Callback. Robot must be fully enabled correctly to function!\nExiting...");
        std::exit(1);
    }

    auto clock = rclcpp::Clock();
    printf("INSIDE JOY CALLBACK!");

    try{
        xbox_input.manual_mode_button = get_button(msg, {9, 8, 6});
        xbox_input.autonomous_mode_button = get_button(msg, {10, 9, 7});
        xbox_input.emergency_stop_button = get_button(msg, {11, 10, 8});
        xbox_input.a_button = get_button(msg, {1, 0, 0});
        xbox_input.b_button = get_button(msg, {0, 2, 1});
        xbox_input.x_button = get_button(msg, {2, 3, 2});
        xbox_input.y_button = get_button(msg, {3, 2, 3});
        xbox_input.left_bumper = get_button(msg, {4, 4, 4});
        xbox_input.right_bumper = get_button(msg, {5, 5, 5});

        xbox_input.dpad_horizontal = get_axis(msg, {4, 6, 6});
        xbox_input.dpad_vertical = get_axis(msg, {5, 7, 7});

        if(xbox_input.manual_mode_button){ 
            if(robot_state.manual_enabled != true){ 
                set_param("manual_enabled",true);//set in state manager
                robot_state.manual_enabled = true;//set locally
            }
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "MANUAL CONTROL:ENABLED");
        }
        if(xbox_input.autonomous_mode_button){ 
            set_param("manual_enabled",false);
            robot_state.manual_enabled = false;
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "AUTONOMOUS CONTROL:ENABLED");
        }
        
        rclcpp::Parameter param = get_parameter("manual_enabled");
        bool current_state_manual = param.as_bool();
        if(current_state_manual == true){
            
            xbox_input.joystick_turn_input = msg->axes[0];
            xbox_input.joystick_forward_input = msg->axes[1];
            xbox_input.secondary_vertical_input = msg->axes[4];
            xbox_input.throttle_backwards = msg->axes[2];
            xbox_input.throttle_forward = msg->axes[5];
            if (robot_state.robot_disabled == true){
                RCLCPP_ERROR(get_logger(), "ROBOT IS CURRENTLY DISABLED");
            }
            else
            {
                SparkMax::Heartbeat(); //ONLY TRY WITH REAL HARDWARE
            }

            control_robot(); //teleoperation
        }

    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(),"Error in JOY Callback: %s", e.what());
    }

}

// Note:Make init function for robot_dimensions struct variables as well or URDF 
void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
    if(robot_state.robot_disabled == true || robot_state.manual_enabled==false || robot_state.PS4==true || robot_state.XBOX==false){
        RCLCPP_ERROR(get_logger(),"Error in CMD VEL Callback. Robot must be fully enabled correctly to function!\nExiting...");
        std::exit(1);
    }
    RCLCPP_INFO(get_logger(),"INSIDE CMD VEL CALLBACK %lf",msg->linear.x);
   
   try{
        rclcpp::Parameter param = get_parameter("manual_enabled");
        bool current_state_manual = param.as_bool();
        if(robot_state.manual_enabled == false || current_state_manual == false){ // autonomous
            SparkMax::Heartbeat();//NEED HARDWARE FOR THIS TO WORK
            /*
            replace with struct variables
            
            double linear_velocity = velocity_msg->linear.x;
            double angular_velocity = velocity_msg->angular.z;
            robot_dimensions.wheel_radius = robot_state.outdoor_mode ? 0.2 : 0.127;
            double wheel_distance = 0.5;
            
            left_wheel_motor_.SetDutyCycle(std::clamp(velocity_left_cmd, -1.0, 1.0));
            right_wheel_motor_.SetDutyCycle(std::clamp(velocity_right_cmd, -1.0, 1.0));
            */
        }
    }
    catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(),"Error in cmdVelCallback: %s", e.what());
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


