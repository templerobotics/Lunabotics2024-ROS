#include "core.hpp"

class Teleop_Control : public rclcpp::Node {
public:
    Teleop_Control() : Node("Teleop_Control")    
        , m_left_front("can0", 1)
        , m_left_rear("can0", 2)
        , m_right_front("can0", 3)
        , m_right_rear("can0", 4)
        , m_belt_left("can0", 5)
        , m_belt_right("can0", 6)
        , m_leadscrew_left("can0", 7)
        , m_leadscrew_right("can0", 8)
        , m_dump_conveyor("can0", 9)
        , m_dump_latch("can0", 10)
        , left_motors(m_left_front, m_left_rear)
        , right_motors(m_right_front, m_right_rear)
    {
        
        ready_client = create_client<std_srvs::srv::Trigger>("state_manager_ready");
        param_client = create_client<teleop_controller::srv::SetParameter>("set_parameter");
        wait_for_state_manager();                               
        prep_robot();

        sub_xbox = create_subscription<msg_Bool>("robot_state/XBOX", 10,std::bind(&Teleop_Control::callback_xbox, this, std::placeholders::_1));
        sub_robot_enabled = create_subscription<msg_Bool>("robot_state/enabled", 10,std::bind(&Teleop_Control::callback_robot_enabled, this, std::placeholders::_1));
        sub_manual_enabled = create_subscription<msg_Bool>("robot_state/manual_enabled", 10,std::bind(&Teleop_Control::callback_manual_enabled, this, std::placeholders::_1));
        sub_outdoor_mode = create_subscription<msg_Bool>("robot_state/outdoor_mode", 10,std::bind(&Teleop_Control::callback_outdoor_mode, this, std::placeholders::_1));
        sub_ps4 = create_subscription<msg_Bool>("robot_state/PS4", 10,std::bind(&Teleop_Control::callback_ps4, this, std::placeholders::_1));
        #ifdef HARDWARE_ENABLED
        configure_all_motors();
        #endif

        timer = create_wall_timer(5s, std::bind(&Teleop_Control::callback_motor_heartbeat, this));
        velocity_subscriber = create_subscription<Twist>("cmd_vel", 10, std::bind(&Teleop_Control::callback_cmd_vel, this, std::placeholders::_1));
        joy_sub = create_subscription<Joy>("joy", 10, std::bind(&Teleop_Control::callback_joy, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Teleop Control initialized and ready");

    }
    
private:
    
    #ifdef HARDWARE_ENABLED
    SparkMax m_left_front, m_left_rear, m_right_front, m_right_rear;
    SparkMax m_belt_left, m_belt_right;
    SparkMax m_leadscrew_left, m_leadscrew_right;
    SparkMax m_dump_conveyor, m_dump_latch;
    MotorControllerGroup left_motors, right_motors;
    #endif

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ready_client;
    SetParameterClientSharedPtr param_client;
    BoolSubscriber sub_xbox;
    BoolSubscriber sub_robot_enabled;
    BoolSubscriber sub_manual_enabled;
    BoolSubscriber sub_outdoor_mode;
    BoolSubscriber sub_ps4;
 
    rclcpp::TimerBase::SharedPtr timer;
    TwistSubscription velocity_subscriber;
    JoySubscription joy_sub;

    ROBOT_ACTUATION_t robot_actuation;
    XBOX_JOYSTICK_INPUT_t xbox_input;
    ROBOTSTATE_t robot_state;
    ROBOT_LIMITS_t robot_dimensions;

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
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == 
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(get_logger(), "State Manager is ready. Message: %s", response->message.c_str());
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
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(get_logger(), "Set parameter %s: %s", param_name.c_str(), response->message.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to set parameter %s: %s", param_name.c_str(), response->message.c_str());
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

    #ifdef HARDWARE_ENABLED
    void config_motor(SparkMax& motor) {
        motor.SetIdleMode(IdleMode::kBrake);
        motor.SetMotorType(MotorType::kBrushless);
        motor.BurnFlash();
    }

    void configure_all_motors() {
        // Config drive motors
        config_motor(m_left_front);
        config_motor(m_left_rear);
        config_motor(m_right_front);
        config_motor(m_right_rear);
        // Config mining motors
        config_motor(m_belt_left);
        config_motor(m_belt_right);
        config_motor(m_leadscrew_left);
        config_motor(m_leadscrew_right);
        // Config dumping motors
        config_motor(m_dump_conveyor);
        config_motor(m_dump_latch);
        m_right_front.SetInverted(true);
        m_right_rear.SetInverted(true);
    }   
    #endif

    //Arbitrary : only If we use a PS4 controller or some other controller for some reason
    void callback_ps4(const msg_Bool &state_PS4){
        robot_state.PS4 = state_PS4.data;
        RCLCPP_INFO(get_logger(), "XBOX Controller State: %s", robot_state.PS4 ? "true" : "false");
        if (robot_state.PS4) {
            RCLCPP_INFO(get_logger(), "PS4 Controller is Active");
        } else {
            RCLCPP_INFO(get_logger(), "PS4 Controller is Inactive");
        } 
    }

    void callback_xbox(const msg_Bool &state_XBOX) {
        robot_state.XBOX = state_XBOX.data;
        RCLCPP_INFO(get_logger(), "XBOX Controller State: %s", robot_state.XBOX ? "true" : "false");
        if (robot_state.XBOX) {
            RCLCPP_INFO(get_logger(), "XBOX Controller is Active");
        } else {
            RCLCPP_INFO(get_logger(), "XBOX Controller is Inactive");
        }
    }

    void callback_robot_enabled(const msg_Bool &state_robot_disabled) {
        robot_state.robot_disabled = state_robot_disabled.data;
        RCLCPP_INFO(get_logger(), "Robot Disabled State: [%s]", 
                    robot_state.robot_disabled ? "true" : "false");
        if (!robot_state.robot_disabled) {
            RCLCPP_INFO(get_logger(), "Robot is ACTIVE");
        } else {
            RCLCPP_INFO(get_logger(), "Robot is INACTIVE");
        }
    }

    void callback_manual_enabled(const msg_Bool &state_manual_enabled) {
        robot_state.manual_enabled = state_manual_enabled.data;
        RCLCPP_INFO(get_logger(), "Manual Control State: %s", robot_state.manual_enabled ? "true" : "false");
    }

    void callback_outdoor_mode(const msg_Bool &state_outdoor_mode){
        robot_state.outdoor_mode = state_outdoor_mode.data;
        RCLCPP_INFO(get_logger(), "Outdoor Mode State: %s", robot_state.outdoor_mode ? "true" : "false");
    }

    void callback_motor_heartbeat() {
        #ifdef HARDWARE_ENABLED   
        try {
            // Drive motors heartbeat
            m_left_front.Heartbeat();
            m_left_rear.Heartbeat();
            m_right_front.Heartbeat();
            m_right_rear.Heartbeat();

            // Mining motors heartbeat
            m_belt_left.Heartbeat();
            m_belt_right.Heartbeat();
            m_leadscrew_left.Heartbeat();
            m_leadscrew_right.Heartbeat();

            // Dumping motors heartbeat
            m_dump_conveyor.Heartbeat();
            m_dump_latch.Heartbeat();
        } catch(const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Motor Heartbeat Error: %s", e.what());
        }
        #endif

        RCLCPP_INFO(get_logger(), 
            "Current States - XBOX: %s, Manual: %s, Robot Enabled: %s",
            robot_state.XBOX ? "true" : "false",
            robot_state.manual_enabled ? "true" : "false",
            !robot_state.robot_disabled ? "true" : "false");
    }

    void callback_joy(const JoyMsg msg) {
        if (robot_state.robot_disabled || !robot_state.manual_enabled || robot_state.PS4 || !robot_state.XBOX) {
            RCLCPP_ERROR(get_logger(), "Error in Joy Callback. Robot must be fully enabled correctly to function!");
            return;
        }

        auto clock = rclcpp::Clock();
        try {
            parse_controller_input(msg);
            handle_mode_switches();
            
            if (get_parameter("manual_enabled").as_bool()) {
                control_robot();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error in joy callback: %s", e.what());
        }
    }

    void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr velocity_msg) {
    if (robot_state.robot_disabled || !robot_state.manual_enabled || 
        robot_state.PS4 || !robot_state.XBOX) {
        RCLCPP_ERROR(get_logger(),
            "Error in CMD VEL Callback. Robot must be fully enabled correctly to function!");
        return;
    }

    RCLCPP_INFO(get_logger(), "INSIDE CMD VEL CALLBACK %lf", velocity_msg->linear.x);
    
    #ifdef HARDWARE_ENABLED
    try {
        rclcpp::Parameter param = get_parameter("manual_enabled");
        bool current_state_manual = param.as_bool();
        
        if (!robot_state.manual_enabled || !current_state_manual) {  // autonomous mode
            SparkMax::Heartbeat();   
            double linear_velocity = velocity_msg->linear.x;
            double angular_velocity = velocity_msg->angular.z;
            robot_dimensions.wheel_radius = robot_state.outdoor_mode ? 0.2 : 0.127;
            
            double velocity_left_cmd = -0.1 * (linear_velocity - angular_velocity * robot_dimensions.wheel_distance / 2.0) / robot_dimensions.wheel_radius;
            double velocity_right_cmd = -0.1 * (linear_velocity + angular_velocity * robot_dimensions.wheel_distance / 2.0) / robot_dimensions.wheel_radius;
        
            left_motors.setSpeed(std::clamp(velocity_left_cmd, -1.0, 1.0));
            right_motors.setSpeed(std::clamp(velocity_right_cmd, -1.0, 1.0));          
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in cmdVelCallback: %s", e.what());
    }
    #endif
}






    void parse_controller_input(const JoyMsg& msg) {
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

        xbox_input.joystick_turn_input = msg->axes[0];
        xbox_input.joystick_forward_input = msg->axes[1];
        xbox_input.secondary_vertical_input = msg->axes[4];
        xbox_input.throttle_backwards = msg->axes[2];
        xbox_input.throttle_forward = msg->axes[5];
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

    void handle_mode_switches() {
        auto clock = rclcpp::Clock();
        
        if (xbox_input.manual_mode_button) {
            set_param("manual_enabled", true);
            robot_state.manual_enabled = true;
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "MANUAL CONTROL: ENABLED");
        }
        
        if (xbox_input.autonomous_mode_button) {
            set_param("manual_enabled", false);
            robot_state.manual_enabled = false;
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "AUTONOMOUS CONTROL: ENABLED");
        }
    }
    
    #ifdef HARDWARE_ENABLED
    void stop(){    
        m_left_front.SetVoltage(0);
        m_left_rear.SetVoltage(0);
        m_right_front.SetVoltage(0);
        m_right_rear.SetVoltage(0);
        
    }
    #endif


    #ifdef HARDWARE_ENABLED
    void emergency_stop() {
        #ifdef HARDWARE_ENABLED
        stop();  
        #endif 
        set_param("robot_disabled", true);  
        robot_state.robot_disabled = true;  
        RCLCPP_ERROR(get_logger(), "EMERGENCY STOP ACTIVATED! DISABLING ROBOT");
    }
    #endif




    void control_robot() {
        if (xbox_input.emergency_stop_button) { emergency_stop(); return;}
        control_drive();
        control_mining();
        control_dumping();
    }

    void control_drive() {
        robot_actuation.velocity_scaling = (xbox_input.x_button && xbox_input.y_button) ? 0.3 : (xbox_input.x_button ? 0.1 : 0.6);
        xbox_input.throttle_forward = (1.0 - xbox_input.throttle_forward) / 2.0;
        xbox_input.throttle_backwards = (1.0 - xbox_input.throttle_backwards) / 2.0;

        if (xbox_input.throttle_forward != 0.0) {
            robot_actuation.wheel_speed_left = xbox_input.throttle_forward - xbox_input.joystick_turn_input;
            robot_actuation.wheel_speed_right = xbox_input.throttle_forward + xbox_input.joystick_turn_input;

        } else if (xbox_input.throttle_backwards != 0.0) {
            robot_actuation.wheel_speed_left = -(xbox_input.throttle_backwards - xbox_input.joystick_turn_input);
            robot_actuation.wheel_speed_right = -(xbox_input.throttle_backwards + xbox_input.joystick_turn_input);

        } else {
            robot_actuation.wheel_speed_left = -xbox_input.joystick_turn_input;
            robot_actuation.wheel_speed_right = xbox_input.joystick_turn_input;
        }
        #ifdef HARDWARE_ENABLED
        left_motors.setSpeed(std::clamp(robot_actuation.wheel_speed_left * robot_actuation.velocity_scaling, -1.0, 1.0));
        right_motors.setSpeed(std::clamp(robot_actuation.wheel_speed_right * robot_actuation.velocity_scaling, -1.0, 1.0));
        #endif
    }

    void control_mining() {
            // Mining belt control (using bumpers)
            double belt_speed = 0.0;
            if (xbox_input.right_bumper) belt_speed = 0.3;  // Forward
            if (xbox_input.left_bumper) belt_speed = -0.3;  // Reverse for jams

            // Leadscrew control (using D-pad vertical)
            double leadscrew_speed = 0.0;
            if (xbox_input.dpad_vertical > 0) leadscrew_speed = 0.3;  // Raise
            if (xbox_input.dpad_vertical < 0) leadscrew_speed = -0.3; // Lower

            #ifdef HARDWARE_ENABLED
            m_belt_left.SetDutyCycle(belt_speed);
            m_belt_right.SetDutyCycle(belt_speed);
            m_leadscrew_left.SetDutyCycle(leadscrew_speed);
            m_leadscrew_right.SetDutyCycle(leadscrew_speed);
            #endif
        }

    void control_dumping() {
        // Dumping control (using A and B buttons)
        double conveyor_speed = xbox_input.a_button ? 0.3 : 0.0;
        double latch_speed = xbox_input.b_button ? 0.3 : 0.0;

        #ifdef HARDWARE_ENABLED
        m_dump_conveyor.SetDutyCycle(conveyor_speed);
        m_dump_latch.SetDutyCycle(latch_speed);
        #endif
    }




};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop_Control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}