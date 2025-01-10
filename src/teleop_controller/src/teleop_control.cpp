/**
 * @file teleop_control.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief XBOX Teleoperated control for our ROS2 robot
 * @version 0.1
 * @date 2025-01-07
 * 
 * @copyright Copyright (c) 2025
 * @todo Can ONLY coding SetDutyCycle() actuate the robot's motors? Test w/ testbench
 */

#include "core.hpp"

class Teleop_Control : public rclcpp::Node {
public:
    Teleop_Control() : Node("Teleop_Control")    
        , m_left_front("can0", 1)
        , m_left_rear("can0", 2)
        , m_right_front("can0", 3)
        , m_right_rear("can0", 4)
        , m_actuator_left("can0",5)
        , m_actuator_right("can0",6)
        , m_belt_left("can0", 7)
        , m_belt_right("can0", 8)
        , m_leadscrew_left("can0", 9)
        , m_leadscrew_right("can0", 10)
        , m_dump_conveyor("can0", 11)
        , m_dump_latch("can0", 12)
        , left_motors(m_left_front, m_left_rear)
        , right_motors(m_right_front, m_right_rear)
    {
        
        ready_client = create_client<std_srvs::srv::Trigger>("state_manager_ready");
        param_client = create_client<teleop_controller::srv::SetParameter>("set_parameter");
        wait_for_state_manager();                               
        prep_robot();
        createPublishers();
        createSubscribers();
        createTimers();

        #ifdef HARDWARE_ENABLED
        configure_all_motors();
        #endif

        RCLCPP_INFO(get_logger(), "Teleop Control initialized and ready");

    }
    
private:
    
    #ifdef HARDWARE_ENABLED
    SparkMax m_left_front, m_left_rear, m_right_front, m_right_rear;
    SparkMax m_actuator_left, m_actuator_right;
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
 
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    JoySubscription joy_sub;
    VelocityPublisher cmd_vel_pub;
    VelocitySubscriber velocity_subscriber;

    ROBOT_ACTUATION_t robot_actuation;
    XBOX_JOYSTICK_INPUT_t xbox_input;
    ROBOTSTATE_t robot_state;
    ROBOT_LIMITS_t robot_dimensions;

    void createSubscribers() {
        sub_xbox = create_subscription<msg_Bool>("robot_state/XBOX", 10, std::bind(&TeleopControl::callback_xbox, this, std::placeholders::_1));
        sub_robot_enabled = create_subscription<msg_Bool>("robot_state/enabled", 10,std::bind(&TeleopControl::callback_robot_enabled, this, std::placeholders::_1));
        sub_manual_enabled = create_subscription<msg_Bool>("robot_state/manual_enabled", 10,std::bind(&TeleopControl::callback_manual_enabled, this, std::placeholders::_1));      
        sub_outdoor_mode = create_subscription<msg_Bool>("robot_state/outdoor_mode", 10,std::bind(&TeleopControl::callback_outdoor_mode, this, std::placeholders::_1));     
        sub_ps4 = create_subscription<msg_Bool>("robot_state/PS4", 10,std::bind(&TeleopControl::callback_ps4, this, std::placeholders::_1));   
        velocity_subscriber = create_subscription<Twist>("cmd_vel", 10, std::bind(&TeleopControl::handleVelocityCommand, this, std::placeholders::_1));
        joy_sub = create_subscription<Joy>("joy", 10, std::bind(&Teleop_Control::callback_joy, this, std::placeholders::_1));
    }

    void createPublishers() {
        cmd_vel_pub = create_publisher<Twist>("cmd_vel", 10);
    }

    void createTimers() {
        heartbeat_timer = create_wall_timer(5s, std::bind(&TeleopControl::callback_motor_heartbeat, this));
    }



/**
 * @brief Halts teleop control until state manager fully completes the "bringup"/robot initialization step. Honestly probably too redundant & not necessary
 * 
 */
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

/**
 * @brief Sets the param argument to the intended new boolean value
 * @param param_name from declared params in State Manager
 * @param new_val 0 or 1 
 */
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

/**
 * @brief Initializes Robot XBOX Teleoperation Control parameters, so the single source of truth(state_mananger.cpp) publishes them
 * 
 */
    void prep_robot() {
        RCLCPP_INFO(get_logger(), "Initializing robot parameters...");
        set_param("robot_disabled", false);
        set_param("XBOX", true);
        set_param("PS4", false);
        set_param("manual_enabled", true);
        set_param("outdoor_mode", false);
        RCLCPP_INFO(get_logger(), "Robot parameters initialized");
    }


/**
 * @brief Configures NEO BLDC Motors
 * @todo Make a version of this function for the smaller NEO motors, unless they are also BLDC;I havent checked.
 * @param motor SparkMax instance
 */
    void config_motor(SparkMax& motor) {
        #ifdef HARDWARE_ENABLED
        motor.SetIdleMode(IdleMode::kBrake);
        motor.SetMotorType(MotorType::kBrushless);
        motor.BurnFlash();
        #endif
    }

/**
 * @brief Self-explanatory
 * @todo If the smaller NEO motors are NOT BLDC, use their version of config_motor() inside this function
 */
    void configure_all_motors() {
        #ifdef HARDWARE_ENABLED
        // Config drive motors
        config_motor(m_left_front);
        config_motor(m_left_rear);
        config_motor(m_right_front);
        config_motor(m_right_rear);
        // Config mining motors
        config_motor(m_actuator_left);
        config_motor(m_actuator_right);
        config_motor(m_belt_left);
        config_motor(m_belt_right);
        config_motor(m_leadscrew_left);
        config_motor(m_leadscrew_right);
        // Config dumping motors
        config_motor(m_dump_conveyor);
        config_motor(m_dump_latch);
        m_right_front.SetInverted(true);
        m_right_rear.SetInverted(true);
        #endif
    }   
    

/**
 * @brief Saves the currently publishing PS4 state param to this file's LOCAL state 
 * 
 * @param state_PS4 
 */
    void callback_ps4(const msg_Bool &state_PS4){
        robot_state.PS4 = state_PS4.data;
        RCLCPP_INFO(get_logger(), "XBOX Controller State: %s", robot_state.PS4 ? "true" : "false");
        if (robot_state.PS4) {
            RCLCPP_INFO(get_logger(), "PS4 Controller is Active");
        } else {
            RCLCPP_INFO(get_logger(), "PS4 Controller is Inactive");
        } 
    }

/**
 * @brief Saves the currently publishing XBOX state param to this file's LOCAL state
 * @param state_XBOX 
 */

    void callback_xbox(const msg_Bool &state_XBOX) {
        robot_state.XBOX = state_XBOX.data;
        RCLCPP_INFO(get_logger(), "XBOX Controller State: %s", robot_state.XBOX ? "true" : "false");
        if (robot_state.XBOX) {
            RCLCPP_INFO(get_logger(), "XBOX Controller is Active");
        } else {
            RCLCPP_INFO(get_logger(), "XBOX Controller is Inactive");
        }
    }

/**
 * @brief Saves the currently publishing robot_disabled state param to this file's LOCAL state
 * @param state_robot_disabled 
 */

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

/**
 * @brief Saves the currently publishing manual_enalbed state param to this file's LOCAL state
 * @param state_manual_enabled 
 */
    void callback_manual_enabled(const msg_Bool &state_manual_enabled) {
        robot_state.manual_enabled = state_manual_enabled.data;
        RCLCPP_INFO(get_logger(), "Manual Control State: %s", robot_state.manual_enabled ? "true" : "false");
    }

/**
 * @brief Saves the currently publishing outdoor_mode state param to this file's LOCAL state
 * @param state_outdoor_mode 
 */
    void callback_outdoor_mode(const msg_Bool &state_outdoor_mode){
        robot_state.outdoor_mode = state_outdoor_mode.data;
        RCLCPP_INFO(get_logger(), "Outdoor Mode State: %s", robot_state.outdoor_mode ? "true" : "false");
    }


/**
 * @brief Periodically sends a heartbeat signal to keep all SPARK controllers active
 */
    void callback_motor_heartbeat() {
        #ifdef HARDWARE_ENABLED   
        try {
            // Drive motors heartbeat
            m_left_front.Heartbeat();
            m_left_rear.Heartbeat();
            m_right_front.Heartbeat();
            m_right_rear.Heartbeat();

            // Mining motors heartbeat
            m_actuator_left.Heartbeat();
            m_actuator_right.Heartbeat();
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




/**
 * @brief Interprets & sets XBOX controller joystick input.Saved locally to this node in struct variable.  Part 2
 * @param msg 
 */
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

/**
 * @brief Fetch the button pressed by XBOX Controller
 * @param joy_msg 
 * @param mappings 
 * @return int 
 */
    int get_button(const JoyMsg &joy_msg, const std::initializer_list<int> &mappings) {
        bool xbox_enabled = get_parameter("XBOX").as_bool();
        bool ps4_enabled = get_parameter("PS4").as_bool();
        size_t index = xbox_enabled ? 2 : ps4_enabled ? 1 : 0;
        return joy_msg->buttons[*(mappings.begin() + index)];
    }

/**
 * @brief Get the axis of button pressed
 * @param joy_msg 
 * @param mappings 
 * @return double 
 */
    double get_axis(const JoyMsg &joy_msg, const std::initializer_list<int> &mappings) {
        bool xbox_enabled = get_parameter("XBOX").as_bool();
        bool ps4_enabled = get_parameter("PS4").as_bool();
        size_t index = xbox_enabled ? 2 : ps4_enabled ? 1 : 0;
        return joy_msg->axes[*(mappings.begin() + index)];
    }

  
   
/**
 * @brief Callback that interprets XBOX controller joystick input. Part 1 
 * @param msg 
 */
    void callback_joy(const JoyMsg& msg) {
        if (robot_state.robot_disabled || robot_state.manual_enabled == false) {
            RCLCPP_ERROR(get_logger(), "Error in Joy Callback. Robot must be fully enabled correctly to function!");
            return;
        }
        parse_controller_input(msg); 
        publishTwistFromJoy();
    }


/**
 * @brief Publishes velocity msgs to send to the Robot. Part 3
 */
    void publishTwistFromJoy() {
        auto twist_msg = geometry_msgs::msg::Twist();
        robot_actuation.speed_scaling_factor_drivebase = (xbox_input.x_button && xbox_input.y_button) ? 0.3 : (xbox_input.x_button ? 0.1 : 0.6);
        double linear_x = 0.0;

        if (xbox_input.throttle_forward != 0.0) {
            linear_x = xbox_input.throttle_forward;
        } else if (xbox_input.throttle_backwards != 0.0) {
            linear_x = -xbox_input.throttle_backwards;
        }

        twist_msg.linear.x = linear_x * robot_actuation.speed_scaling_factor_drivebase ;
        twist_msg.angular.z = -xbox_input.joystick_turn_input * robot_actuation.speed_scaling_factor_drivebase;
        cmd_vel_pub->publish(twist_msg);

    }

/**
 * @brief Actuates motors based on incoming published velocities . Step 3
 * @param velocity_msg incoming published velocities
 * @note see publishTwistFromJoy() to see how motor speed scaling factors to Twist velocities are applied.
 */
    void handleVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr velocity_msg) {
        double linear_velocity = velocity_msg->linear.x;
        double angular_velocity = velocity_msg->angular.z;
        
        robot_actuation.wheel_speed_left =  ( linear_velocity - angular_velocity * robot_dimensions.wheel_distance / 2.0 );
        robot_actuation.wheel_speed_right = ( linear_velocity + angular_velocity * robot_dimensions.wheel_distance / 2.0 );
        
        left_motors.SetSpeed(std::clamp(robot_actuation.wheel_speed_left, -1.0, 1.0));
        right_motors.SetSpeed(std::clamp( robot_actuation.wheel_speed_right, -1.0, 1.0));

    }


/**
 * @brief Wrapper that encapsulates robotic mining & dumping for teleoperation
 * 
 */
    void control_robot() {
        if (xbox_input.emergency_stop_button) { emergency_stop(); return;}
        control_mining();
        control_dumping();
    }



/**
 * @brief Teleop Mining Control 
 * @todo Limit Switches/ LeadScrew State / Encoder Usage / PID instead of duty cycle / Temperature & Current Monitoring
 */
    void control_mining() {
        robot_actuation.speed_lift_actuator = (xbox_input.dpad_vertical == 1.0) ? -0.3 : (xbox_input.dpad_vertical == -1.0) ? 0.3 : 0.0;
        robot_actuation.speed_tilt_actuator = (xbox_input.secondary_vertical_input > 0.1) ? -0.3 : (xbox_input.secondary_vertical_input < -0.1) ? 0.3 : 0.0;

        double leadscrew_speed = 0.0;
        if (xbox_input.dpad_vertical > 0) leadscrew_speed = 0.3;  // Raise
        if (xbox_input.dpad_vertical < 0) leadscrew_speed = -0.3; // Lower

        double belt_speed = 0.0;
        if (xbox_input.right_bumper) belt_speed = 0.3;  // Forward
        if (xbox_input.left_bumper) belt_speed = -0.3;  // Reverse

        #ifdef HARDWARE_ENABLED
        m_actuator_left.SetDutyCycle(std::clamp(robot_actuation.speed_lift_actuator,-1.0,1.0));
        m_actuator_left.SetDutyCycle(std::clamp(robot_actuation.speed_tilt_actuator,-1.0,1.0));
        m_belt_left.SetDutyCycle(std::clamp(belt_speed,-1.0,1.0));
        m_belt_right.SetDutyCycle(std::clamp(belt_speed,-1.0,1.0));
        m_leadscrew_left.SetDutyCycle(std::clamp(leadscrew_speed,-1.0,1.0));
        m_leadscrew_right.SetDutyCycle(std::clamp(leadscrew_speed,-1.0,1.0));
        #endif
    }

/**
 * @brief Teleop Dumping control
 * @todo Review the dumping system w/ the team to make additions/changes
 */
    void control_dumping() {
        double conveyor_speed = xbox_input.a_button ? 0.3 : 0.0;
        double latch_speed = xbox_input.b_button ? 0.3 : 0.0;
        #ifdef HARDWARE_ENABLED
        m_dump_conveyor.SetDutyCycle(std::clamp(conveyor_speed,-1.0,1.0));
        m_dump_latch.SetDutyCycle(std::clamp(latch_speed,-1.0,1.0));
        #endif
    }

/**
 * @brief Sets/notifies source of truth(state_manager.cpp) that we want Teleop Mode or Autonomous Mode. If Autonomous mode, no teleoperation should function
 * 
 */
void handle_mode_switches() {
        
        if (xbox_input.manual_mode_button) {
            set_param("manual_enabled", true);
            robot_state.manual_enabled = true;
            RCLCPP_INFO(get_logger(), "MANUAL CONTROL: ENABLED");
        }
        
        if (xbox_input.autonomous_mode_button) {
            set_param("manual_enabled", false);
            robot_state.manual_enabled = false;
            RCLCPP_INFO(get_logger() "AUTONOMOUS CONTROL: ENABLED");
        }
    }


/**
 * @brief Emergency Stop functions.
 * @todo  Make sure the estop() function is called in an appropriate place in a robot control function if needed 
 * @note Maybe do a while(count<3) decrease down to 0 to act as a kCoast for safety? Prob not necessary though.
 */
    #ifdef HARDWARE_ENABLED
    void stop(){    
        m_left_front.SetDutyCycle(0.0);
        m_left_front.SetVoltage(0);
        m_left_rear.SetDutyCycle(0.0);
        m_left_rear.SetVoltage(0);
        m_right_front.SetDutyCycle(0.0);
        m_right_front.SetVoltage(0);
        m_right_rear.SetDutyCycle(0.0);
        m_right_rear.SetVoltage(0);
        stop_mining();
        stop_digging();
    }
    void stop_mining(){
        m_actuator_left.SetDutyCycle(0.0);
        m_actuator_left.SetVoltage(0);
        m_actuator_right.SetDutyCycle(0.0);
        m_actuator_right.SetVoltage(0);
        m_belt_left.SetDutyCycle(0.0);
        m_belt_left.SetVoltage(0);
        m_belt_right.SetDutyCycle(0.0);
        m_belt_right.SetVoltage(0);
        m_leadscrew_left.SetDutyCycle(0.0);
        m_leadscrew_left.SetVoltage(0);
        m_leadscrew_right.SetDutyCycle(0.0);
        m_leadscrew_right.SetVoltage(0);   
    }
    void stop_digging(){
        m_dump_conveyor.SetDutyCycle(0.0);
        m_dump_conveyor.SetVoltage(0);
        m_dump_latch.SetDutyCycle(0.0);
        m_dump_latch.SetVoltage(0);
    }
    #endif

    #ifdef HARDWARE_ENABLED
    void emergency_stop() {
        #ifdef HARDWARE_ENABLED
        stop(); 
        #endif 
        set_param("robot_disabled", true);  
        robot_state.robot_disabled = true;  
        RCLCPP_ERROR(get_logger(), "EMERGENCY STOP ACTIVATED! DISABLING ROBOT MOTORS...");
    }
    #endif



};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop_Control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}