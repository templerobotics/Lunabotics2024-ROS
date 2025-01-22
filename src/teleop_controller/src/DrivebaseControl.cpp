/**
 * @file drivebase_control.cpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief ROS2 node for managing the robot's drivebase and coordinating subsystem commands
 * @version 0.1
 * @date 2025-01-11
 * 
 *      This node serves as the primary interface between operator input and robot control.
 *      It directly manages the drivebase motors and publishes commands for other subsystems
 *      while respecting the robot's state as managed by the state manager.
 * 
 *      Why?
 *          I don't want duplicate code in Mining & Dumping Nodes
 *          At runtime all nodes are launched via launch file, but drivebase is the one that 
 *          interprets joystick input & publishes to subsystems: Mining & Dumping
 *          These subsystems then do their work. Would prevent duplicate code that interprets joystick input I think
 */

#include "core.hpp"
#include "MotorControllerGroup.hpp"

class DrivebaseControl : public rclcpp::Node {
public:
    DrivebaseControl() 
        : Node("drivebase_control")
        , m_left_front("can0", 1)
        , m_left_rear("can0", 2)
        , m_right_front("can0", 3)
        , m_right_rear("can0", 4)
        , left_motors(m_left_front, m_left_rear)
        , right_motors(m_right_front, m_right_rear)
    {
        ready_client = create_client<std_srvs::srv::Trigger>("state_manager_ready");
        param_client = create_client<teleop_controller::srv::SetParameter>("set_parameter");
        
        waitForStateManager();
        initializeRobotState();
               
        createSubscribers();
        createPublishers();
        createTimers();

        #ifdef HARDWARE_ENABLED
        configureDrivebase();
        #endif

        RCLCPP_INFO(get_logger(), "Drivebase Control initialized and ready");
    }

private:
    //drivebase core
    SparkMax m_left_front;
    SparkMax m_left_rear;
    SparkMax m_right_front;
    SparkMax m_right_rear;
    MotorControllerGroup left_motors, right_motors;

    //communication interfaces
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ready_client;
    SetParameterClientSharedPtr param_client;
    
    // Publishers for subsystem commands
    VelocityPublisher drivebase_cmd_vel_pub;
    Float64Publisher mining_belt_pub;
    Float64Publisher mining_leadscrew_pub;
    //limit switch?
    Float64Publisher dump_conveyor_pub;
    Float64Publisher dump_latch_pub;

    // Subscribers for input and state
    VelocitySubscriber velocity_sub;
    JoySubscription joy_sub;
    std::vector<BoolSubscriber> state_subscribers;
    
    // Internal state tracking
    ROBOT_ACTUATION_t robot_actuation;
    XBOX_JOYSTICK_INPUT_t xbox_input;
    ROBOTSTATE_t robot_state;
    ROBOT_LIMITS_t robot_dimensions;
    
    rclcpp::TimerBase::SharedPtr timer_heartbeat;
    rclcpp::TimerBase::SharedPtr timer_temp_monitor;
    rclcpp::TimerBase::SharedPtr timer_motor_integrity;

    void waitForStateManager() {
        while (!ready_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for state manager");
                return;
            }
            RCLCPP_INFO(get_logger(), "Waiting for state manager...");
        }
        
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = ready_client->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (!response->success) {
                RCLCPP_ERROR(get_logger(), "Failed to confirm State Manager readiness");
            }
        }
    }

    void initializeRobotState() {
        robot_state.robot_disabled = true;
        robot_state.manual_enabled = true;
        robot_state.XBOX = true;
        robot_state.outdoor_mode = false;

        updateStateManagerParam("robot_disabled", false);
        updateStateManagerParam("XBOX", true);
        updateStateManagerParam("manual_enabled", true);
        updateStateManagerParam("outdoor_mode", false);
    }

    void createSubscribers() {    
        auto sub_robot_enabled = create_subscription<msg_Bool>("robot_state/robot_disabled", 10,std::bind(&DrivebaseControl::updateRobotEnabled, this, std::placeholders::_1));
        auto sub_manual_enabled = create_subscription<msg_Bool>("robot_state/manual_enabled", 10,std::bind(&DrivebaseControl::updateManualEnabled, this, std::placeholders::_1));
        auto sub_xbox = create_subscription<msg_Bool>("robot_state/XBOX", 10,std::bind(&DrivebaseControl::updateXboxState, this, std::placeholders::_1));

        joy_sub = create_subscription<Joy>("joy", 10, std::bind(&DrivebaseControl::handleJoystickInput, this, std::placeholders::_1));
        velocity_sub = create_subscription<Twist>("cmd_vel", 10,std::bind(&DrivebaseControl::drivebase_HandleVelocityCommand, this, _1));
        
        state_subscribers.push_back(sub_robot_enabled);
        state_subscribers.push_back(sub_manual_enabled);
        state_subscribers.push_back(sub_xbox);
    }

    void createPublishers() {
        drivebase_cmd_vel_pub = create_publisher<Twist>("drivebase/cmd_vel", 10);
        mining_belt_pub = create_publisher<Float64>("mining/belt_speed", 10);  
        mining_leadscrew_pub = create_publisher<Float64>("mining/leadscrew_speed", 10);
        dump_conveyor_pub = create_publisher<Float64>("dumping/conveyor_speed", 10);
        dump_latch_pub = create_publisher<Float64>("dumping/latch_speed", 10);
    }

    void createTimers() {
        timer_heartbeat = create_wall_timer(3s, std::bind(&DrivebaseControl::sendMotorHeartbeat, this));
        timer_temp_monitor = create_wall_timer(3s, std::bind(&DrivebaseControl::monitorTemperatures, this));
        timer_motor_integrity = create_wall_timer(3s, std::bind(&DrivebaseControl::checkMotorFaults, this));
    }

   void configureDrivebase() {
        #ifdef HARDWARE_ENABLED
        std::vector<std::reference_wrapper<SparkMax>> motors = {
            std::ref(m_left_front), std::ref(m_left_rear),
            std::ref(m_right_front), std::ref(m_right_rear)
        };
        
        for (SparkMax& motor : motors) {
            motor.SetIdleMode(IdleMode::kCoast);
            motor.SetMotorType(MotorType::kBrushless);
            motor.BurnFlash();
        }
        //verify this in JAVA FRC code
        m_right_front.SetInverted(true);
        m_right_rear.SetInverted(true);
        #endif
    }

    /**
     * @brief interprets XBOX commands for drivebase & robot subsystems like : Mining/Digging & Dumping(conveyor belt & latch)
     * @note I tried to call wrap these function calls within this function to prevent having to interpret JoyMSG commands in every single subsystem file
     */
    void handleJoystickInput(const JoyMsg msg) {
        if (!isRobotOperational()) {
            RCLCPP_ERROR(get_logger(),"ERROR! Robot is NOT operational!");
        }

        parseControllerInput(msg);
        drivebase_PubTwistFromJoy();  
        publishSubsystemCommands();
    }

    bool isRobotOperational() {
        return !robot_state.robot_disabled && robot_state.manual_enabled && robot_state.XBOX;
    }

    void parseControllerInput(const JoyMsg msg) {
        xbox_input.joystick_turn_input = msg->axes[0];
        xbox_input.joystick_forward_input = msg->axes[1];
        xbox_input.throttle_backwards = msg->axes[2];
        xbox_input.throttle_forward = msg->axes[5];
        
        // Parse other inputs for subsystem control
        xbox_input.right_bumper = msg->buttons[5];
        xbox_input.left_bumper = msg->buttons[4];
        xbox_input.dpad_vertical = msg->axes[7];
        xbox_input.a_button = msg->buttons[0];
        xbox_input.b_button = msg->buttons[1];
        xbox_input.y_button = msg->buttons[2];
    }


    void drivebase_PubTwistFromJoy(){
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
        drivebase_cmd_vel_pub->publish(twist_msg);
    }

    void drivebase_HandleVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr velocity_msg) {
        double linear_velocity = velocity_msg->linear.x;
        double angular_velocity = velocity_msg->angular.z;
        
        robot_actuation.wheel_speed_left =  ( linear_velocity - angular_velocity * robot_dimensions.wheel_distance / 2.0 );
        robot_actuation.wheel_speed_right = ( linear_velocity + angular_velocity * robot_dimensions.wheel_distance / 2.0 );
        
        left_motors.setSpeed(std::clamp(robot_actuation.wheel_speed_left, -1.0, 1.0));
        right_motors.setSpeed(std::clamp( robot_actuation.wheel_speed_right, -1.0, 1.0));
    }

    /**
     * @todo Understand how the system is engineered & the way subsystems will actually run code.
     */
    void publishSubsystemCommands() {
        publishMiningCommands();
        publishDumpingCommands();
    }

/**
 * @brief Mining Belt & Leadscrew joystick interpretations
 * @note Prevents code duplication, by interpreting joystick input inside mining node
 * @details In DiggingBelt.cpp we increase belt speed with : right/left bumpers, WHILE mining w/ Y & A in this class
 */
    void publishMiningCommands() {
        auto belt_speed = std_msgs::msg::Float64();
        belt_speed.data = (xbox_input.y_button) ? 0.3 : (xbox_input.a_button) ? -0.3 : 0.0;
        RCLCPP_INFO(get_logger(),"Sending mining belt float command");
        mining_belt_pub->publish(belt_speed);

        auto leadscrew_speed = std_msgs::msg::Float64();
        leadscrew_speed.data = (xbox_input.dpad_vertical > 0) ? 0.3 :(xbox_input.dpad_vertical < 0) ? -0.3 : 0.0;
        RCLCPP_INFO(get_logger(),"Sending mining leadscrew command");
        mining_leadscrew_pub->publish(leadscrew_speed);
    }

    /**
     * @brief Dumping System commands sent packaged as a float.
     * @note Prevents code duplication, by interpreting joystick input inside dumping node
     */
    void publishDumpingCommands() {
        auto dump_cmd_conveyor = std_msgs::msg::Float64();
        dump_cmd_conveyor.data = xbox_input.a_button ? 0.3 : 0.0;
        RCLCPP_INFO(get_logger(),"Sending dumping conveyor float command");  
        dump_conveyor_pub->publish(dump_cmd_conveyor);

        auto dump_cmd_dump_latch = std_msgs::msg::Float64();
        dump_cmd_dump_latch.data = xbox_input.b_button ? 0.3 : 0.0;
        RCLCPP_INFO(get_logger(),"Sending dumping latch float command");
        dump_latch_pub->publish(dump_cmd_dump_latch);
    }

    void sendMotorHeartbeat() {
        #ifdef HARDWARE_ENABLED
        try {
            std::vector<std::reference_wrapper<SparkMax>> motors = {
                std::ref(m_left_front), std::ref(m_left_rear),
                std::ref(m_right_front), std::ref(m_right_rear)
            };
            
            for (SparkMax& motor : motors) {
                motor.Heartbeat();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Motor heartbeat error: %s", e.what());
        }
        #endif
        
        RCLCPP_DEBUG(get_logger(), "Current robot state - Manual: %s, Enabled: %s",robot_state.manual_enabled ? "true" : "false",!robot_state.robot_disabled ? "true" : "false");
    }

    // State update callbacks
    void updateRobotEnabled(const msg_Bool& msg) {
        robot_state.robot_disabled = msg.data;
        if (robot_state.robot_disabled) {
            RCLCPP_INFO(get_logger(),"Robot is disabled...Stopping ALL motors");
            stopAllMotors();
        }
    }

    void updateManualEnabled(const msg_Bool& msg) {
        robot_state.manual_enabled = msg.data;
        if (!msg.data) {
            stopAllMotors();
        }
    }

    void updateXboxState(const msg_Bool& msg) {
        robot_state.XBOX = msg.data;
    }

    /**
     * @brief Stops all drivebase motors & sends stop command to other subsystems 
     * @todo Expand on functionality? Call SetVoltage() on each individual motor if needed for actuation?
     */
    void stopAllMotors() {
        #ifdef HARDWARE_ENABLED
        left_motors.setSpeed(0.0);
        right_motors.setSpeed(0.0);
        #endif
    }

    void updateStateManagerParam(const std::string& param_name, bool value) {
        if (!param_client->wait_for_service(1s)) {
            RCLCPP_ERROR(get_logger(), "Parameter service unavailable");
            return;
        }
        
        auto request = std::make_shared<teleop_controller::srv::SetParameter::Request>();
        request->param_name = param_name;
        request->new_value = value;
        
        auto future = param_client->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (!response->success) {
                RCLCPP_ERROR(get_logger(), "Failed to set %s: %s", param_name.c_str(), response->message.c_str());
            }
        }
    }


    bool checkMotorFaults() {
        #ifdef HARDWARE_ENABLED
        std::vector<std::reference_wrapper<SparkMax>> motors = {
            std::ref(m_left_front), std::ref(m_left_rear),
            std::ref(m_right_front), std::ref(m_right_rear)
        };
        
        for (SparkMax& motor : motors) {
            if (motor.GetFaults() != 0) {
                RCLCPP_ERROR(get_logger(), "Motor fault detected!");
                return false;
            }
        }
        #endif
        return true;
    }

    /**
     * @brief If motors too hot send warning. Might want to give this function more power to cease all robot activity. 
     * 
     */
    void monitorTemperatures() {
        #ifdef HARDWARE_ENABLED
        const float TEMP_THRESHOLD = 70.0;  // Celsius
        std::vector<std::reference_wrapper<SparkMax>> motors = {
            std::ref(m_left_front), std::ref(m_left_rear),
            std::ref(m_right_front), std::ref(m_right_rear)
        };
        
        for (SparkMax& motor : motors) {
            float temp = motor.GetTemperature();
            if (temp > TEMP_THRESHOLD) {
                RCLCPP_WARN(get_logger(), "Motor temperature high: %f C", temp);
            }
        }
        #endif
}





};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrivebaseControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}