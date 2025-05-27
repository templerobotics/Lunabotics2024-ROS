/**
 * @brief Temple Lunabotics Digging Subsystem
 * @todo Digging Belt - PID & Sensor Diagnostics via Timer
 * @todo Leadscrews start at 0 , Actuators are down. Initializatin was leadscrews down for 2 seconds then came up till hit top limit switch even if at 0
 */
#include "core.hpp"
#include "Digging.hpp"
#include "DriveBase.hpp"

Digging::Digging()
    : Node("digging")
    , m_belt_left("can0", BELT_left_CAN_ID)
    , m_belt_right("can0", BELT_right_CAN_ID)
    , m_linear_left("can0", LINEAR_LEFT_CAN_ID)
    , m_linear_right("can0", LINEAR_RIGHT_CAN_ID)
    , m_leadscrew_left("can0", LEADSCREW_left_CAN_ID)
    , m_leadscrew_right("can0", LEADSCREW_right_CAN_ID)
        {
            joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Digging::joy_callback_digging, this, std::placeholders::_1));
            initMotors();    
            configureLimitSwitches();
            linear_actuator_state_left = LinearActuatorStateLeft::Unknown;
            linear_actuator_state_right = LinearActuatorStateRight::Unknown;
            digging_right_speed_pub = create_publisher<std_msgs::msg::Float64>("/digging/right/speed", 10);
            digging_left_speed_pub = create_publisher<std_msgs::msg::Float64>("/digging/left/speed", 10);
            // digging_right_temp_pub = create_publisher<std_msgs::msg::Float64>("/digging/right/temp", 30);
            // digging_left_temp_pub = create_publisher<std_msgs::msg::Float64>("/digging/left/temp", 30);
            leadscrew_right_speed_pub = create_publisher<std_msgs::msg::Float64>("/leadscrew/right/speed", 10);
            leadscrew_left_speed_pub = create_publisher<std_msgs::msg::Float64>("/leadscrew/left/speed", 10);
            // leadscrew_right_temp_pub = create_publisher<std_msgs::msg::Float64>("/leadscrew/right/temp", 30);
            // leadscrew_left_temp_pub = create_publisher<std_msgs::msg::Float64>("/leadscrew/left/temp", 30);
            leadscrew_right_position_pub = create_publisher<std_msgs::msg::Float64>("leadscrew/right/position", 30);
            leadscrew_left_position_pub = create_publisher<std_msgs::msg::Float64>("leadscrew/left/position", 30);
            actuator_right_position_pub = create_publisher<std_msgs::msg::Float64>("/actuator/right/position", 10);
            actuator_left_position_pub = create_publisher<std_msgs::msg::Float64>("/actuator/left/position", 10);
            actuator_right_state_pub = create_publisher<std_msgs::msg::String>("actuator/right/state", 100);
            actuator_left_state_pub = create_publisher<std_msgs::msg::String>("actuator/left/state", 100);
            timer_diagnostics = create_wall_timer(std::chrono::milliseconds(10), std::bind(&Digging::periodic, this));
            timer_linear_actuators = create_wall_timer(std::chrono::milliseconds(10), std::bind(&Digging::periodicLinearActuatorCheck, this));
            is_digging_running_pub = this->create_publisher<std_msgs::msg::Bool>("/is_digging_running", 10);
            drivetrain_right_pub = this->create_publisher<std_msgs::msg::Float64>("/drivetrain_right", 10);
            drivetrain_left_pub = this->create_publisher<std_msgs::msg::Float64>("/drivetrain_left", 10);
            RCLCPP_INFO(this->get_logger(), "Digging Subsystem Successfully Initialized!");


            telemetry_timer = this->create_wall_timer(
                100ms, [this]() {
            try {
                auto msg = std_msgs::msg::Float64();
                msg.data = m_leadscrew_right.GetVelocity();
                leadscrew_right_speed_pub->publish(msg);
                msg.data = m_leadscrew_left.GetVelocity();
                leadscrew_left_speed_pub->publish(msg);
                msg.data = m_belt_right.GetVelocity();
                digging_right_speed_pub->publish(msg);
                msg.data = m_belt_left.GetVelocity();
                digging_left_speed_pub->publish(msg);
                // msg.data = m_leadscrew_right.GetTemperature();
                // leadscrew_right_temp_pub->publish(msg);
                // msg.data = m_leadscrew_left.GetTemperature();
                // leadscrew_left_temp_pub->publish(msg);
                // msg.data = m_belt_right.GetTemperature();
                // digging_right_temp_pub->publish(msg);
                // msg.data = m_belt_left.GetTemperature();
                // digging_left_temp_pub->publish(msg);
                msg.data = m_linear_right.GetAnalogPosition();
                actuator_right_position_pub->publish(msg);
                msg.data = m_linear_left.GetAnalogPosition();
                actuator_left_position_pub->publish(msg);
                msg.data = m_leadscrew_left.GetPosition();
                leadscrew_left_position_pub->publish(msg);
                msg.data = m_leadscrew_right.GetPosition();
                leadscrew_right_position_pub->publish(msg);

                auto msgStateRight = std_msgs::msg::String();
                msgStateRight.data = stateToStringActuatorRight(linear_actuator_state_right);
                actuator_right_state_pub->publish(msgStateRight);

                auto msgStateLeft = std_msgs::msg::String();
                msgStateLeft.data = stateToStringActuatorLeft(linear_actuator_state_left);
                actuator_left_state_pub->publish(msgStateLeft);

                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "Failed to read motor metrics: %s", e.what());
                }
            });
        }


    /**
     * @brief Interprets XBOX Joystick digging commands
     * @todo Implement more logic for this function
     */
    void Digging::joy_callback_digging(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        // m_linear_left.SetDutyCycle(1);
        // m_linear_right.SetDutyCycle(1);
        bool current_a = joy_msg->buttons[0];
        if (current_a && !last_a_state) {  // Button A just pressed
            if (belt_running) {
                stopDiggingBeltMotors();
            } else {
                setBeltSpeedForward(1);//1  // Or however fast you want
            }
        }
        last_a_state = current_a;

        // Debounced toggle logic for Y (button 3)
        bool current_y = joy_msg->buttons[3]; //2 when on the nuc
        if (current_y && !last_y_state) {  // Button Y just pressed
            if (belt_running) {
                stopDiggingBeltMotors();
            } else {
                setBeltSpeedReverse(1); //1 // Opposite direction
            }
        }
        last_y_state = current_y;

        double rightTrigger = joy_msg->axes[4]; //5 when on the nuc
        double leftTrigger = joy_msg->axes[5]; //2 when on the nuc
        double leadscrewSpeed;
        if (leftTrigger > MIN_THROTTLE_DEADZONE && rightTrigger > MIN_THROTTLE_DEADZONE) {
            leadscrewSpeed = 0;
        }
        else if (leftTrigger > MIN_THROTTLE_DEADZONE){
			leadscrewSpeed = -1*leftTrigger;
		}
		else if (rightTrigger > MIN_THROTTLE_DEADZONE){
			leadscrewSpeed = rightTrigger;
		}
        if(belt_running){
            setLeadscrewSpeed(clamp(leadscrewSpeed, -0.3, 1.0));
        }
        else{
            setLeadscrewSpeed(clamp(leadscrewSpeed, -1.0, 1.0));
        }

        bool current_b = joy_msg->buttons[1];
        if (current_b && !last_b_state) {
            if (actuators_running_right) {
                commandStopRight();
            } else {
                commandUpRight();
            }
            if (actuators_running_left) {
                commandStopLeft();
            } else {
                commandUpLeft();
            }
        }
        last_b_state = current_b;
    
        bool current_x = joy_msg->buttons[2];  // X = down //3 when on the nuc
        if (current_x && !last_x_state) {
            if (actuators_running_right) {
                commandStopRight();
            } else {
                commandDownRight();
            }
            if (actuators_running_left) {
                commandStopLeft();
            } else {
                commandDownLeft();
            }
        }
        last_x_state = current_x;


    //     bool auto_button = joy_msg->buttons[11];
    //     if(auto_button && !last_auto_state){
    //         try{
    //             mode_publisher = create_publisher<std_msgs::msg::String>("current_mode", 10);
    //             auto msg = std_msgs::msg::String();
    //             msg.data = "autonomy";
    //             mode_publisher->publish(msg);
    //             // commandUpRight();
    //             // commandUpLeft();
    //             // if(linear_actuator_state_left == LinearActuatorStateLeft::Raised && linear_actuator_state_right == LinearActuatorStateRight::Raised){
    //             //     setBeltSpeedForward(1);
    //             // }
    //             auto start = std::chrono::high_resolution_clock::now();
    //             // while (std::chrono::duration_cast<std::chrono::seconds>(
    //             //     std::chrono::high_resolution_clock::now() -
    //             //     start1)
    //             // .count() < 10)
    //             // {
    //             //     setLeadscrewSpeed(clamp(leadscrewSpeed, -0.3, 0.0));
    //             //     std::cout.flush();
    //             // }
    //             // stopLeadScrew();
    //             // auto start2 = std::chrono::high_resolution_clock::now();
    //             // while (std::chrono::duration_cast<std::chrono::seconds>(
    //             //     std::chrono::high_resolution_clock::now() -
    //             //     start2)
    //             // .count() < 10)
    //             // {
    //             //     setLeadscrewSpeed(clamp(leadscrewSpeed, 0.0, 1.0));
    //             //     commandDownLeft();
    //             //     commandDownRight();
    //             //     stopDiggingBeltMotors();
    //             //     std::cout.flush();
    //             // }
    //             double motor_cmd_left = 0.0;
    //             double motor_cmd_right = 0.0;
    //             using namespace std::chrono;
    //             start = high_resolution_clock::now();
    //             while (duration<double>(high_resolution_clock::now() - start).count() < 2) {
    //                 motor_cmd_left = 0.3;
    //                 motor_cmd_right = 0.3;

    //                 std_msgs::msg::Float64 msg;
    //                 msg.data = motor_cmd_left;
    //                 drivetrain_left_pub->publish(msg);

    //                 msg.data = motor_cmd_right;
    //                 drivetrain_right_pub->publish(msg);
    //             }
    //             motor_cmd_left = 0.0;
    //             motor_cmd_right = 0.0;

    //             std_msgs::msg::Float64 msgDrive;
    //             msgDrive.data = motor_cmd_left;
    //             drivetrain_left_pub->publish(msgDrive);

    //             msgDrive.data = motor_cmd_right;
    //             drivetrain_right_pub->publish(msgDrive);
    //             start = high_resolution_clock::now();
    //             // while (duration<double>(high_resolution_clock::now() - start).count() < 1) {
    //             //     commandUpLeft();
    //             //     commandUpRight();
    //             // }
    //             // commandStopLeft();
    //             // commandStopRight();
    //         }
    //         catch (const std::exception& e) {
    //             RCLCPP_ERROR(get_logger(), "Failed to run leadscrew motors: %s", e.what());
    //             stopDiggingBeltMotors();
    //             stopLinearActuatorMotorsLeft();
    //             stopLinearActuatorMotorsRight();
    //             stopLeadScrew();
    //         }
    //     }
    //     last_auto_state = auto_button;
    }

    /**
     * @brief The Digging Belt spins like a smily face 
     */
    void Digging::setBeltSpeedForward(double speed) {
        m_belt_left.SetDutyCycle(speed);
        m_belt_right.SetDutyCycle(-1*speed);
        belt_running = true;

        std_msgs::msg::Bool msg;
        msg.data = true;
        is_digging_running_pub->publish(msg);
    }
    
    void Digging::setBeltSpeedReverse(double speed) {
        m_belt_left.SetDutyCycle(-1*speed);
        m_belt_right.SetDutyCycle(speed);
        belt_running = true;

        std_msgs::msg::Bool msg;
        msg.data = true;
        is_digging_running_pub->publish(msg);
    }

    void Digging::stopDiggingBeltMotors() {
        RCLCPP_INFO(get_logger(), "STOPPING DIGGING BELT MOTORS!");
        m_belt_left.SetDutyCycle(0.0);
        m_belt_right.SetDutyCycle(0.0);
        belt_running = false;

        std_msgs::msg::Bool msg;
        msg.data = false;
        is_digging_running_pub->publish(msg);
    }
    void Digging::stopLeadScrew(){
        m_leadscrew_left.SetDutyCycle(0); 
        m_leadscrew_right.SetDutyCycle(0);
    }

    bool Digging::isRunning(){
        return belt_running;
    }
    void Digging::configureLimitSwitches() {
        // Config forward limit switches as normally closed
        m_leadscrew_left.SetLimitSwitchFwdPolarity(true);  // NC = true
        m_leadscrew_right.SetLimitSwitchFwdPolarity(true);
        
        // Config reverse limit switches as normally open
        m_leadscrew_left.SetLimitSwitchRevPolarity(false); // NO = false
        m_leadscrew_right.SetLimitSwitchRevPolarity(false);
        
        m_leadscrew_left.SetHardLimitFwdEn(true);
        m_leadscrew_left.SetHardLimitRevEn(true);

        m_leadscrew_right.SetHardLimitFwdEn(true);
        m_leadscrew_right.SetHardLimitRevEn(true);

        RCLCPP_INFO(get_logger(), "Limit switches configured");
    }

    bool Digging::checkFault(uint16_t faults, FaultBits bit) {
        return (faults & (1 << static_cast<int>(bit))) != 0;
    }

    void Digging::periodic() {
        checkLeadscrewLimits();
        checkLinearActuatorLimits();
    }

    void Digging::checkLeadscrewLimits() {
        auto position = m_leadscrew_left.GetPosition();
        uint16_t faults1 = m_leadscrew_left.GetFaults();
        uint16_t faults2 = m_leadscrew_right.GetFaults();

        bool topLimitRight = checkFault(faults1, FaultBits::kHardLimitFwd);
        bool topLimitLeft = checkFault(faults2, FaultBits::kHardLimitFwd);
        bool bottomLimit = checkFault(faults1, FaultBits::kHardLimitRev) || checkFault(faults2, FaultBits::kHardLimitRev);
        if(topLimitLeft){
            m_leadscrew_left.SetPosition(0);
        }
        if(topLimitRight){
            m_leadscrew_right.SetPosition(0);
        }
        bool topLimit = topLimitLeft && topLimitRight;
        if (topLimit || bottomLimit) {
            RCLCPP_DEBUG(get_logger(), "Limit switch triggered - TopLeft: %s, Bottom: %s", topLimit ? "true" : "false", bottomLimit ? "true" : "false");
        }

        // Update state based on position and limits
        if (leadscrew_state != LeadscrewState::FullExtended && position >= LEADSCREW_MAX_TRAVEL - LEADSCREW_MAX_ERROR) {
            leadscrew_state = LeadscrewState::FullExtended;
            // RCLCPP_INFO(get_logger(), "Leadscrew reached full extension");
        }
        else if (leadscrew_state != LeadscrewState::Retracted && (position < LEADSCREW_MAX_ERROR || topLimit)) {
            leadscrew_state = LeadscrewState::Retracted;
            // RCLCPP_INFO(get_logger(), "Leadscrew fully retracted");
        }
        else if (leadscrew_state != LeadscrewState::Extended && position > LEADSCREW_MAX_ERROR && std::abs(position - LEADSCREW_MAX_TRAVEL) >= LEADSCREW_MAX_ERROR) {
            leadscrew_state = LeadscrewState::Extended;
        }
    }

    std::string Digging::stateToStringLeadScrew(LeadscrewState state) {
        switch (state) {
            case LeadscrewState::Extended: return "Extended";
            case LeadscrewState::Retracted: return "Retracted";
            case LeadscrewState::Traveling: return "Traveling";
            case LeadscrewState::FullExtended: return "FullExtended";
            case LeadscrewState::GivenCommand: return "GivenCommand";
            default: return "Unknown";
        }
    }
    std::string Digging::stateToStringActuatorLeft(LinearActuatorStateLeft state){
        switch (state) {
            case LinearActuatorStateLeft::Raised: return "Raised";
            case LinearActuatorStateLeft::Lowered: return "Lowered";
            case LinearActuatorStateLeft::TravelingUp: return "TravelingUp";
            case LinearActuatorStateLeft::TravelingDown: return "TravelingDown";
            case LinearActuatorStateLeft::Commanded: return "Commanded";
            case LinearActuatorStateLeft::Stopped: return "Stopped";
            default: return "Unknown";
        }
    }
    std::string Digging::stateToStringActuatorRight(LinearActuatorStateRight state){
        switch (state) {
            case LinearActuatorStateRight::Raised: return "Raised";
            case LinearActuatorStateRight::Lowered: return "Lowered";
            case LinearActuatorStateRight::TravelingUp: return "TravelingUp";
            case LinearActuatorStateRight::TravelingDown: return "TravelingDown";
            case LinearActuatorStateRight::Commanded: return "Commanded";
            case LinearActuatorStateRight::Stopped: return "Stopped";
            default: return "Unknown";
        }
    }
    bool Digging::isTopLimitPressed() {
        uint16_t faults1 = m_leadscrew_left.GetFaults();
        uint16_t faults2 = m_leadscrew_right.GetFaults();
        return checkFault(faults1, FaultBits::kHardLimitFwd) || checkFault(faults2, FaultBits::kHardLimitFwd);
    }

    bool Digging::isBottomLimitPressed() {
        uint16_t faults1 = m_leadscrew_left.GetFaults();
        uint16_t faults2 = m_leadscrew_right.GetFaults();
        return checkFault(faults1, FaultBits::kHardLimitRev) || checkFault(faults2, FaultBits::kHardLimitRev);
    }

    void Digging::setLeadscrewSpeed(double speed) {
        auto position = std::abs(m_leadscrew_left.GetPosition());
        
        // if (position <= LEADSCREW_MAX_ERROR && speed < 0) {
        //     RCLCPP_WARN(get_logger(), "At bottom limit, cannot move down further");
        //     return;
        // }
        
        if (position >= LEADSCREW_MAX_TRAVEL - LEADSCREW_MAX_ERROR && speed > 0) {
            RCLCPP_WARN(get_logger(), "At top limit, cannot move up further");
            return;
        }

        leadscrew_state = LeadscrewState::Traveling;
        m_leadscrew_left.SetDutyCycle(speed);
        m_leadscrew_right.SetDutyCycle(speed);

    }

    LeadscrewState Digging::getLeadscrewState() {
        return leadscrew_state;
    }

    // LINEAR ACTUATORS

    void Digging::periodicLinearActuatorCheck(){
        checkLinearActuatorLimits();
    }


    void Digging::commandUpRight() {
        actuators_running_right = true;
        linear_actuator_state_right = LinearActuatorStateRight::Commanded;
        linearUpRight();
    }
    
    void Digging::commandUpLeft() {
        actuators_running_left = true;
        linear_actuator_state_left = LinearActuatorStateLeft::Commanded;
        linearUpLeft();
    }
    void Digging::commandDownRight() {
        actuators_running_right = true;
        linear_actuator_state_right = LinearActuatorStateRight::Commanded;
        linearDownRight();
    }
    
    void Digging::commandDownLeft() {
        actuators_running_left = true;
        linear_actuator_state_left = LinearActuatorStateLeft::Commanded;
        linearDownLeft();
    }

    void Digging::commandStopRight() {
        stopLinearActuatorMotorsRight();
    }
    
    void Digging::commandStopLeft() {
        stopLinearActuatorMotorsLeft();
    }
    
    void Digging::linearUpRight() {
        linear_actuator_state_right = LinearActuatorStateRight::TravelingUp;
        m_linear_right.SetDutyCycle(-1);
    }
    
    void Digging::linearUpLeft() {
        linear_actuator_state_left = LinearActuatorStateLeft::TravelingUp;
        m_linear_left.SetDutyCycle(-1);
    }
    
    void Digging::linearDownRight() {
        if (m_linear_right.GetAnalogPosition() <= LINEAR_MIN_TRAVEL) {
            m_linear_right.SetDutyCycle(0);
            return;
        }
        linear_actuator_state_right = LinearActuatorStateRight::TravelingDown;
        m_linear_right.SetDutyCycle(1.0);
    }
    
    void Digging::linearDownLeft() {
        if (m_linear_left.GetAnalogPosition() <= LINEAR_MIN_TRAVEL) {
            m_linear_left.SetDutyCycle(0);
            return;
        }
        linear_actuator_state_left = LinearActuatorStateLeft::TravelingDown;
        m_linear_left.SetDutyCycle(1.0);
    }
    
    void Digging::stopLinearActuatorMotorsRight() {
        if(m_linear_left.GetDutyCycle() != 0 || linear_actuator_state_right == LinearActuatorStateRight::Raised){
            // RCLCPP_INFO(get_logger(), "STOPPING LINEAR ACTUATOR SPARKMAXES Right!");
            m_linear_right.SetDutyCycle(0.0);
        }
        actuators_running_right = false;
    }
    
    void Digging::stopLinearActuatorMotorsLeft() {
        if(m_linear_right.GetDutyCycle() != 0 || linear_actuator_state_left == LinearActuatorStateLeft:: Raised){
            // RCLCPP_INFO(get_logger(), "STOPPING LINEAR ACTUATOR SPARKMAXES Left!");
            m_linear_left.SetDutyCycle(0.0);
        }
        actuators_running_left = false;
    }

    LinearActuatorStateRight Digging::getLinearActuatorStateRight(){ return linear_actuator_state_right; }
    LinearActuatorStateLeft Digging::getLinearActuatorStateLeft(){ return linear_actuator_state_left; }

    void Digging::checkLinearActuatorLimits() {
        double positionRight = m_linear_right.GetAnalogPosition();
        double positionLeft = m_linear_left.GetAnalogPosition();

        if (positionLeft >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND)) {
            if (linear_actuator_state_left != LinearActuatorStateLeft::TravelingDown) {
                linear_actuator_state_left = LinearActuatorStateLeft::Raised;
                commandStopLeft();
            }
        }
        if (positionLeft <= LINEAR_MIN_TRAVEL) {
            if (linear_actuator_state_left != LinearActuatorStateLeft::TravelingUp) {
                linear_actuator_state_left = LinearActuatorStateLeft::Lowered;
                commandStopLeft();
            }
        }
        if (positionRight >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND)) {
            if (linear_actuator_state_right != LinearActuatorStateRight::TravelingDown) {
                linear_actuator_state_right = LinearActuatorStateRight::Raised;
                commandStopRight();
            }
        }
        if (positionRight <= LINEAR_MIN_TRAVEL) {
            if (linear_actuator_state_right != LinearActuatorStateRight::TravelingUp) {
                linear_actuator_state_right = LinearActuatorStateRight::Lowered;
                commandStopRight();
            }
        }
        if(linear_actuator_state_left == LinearActuatorStateLeft:: Raised){
            stopLinearActuatorMotorsRight();
        }
        if(linear_actuator_state_right == LinearActuatorStateRight:: Raised){
            stopLinearActuatorMotorsLeft();
        }
    }


    void Digging::initMotors() {
        try {
            RCLCPP_INFO(get_logger(), "Configuring Digging Subsystem Motors");
            // Belt motors
            m_belt_left.SetIdleMode(IdleMode::kCoast);
            m_belt_left.SetMotorType(MotorType::kBrushless);
            m_belt_left.SetDutyCycle(0.0);
            m_belt_left.ClearStickyFaults();
            m_belt_left.ResetFaults();
            m_belt_left.SetPeriodicStatus3Period(0);
            m_belt_left.SetPeriodicStatus4Period(0);
            m_belt_left.BurnFlash();
            
            m_belt_right.SetIdleMode(IdleMode::kCoast);
            m_belt_right.SetMotorType(MotorType::kBrushless);
            m_belt_right.SetDutyCycle(0.0);
            m_belt_right.ClearStickyFaults();
            m_belt_right.ResetFaults();
            m_belt_right.SetPeriodicStatus3Period(0);
            m_belt_right.SetPeriodicStatus4Period(0);
            m_belt_right.BurnFlash();
            
            // Linear actuators
            m_linear_left.SetIdleMode(IdleMode::kCoast);
            m_linear_left.SetMotorType(MotorType::kBrushed);
            m_linear_left.SetSensorType(SensorType::kEncoder);
            m_linear_left.SetDutyCycle(0.0);
            m_linear_left.ClearStickyFaults();
            m_linear_left.ResetFaults();
            m_linear_left.SetPeriodicStatus1Period(0);
            m_linear_left.SetPeriodicStatus2Period(0);
            m_linear_left.SetP(0, 0.00000);
            m_linear_left.SetI(0, 0);
            m_linear_left.SetD(0, 0.00000);
            m_linear_left.SetF(0, 0.00000);
            m_linear_left.BurnFlash();
            
            m_linear_right.SetIdleMode(IdleMode::kCoast);
            m_linear_right.SetMotorType(MotorType::kBrushed);
            m_linear_right.SetSensorType(SensorType::kEncoder);
            m_linear_right.SetDutyCycle(0.0);
            m_linear_right.ClearStickyFaults();
            m_linear_right.ResetFaults();
            m_linear_right.SetPeriodicStatus1Period(0);
            m_linear_right.SetPeriodicStatus2Period(0);
            m_linear_right.SetP(0, 0.00000);
            m_linear_right.SetI(0, 0);
            m_linear_right.SetD(0, 0.00000);
            m_linear_right.SetF(0, 0.00000);
            m_linear_right.BurnFlash();
            
            // Leadscrews
            m_leadscrew_left.SetIdleMode(IdleMode::kCoast);
            m_leadscrew_left.SetMotorType(MotorType::kBrushless);
            m_leadscrew_left.ClearStickyFaults();
            m_leadscrew_left.SetDutyCycle(0.0);
            m_leadscrew_left.ResetFaults();
            m_leadscrew_left.SetPeriodicStatus3Period(0);
            m_leadscrew_left.SetPeriodicStatus4Period(0);
            m_leadscrew_left.BurnFlash();

            m_leadscrew_right.SetIdleMode(IdleMode::kCoast);
            m_leadscrew_right.SetMotorType(MotorType::kBrushless);
            m_leadscrew_right.ClearStickyFaults();
            m_leadscrew_right.SetDutyCycle(0.0);
            m_leadscrew_right.ResetFaults();
            m_leadscrew_right.SetPeriodicStatus3Period(0);
            m_leadscrew_right.SetPeriodicStatus4Period(0);
            m_leadscrew_right.BurnFlash();

            RCLCPP_INFO(get_logger(), "Digging Subsystem Motors configured successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to configure Digging Subsystem motors: %s", e.what());
        }
    }

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Digging>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}