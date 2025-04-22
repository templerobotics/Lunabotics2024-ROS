/**
 * @brief Temple Lunabotics Digging Subsystem
 * @todo Digging Belt - PID & Sensor Diagnostics via Timer
 * @todo Leadscrews start at 0 , Actuators are down. Initializatin was leadscrews down for 2 seconds then came up till hit top limit switch even if at 0
 */
#include "core.hpp"
#include "Digging.hpp"

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
            state_pub = create_publisher<std_msgs::msg::String>("leadscrew/state", 10);
            timer_diagnostics = create_wall_timer(std::chrono::milliseconds(500), std::bind(&Digging::periodic, this));
            timer_linear_actuators = create_wall_timer(std::chrono::milliseconds(500), std::bind(&Digging::periodicLinearActuatorCheck, this));

            RCLCPP_INFO(this->get_logger(), "Digging Subsystem Successfully Initialized!");
            
        }


    /**
     * @brief Interprets XBOX Joystick digging commands
     * @todo Implement more logic for this function
     */
    void Digging::joy_callback_digging(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {   
        m_belt_left.Heartbeat();
        m_belt_right.Heartbeat();
        m_linear_left.Heartbeat();
        m_linear_right.Heartbeat();
        m_leadscrew_left.Heartbeat();
        m_leadscrew_right.Heartbeat();
        
        // m_linear_left.SetDutyCycle(1.0);
        // rclcpp::sleep_for(std::chrono::seconds(2));
        // m_linear_left.SetDutyCycle(0.0);
        // rclcpp::sleep_for(std::chrono::seconds(2));
        // m_linear_left.SetDutyCycle(-1.0);
        // rclcpp::sleep_for(std::chrono::seconds(2));
        // m_linear_left.SetDutyCycle(0.0);

        
        // RCLCPP_INFO(this->get_logger(), "%lf", extend_leadscrew);
        bool current_a = joy_msg->buttons[0];
        if (current_a && !last_a_state) {  // Button A just pressed
            if (belt_running) {
                stopDiggingBeltMotors();
            } else {
                setBeltSpeedForward(1.0);  // Or however fast you want
            }
        }
        last_a_state = current_a;

        // Debounced toggle logic for Y (button 3)
        bool current_y = joy_msg->buttons[3];
        if (current_y && !last_y_state) {  // Button Y just pressed
            if (belt_running) {
                stopDiggingBeltMotors();
            } else {
                setBeltSpeedReverse(1.0);  // Opposite direction
            }
        }
        last_y_state = current_y;

        double rightTrigger = joy_msg->axes[5];
        double leftTrigger = joy_msg->axes[4];
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
		setLeadscrewSpeed(std::clamp(leadscrewSpeed, -1.0, 0.5));

        // bool current_b = joy_msg->buttons[1];
        // if(current_b){
        //     commandUp();
        // }
        // bool current_x = joy_msg->buttons[2];
        // if(current_x){
        //     commandDown();
        // }
        // bool current_b = joy_msg->buttons[1];
        // if (current_b && !last_b_state) {
        //     if (actuators_running) {
        //         commandStopLeft();
        //         commandStopRight();
        //     } else {
        //         commandUp();
        //         actuators_running = true;
        //     }
        // }
        // last_b_state = current_b;

        // bool current_x = joy_msg->buttons[2];
        // if (current_x && !last_x_state) {
        //     if (actuators_running) {
        //         commandStopLeft();
        //         commandStopRight();
        //     } else {
        //         commandDown();
        //         actuators_running = true;
        //     }
        // }
        // last_x_state = current_x;

        bool current_b = joy_msg->buttons[1];  // B = up
        if (current_b && !last_b_state) {
            if (actuators_running) {
                commandStop();
            } else {
                commandUp();
            }
        }
        last_b_state = current_b;

        bool current_x = joy_msg->buttons[2];  // X = down
        if (current_x && !last_x_state) {
            if (actuators_running) {
                commandStop();
            } else {
                commandDown();
            }
        }
        last_x_state = current_x;
    }

    /**
     * @brief The Digging Belt spins like a smily face 
     */
    void Digging::setBeltSpeedForward(double speed) {
        m_belt_left.SetDutyCycle(speed);
        m_belt_right.SetDutyCycle(-1*speed);
        belt_running = true;
    }
    
    void Digging::setBeltSpeedReverse(double speed) {
        m_belt_left.SetDutyCycle(-1*speed);
        m_belt_right.SetDutyCycle(speed);
        belt_running = true;
    }

    void Digging::stopDiggingBeltMotors() {
        RCLCPP_INFO(get_logger(), "STOPPING DIGGING BELT MOTORS!");
        m_belt_left.SetDutyCycle(0.0);
        m_belt_right.SetDutyCycle(0.0);
        belt_running = false;
    }
    void Digging::stopLeadScrew(){
        m_leadscrew_left.SetDutyCycle(0); 
        m_leadscrew_right.SetDutyCycle(0);
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
        // checkLeadscrewLimits();
        publishState();
    }

    void Digging::checkLeadscrewLimits() {
        auto position = m_leadscrew_left.GetPosition();
        uint16_t faults1 = m_leadscrew_left.GetFaults();
        uint16_t faults2 = m_leadscrew_right.GetFaults();

        bool topLimit = checkFault(faults1, FaultBits::kHardLimitFwd) || checkFault(faults2, FaultBits::kHardLimitFwd);
        bool bottomLimit = checkFault(faults1, FaultBits::kHardLimitRev) || checkFault(faults2, FaultBits::kHardLimitRev);

        if (topLimit || bottomLimit) {
            RCLCPP_DEBUG(get_logger(), "Limit switch triggered - Top: %s, Bottom: %s", topLimit ? "true" : "false", bottomLimit ? "true" : "false");
        }

        // Update state based on position and limits
        if (leadscrew_state != LeadscrewState::FullExtended && position >= LEADSCREW_MAX_TRAVEL - LEADSCREW_MAX_ERROR) {
            leadscrew_state = LeadscrewState::FullExtended;
            RCLCPP_INFO(get_logger(), "Leadscrew reached full extension");
        }
        else if (leadscrew_state != LeadscrewState::Retracted && (position < LEADSCREW_MAX_ERROR || topLimit)) {
            leadscrew_state = LeadscrewState::Retracted;
            RCLCPP_INFO(get_logger(), "Leadscrew fully retracted");
        }
        else if (leadscrew_state != LeadscrewState::Extended && position > LEADSCREW_MAX_ERROR && std::abs(position - LEADSCREW_MAX_TRAVEL) >= LEADSCREW_MAX_ERROR) {
            leadscrew_state = LeadscrewState::Extended;
        }
    }

    void Digging::publishState() {
        auto msg = std_msgs::msg::String();
        msg.data = stateToString(leadscrew_state);
        state_pub->publish(msg);
    }

    std::string Digging::stateToString(LeadscrewState state) {
        switch (state) {
            case LeadscrewState::Extended: return "Extended";
            case LeadscrewState::Retracted: return "Retracted";
            case LeadscrewState::Traveling: return "Traveling";
            case LeadscrewState::FullExtended: return "FullExtended";
            case LeadscrewState::GivenCommand: return "GivenCommand";
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
        // RCLCPP_INFO(get_logger(), "Leadscrew position %lf", position);
        // RCLCPP_INFO(get_logger(), "Leadscrew speed %lf", speed);
        
        if (position <= LEADSCREW_MAX_ERROR && speed < 0) {
            RCLCPP_WARN(get_logger(), "At bottom limit, cannot move down further");
            return;
        }
        
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



    void Digging::linearUp(){
        // if (m_linear_right.GetAnalogPosition() >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND)) {
        //     m_linear_right.SetDutyCycle(0);
        //     return;
        // }
        // if (m_linear_left.GetAnalogPosition() >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND)) {
        //     m_linear_left.SetDutyCycle(0);
        //     return;
        // }
        // linear_actuator_state_right = LinearActuatorStateRight::TravelingUp;
        // linear_actuator_state_left = LinearActuatorStateLeft::TravelingUp;
        m_linear_left.SetDutyCycle(-1);
        m_linear_right.SetDutyCycle(-1);
    }

    void Digging::linearDown(){
        // if (m_linear_left.GetAnalogPosition() <= LINEAR_MIN_TRAVEL){ return; }
        // if (m_linear_right.GetAnalogPosition() <= LINEAR_MIN_TRAVEL){ return; }
        // RCLCPP_INFO(get_logger(), "Setting actuator speed to 1");
        // linear_actuator_state_right = LinearActuatorStateRight::TravelingDown;
        // linear_actuator_state_left = LinearActuatorStateLeft::TravelingDown;
        m_linear_left.SetDutyCycle(1.0);
        m_linear_right.SetDutyCycle(1.0);
    }  
    void Digging::stopLinearActuatorMotorsRight(){
        RCLCPP_INFO(get_logger(), "STOPPING LINEAR ACTUATOR SPARKMAXES Right!");
        m_linear_right.SetDutyCycle(0.0);
    }      
    void Digging::stopLinearActuatorMotorsLeft(){
        RCLCPP_INFO(get_logger(), "STOPPING LINEAR ACTUATOR SPARKMAXES Left!");
        m_linear_left.SetDutyCycle(0.0);
    }                            

    double Digging::getLinearActuatorLeftPosition(){ return m_linear_left.GetAnalogPosition(); }
    double Digging::getLinearActuatorRightPosition(){  return m_linear_right.GetAnalogPosition(); }

    void Digging::commandUp(){
        actuators_running = true;
        linearUp();
        // linear_actuator_state_right = LinearActuatorStateRight::Commanded;
        // linear_actuator_state_left = LinearActuatorStateLeft::Commanded;
    }
    void Digging::commandDown(){
        actuators_running = true;
        linearDown();
        // linear_actuator_state_right = LinearActuatorStateRight::Commanded;
        // linear_actuator_state_left = LinearActuatorStateLeft::Commanded;
    }
    void Digging::commandStop(){
        m_linear_left.SetDutyCycle(0);
        m_linear_right.SetDutyCycle(0);
        actuators_running = false;
    }
    void Digging::commandStopRight(){ 
        stopLinearActuatorMotorsRight(); 
    }
    void Digging::commandStopLeft(){
        stopLinearActuatorMotorsLeft();
    }

    LinearActuatorStateRight Digging::getLinearActuatorStateRight(){ return linear_actuator_state_right; }
    LinearActuatorStateLeft Digging::getLinearActuatorStateLeft(){ return linear_actuator_state_left; }

    void Digging::checkLinearActuatorLimits(){
        RCLCPP_INFO(
        get_logger(),"CURRENT LINEAR ACTUATOR POSITONS Analog = LEFT %lf\tRIGHT %lf\n ", 
        m_linear_left.GetAnalogPosition(), 
        m_linear_right.GetAnalogPosition()
        );

        if (linear_actuator_state_right != LinearActuatorStateRight::Raised && linear_actuator_state_right != LinearActuatorStateRight::TravelingDown 
                && m_linear_right.GetAnalogPosition() >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND)) {
            linear_actuator_state_right = LinearActuatorStateRight::Raised;
            commandStopRight();
        }
        // if (linear_actuator_state_right != LinearActuatorStateRight::Lowered && linear_actuator_state_right != LinearActuatorStateRight::TravelingUp
        //     && m_linear_right.GetAnalogPosition() <= LINEAR_MIN_TRAVEL) {
        //     linear_actuator_state_right = LinearActuatorStateRight::Lowered;
        //     commandStopRight();
        // }
        if (linear_actuator_state_left != LinearActuatorStateLeft::Raised && linear_actuator_state_left != LinearActuatorStateLeft::TravelingDown
                && m_linear_left.GetAnalogPosition() >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND)) {
            linear_actuator_state_left = LinearActuatorStateLeft::Raised;
            commandStopLeft();
        }
        
        // if (linear_actuator_state_left != LinearActuatorStateLeft::Lowered && linear_actuator_state_left != LinearActuatorStateLeft::TravelingUp
        //         && m_linear_left.GetAnalogPosition() <= LINEAR_MIN_TRAVEL) {
        //     linear_actuator_state_left = LinearActuatorStateLeft::Lowered;
        //     commandStopLeft();
        // }
    }


    void Digging::initMotors() {
        try {
            RCLCPP_INFO(get_logger(), "Configuring Digging Subsystem Motors");
            // Belt motors
            m_belt_left.SetIdleMode(IdleMode::kCoast);
            m_belt_left.SetMotorType(MotorType::kBrushless);
            m_belt_left.SetDutyCycle(0.0);
            m_belt_left.ClearStickyFaults();
            m_belt_left.BurnFlash();
            
            m_belt_right.SetIdleMode(IdleMode::kCoast);
            m_belt_right.SetMotorType(MotorType::kBrushless);
            m_belt_right.SetDutyCycle(0.0);
            m_belt_right.ClearStickyFaults();
            m_belt_right.BurnFlash();
            
            // Linear actuators
            // m_linear_left.SetIdleMode(IdleMode::kCoast);
            m_linear_left.SetMotorType(MotorType::kBrushed);
            m_linear_left.SetSensorType(SensorType::kEncoder);
            m_linear_left.SetDutyCycle(0.0);
            m_linear_left.ClearStickyFaults();
            m_linear_left.BurnFlash();
            
            // m_linear_right.SetIdleMode(IdleMode::kCoast);
            m_linear_right.SetMotorType(MotorType::kBrushed);
            m_linear_right.SetSensorType(SensorType::kEncoder);
            m_linear_right.SetDutyCycle(0.0);
            m_linear_right.ClearStickyFaults();
            m_linear_right.BurnFlash();
            
            // Leadscrews
            m_leadscrew_left.SetIdleMode(IdleMode::kCoast);
            m_leadscrew_left.SetMotorType(MotorType::kBrushless);
            m_leadscrew_left.ClearStickyFaults();
            m_leadscrew_left.SetDutyCycle(0.0);
            m_leadscrew_left.BurnFlash();

            m_leadscrew_right.SetIdleMode(IdleMode::kCoast);
            m_leadscrew_right.SetMotorType(MotorType::kBrushless);
            m_leadscrew_right.ClearStickyFaults();
            m_leadscrew_right.SetDutyCycle(0.0);
            m_leadscrew_right.BurnFlash();

            // m_leadscrew_left.SetDutyCycle(-1);
            // m_leadscrew_right.SetDutyCycle(-1);
            // rclcpp::sleep_for(std::chrono::seconds(2));
            // m_leadscrew_left.SetDutyCycle(1);
            // m_leadscrew_right.SetDutyCycle(1);
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