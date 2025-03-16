/**
 * @brief Temple Lunabotics Digging Subsystem
 * @todo Digging Belt - PID & Sensor Diagnostics via Timer
 */
#include "core.hpp"
#include "Digging.hpp"

Digging::Digging()
    : Node("digging")
    , m_belt_left("can0", BELT_1_CAN_ID)
    , m_belt_right("can0", BELT_2_CAN_ID)
    , m_linear_left("can0", LINEAR_LEFT_CAN_ID)
    , m_linear_right("can0", LINEAR_RIGHT_CAN_ID)
    , m_leadscrew_left("can0", LEADSCREW_1_CAN_ID)
    , m_leadscrew_right("can0", LEADSCREW_2_CAN_ID)
{

    joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Digging::joy_callback_digging, this, std::placeholders::_1));
    initMotors();    
    configureLimitSwitches();
    
    m_leadscrew_right.SetFollowerID(LEADSCREW_1_CAN_ID);
    m_leadscrew_right.SetFollowerConfig(1);  // follower mode
    m_leadscrew_left.BurnFlash();
    m_leadscrew_right.BurnFlash();
    
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

        double dig_forward = joy_msg->buttons[3];
        double dig_reverse = joy_msg->buttons[0];
        
        double extend_leadscrew = joy_msg->buttons[9];
        double retract_leadscrew = joy_msg->buttons[10];
        
        if(dig_forward) { setBeltSpeedForward(dig_forward); }
        if(dig_reverse) { setBeltSpeedReverse(dig_reverse); }
        
        if(extend_leadscrew) { setLeadscrewSpeed(0.5); }
        if(retract_leadscrew) { setLeadscrewSpeed(-0.5); }

        double raise_linear_actuators = joy_msg->buttons[2];
        double lower_linear_actuators = joy_msg->buttons[1];
        if(raise_linear_actuators){ commandUp(); }
        if(lower_linear_actuators){ commandDown(); }
        
    }

    /**
     * @brief Our Digging Belt spins like a smily face 
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
        auto position = m_leadscrew_left.GetPosition();
        
        if (position <= LEADSCREW_MAX_ERROR && speed < 0) {
            RCLCPP_WARN(get_logger(), "At bottom limit, cannot move down further");
            return;
        }
        
        if (position >= LEADSCREW_MAX_TRAVEL - LEADSCREW_MAX_ERROR && speed > 0) {
            RCLCPP_WARN(get_logger(), "At top limit, cannot move up further");
            return;
        }

        leadscrew_state = LeadscrewState::Traveling;
        m_leadscrew_left.SetDutyCycle(speed);  // Leadscrew2 follows automatically due to follower config
    }

    LeadscrewState Digging::getLeadscrewState() {
        return leadscrew_state;
    }

    // LINEAR ACTUATORS

    void Digging::periodicLinearActuatorCheck(){
        checkLinearActuatorLimits();
    }



    void Digging::linearUp(){
        if (m_linear_left.GetPosition() >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND) || linear_actuator_state == LinearActuatorState::Lowered) {
            stopLinearActuatorMotors();
            return;
        }
        if (m_linear_right.GetPosition() >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND) || linear_actuator_state == LinearActuatorState::Lowered) {
            stopLinearActuatorMotors();
            return;
        }

        linear_actuator_state = LinearActuatorState::TravelingUp;
        m_linear_left.SetDutyCycle(-1.0);
        m_linear_right.SetDutyCycle(-1.0);
    }

    void Digging::linearDown(){
        if (m_linear_left.GetPosition() <= LINEAR_MIN_TRAVEL || linear_actuator_state == LinearActuatorState::Lowered){ return; }
        if (m_linear_right.GetPosition() <= LINEAR_MIN_TRAVEL || linear_actuator_state == LinearActuatorState::Lowered){ return; }
        
        linear_actuator_state = LinearActuatorState::TravelingDown;
        m_linear_left.SetDutyCycle(1.0);
        m_linear_right.SetDutyCycle(1.0);
    }

    void Digging::stopLinearActuatorMotors(){
        RCLCPP_INFO(get_logger(), "STOPPING LINEAR ACTUATOR SPARKMAXES!");
        m_linear_left.SetDutyCycle(0.0);
        m_linear_right.SetDutyCycle(0.0);
        
    }                            

    double Digging::getLinearActuatorLeftPosition(){ return m_linear_left.GetPosition(); }
    double Digging::getLinearActuatorRightPosition(){  return m_linear_right.GetPosition(); }

    void Digging::commandUp(){
        linearUp();
        linear_actuator_state = LinearActuatorState::Commanded;
    }
    void Digging::commandDown(){
        linearDown();
        linear_actuator_state = LinearActuatorState::Commanded;
    }
    
    void Digging::commandStop(){ stopLinearActuatorMotors(); }

    LinearActuatorState Digging::getLinearActuatorState(){ return linear_actuator_state; }

    void Digging::checkLinearActuatorLimits(){
        RCLCPP_INFO(
        get_logger(),"CURRENT LINEAR ACTUATOR POSITONS = LEFT %lf\tRIGHT %lf\n ", 
        m_linear_left.GetPosition(), 
        m_linear_right.GetPosition()
        );

        if (linear_actuator_state != LinearActuatorState::Raised && linear_actuator_state != LinearActuatorState::TravelingDown 
                && m_linear_left.GetPosition() >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND)) {
            linear_actuator_state = LinearActuatorState::Raised;
            commandStop();
        }
        if (linear_actuator_state != LinearActuatorState::Lowered && linear_actuator_state != LinearActuatorState::TravelingUp
                && m_linear_left.GetPosition() <= LINEAR_MIN_TRAVEL) {
            linear_actuator_state = LinearActuatorState::Lowered;
            commandStop();
        }
        if (linear_actuator_state != LinearActuatorState::Raised && linear_actuator_state != LinearActuatorState::TravelingDown
                && m_linear_right.GetPosition() >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND)) {
            linear_actuator_state = LinearActuatorState::Raised;
            commandStop();
        }
        if (linear_actuator_state != LinearActuatorState::Lowered && linear_actuator_state != LinearActuatorState::TravelingUp
                && m_linear_right.GetPosition() <= LINEAR_MIN_TRAVEL) {
            linear_actuator_state = LinearActuatorState::Lowered;
            commandStop();
        }
    }


    void Digging::initMotors() {
        try {
            RCLCPP_INFO(get_logger(), "Configuring Digging Subsystem Motors");
            
            // Belt motors
            m_belt_left.SetIdleMode(IdleMode::kCoast);
            m_belt_left.SetMotorType(MotorType::kBrushless);
            m_belt_left.SetDutyCycle(0.0);
            m_belt_left.BurnFlash();
            
            m_belt_right.SetIdleMode(IdleMode::kCoast);
            m_belt_right.SetMotorType(MotorType::kBrushless);
            m_belt_right.SetDutyCycle(0.0);
            m_belt_right.BurnFlash();
            
            // Linear actuators
            m_linear_left.SetIdleMode(IdleMode::kCoast);
            m_linear_left.SetMotorType(MotorType::kBrushless);
            m_linear_left.SetDutyCycle(0.0);
            m_linear_left.BurnFlash();
            
            m_linear_right.SetIdleMode(IdleMode::kCoast);
            m_linear_right.SetMotorType(MotorType::kBrushless);
            m_linear_right.SetDutyCycle(0.0);
            m_linear_right.BurnFlash();
            
            // Leadscrews
            m_leadscrew_left.SetIdleMode(IdleMode::kCoast);
            m_leadscrew_left.SetMotorType(MotorType::kBrushless);
            m_leadscrew_left.SetDutyCycle(0.0);
            
            m_leadscrew_right.SetIdleMode(IdleMode::kCoast);
            m_leadscrew_right.SetMotorType(MotorType::kBrushless);
            m_leadscrew_right.SetDutyCycle(0.0);
            
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