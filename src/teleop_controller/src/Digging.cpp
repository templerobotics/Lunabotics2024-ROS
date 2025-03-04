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

    RCLCPP_INFO(this->get_logger(), "Digging Subsystem Successfully Initialized!");
}



    /**
     * @brief Digging Subsystem Implementation
     * @details Robot is enabled in DrivebaseControl.cpp, so skip the check. Could make it a global variable. If I do, TEST IT
     * @details Redundant software call to SparkmMax::Heartbeat() because redundancy is good
     * @remark Could use OOP by implementing a Robot Base class, but bigger fish to fry
     * @remark Axes[1] & Axes[2] Used for Drivebase
     */
    void Digging::joy_callback_digging(const sensor_msgs::msg::Joy::SharedPtr joy_msg){   

        SparkMax::Heartbeat();
        double dig_forward = joy_msg->buttons[3];
        double dig_reverse = joy_msg->buttons[0];

        double raise_linear_actuators = joy_msg->buttons[2];
        double lower_linear_actuators = joy_msg->buttons[1];

        double extend_leadscrew = joy_msg->buttons[9];
        double retract_leadscrew = joy_msg->buttons[10];

        if(dig_forward){setBeltSpeedForward(dig_forward);}
        if(dig_reverse){setBeltSpeedReverse(dig_reverse);}

    }
    
    //===========================
    // Digging Belt Functions
    //===========================


    void Digging::setBeltSpeedForward(double speed){
        m_belt_left.SetDutyCycle(speed);
        belt_running = true;
    }
    
    void Digging::setBeltSpeedReverse(double speed){
        m_belt_right.SetDutyCycle(-1*speed);
        belt_running = true;
    }

    void Digging::stopMotors(){
        RCLCPP_INFO(get_logger(),"STOPPING DIGGING BELT MOTORS!");
        m_belt_left.SetDutyCycle(0.0);
        m_belt_right.SetDutyCycle(0.0);
        belt_running = false;
    }
    
    //===========================
    // Leadscrew & Limit Switch Functions
    //===========================



    void Digging::initMotors() {
        try {
            RCLCPP_INFO(get_logger(), "Configuring Digging Subsystem Motors");
            
            m_belt_left.SetIdleMode(IdleMode::kCoast);
            m_belt_left.SetMotorType(MotorType::kBrushless);
            m_belt_left.SetDutyCycle(0.0);
            m_belt_left.BurnFlash();
            
            m_belt_right.SetIdleMode(IdleMode::kCoast);
            m_belt_right.SetMotorType(MotorType::kBrushless);
            m_belt_right.SetDutyCycle(0.0);
            m_belt_right.BurnFlash();
            
            m_linear_left.SetIdleMode(IdleMode::kCoast);
            m_linear_left.SetMotorType(MotorType::kBrushless);
            m_linear_left.SetDutyCycle(0.0);
            m_linear_left.BurnFlash();
            
            m_linear_right.SetIdleMode(IdleMode::kCoast);
            m_linear_right.SetMotorType(MotorType::kBrushless);
            m_linear_right.SetDutyCycle(0.0);
            m_linear_right.BurnFlash();
            
            m_leadscrew_left.SetIdleMode(IdleMode::kCoast);
            m_leadscrew_left.SetMotorType(MotorType::kBrushless);
            m_leadscrew_left.SetDutyCycle(0.0);
            m_leadscrew_left.BurnFlash();
            
            m_leadscrew_right.SetIdleMode(IdleMode::kCoast);
            m_leadscrew_right.SetMotorType(MotorType::kBrushless);
            m_leadscrew_right.SetDutyCycle(0.0);
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



