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
   
    joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&DiggingBelt::joy_callback_digging, this, std::placeholders::_1));
    init_motors();


    RCLCPP_INFO(this->get_logger(), "Digging Subsystem Successfully Initialized!");
}


private:
   
    void Digging::init_motors() {
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

    /**
     * @brief Digging Subsystem Implementation
     * @details Robot is enabled in DrivebaseControl.cpp, so skipp the check
     * @remark Could use OOP by implementing a Robot Base class, but who cares
     * @remark Axes[1] & Axes[2] Used for Drivebase
     */
    void Digging::joy_callback_digging(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
        assert(joy_msg->data != NULL);
        SparkMax::Heartbeat();


        
    }



    /*
        Left Trigger : joy_msg->axes[4] 
        Right Trigger : joy_msg->axes[5] 
        msg->buttons[5];    Right Bumper 
        msg->buttons[4];    Left Bumper 
        msg->buttons[1];    B Button
        msg->buttons[2];    X Button  
            
    */
    bool x_button = false;          //raise linear actuator
    bool b_button = false;          // lower linear actuator
    bool left_bumper = false;       // Extend Leadscrew
    bool right_bumper = false;      //Retract Leadscrew
    bool left_trigger = false;      // Increase Speed of Leadscrew extension
    bool right_trigger = false;     // Increase Speed of Leadscrew retraction




int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Digging>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



