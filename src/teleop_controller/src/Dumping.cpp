/**
 * @brief Node pertaining to our Robot's dumping conveyor belt
 * @note Linear Actuators : Move the digging belt from : Rest angle ----> Digging angle ---> back to rest angle
 * @note Dumping Servo --> Hold a button down to hold the dumping door OPEN, let go of the button & the latch closes
 * @note Dumping Servos Connection Mechanism : 
 * @note getPosition() interval of [0,1]
 * @note setSpeed() interval of [-1,1]
 * @note Dumping Servos connected to roboRIO last year ---> Use Raspberry PIS
 * @note Visualization Notes --> In Obsidian Notes
 */
#include "core.hpp"
#include "Dumping.hpp"

Dumping::Dumping()
    : Node("dumping_conveyor_belt")
    , m_dumping_left("can0", DUMPING_LEFT_CAN_ID)
    , m_dumping_right("can0", DUMPING_RIGHT_CAN_ID)
{ 
    initMotors();
    RCLCPP_INFO(this->get_logger(), "Dumping Subsystem ready to go!\n");
}
    /**
     * @brief
     * @var conveyor_belt_forward
     *  XBOX DPAD-UP
     * @var conveyor_belt_backwards
     *  XBOX DPAD-DOWN
     * @var activate_dump_latch
     *  XBOX DPAD_RIGHT
     */
    void Dumping::joy_callback_dumping(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
        double conveyor_belt_forward = joy_msg->buttons[11];
        double conveyor_belt_reverse = joy_msg->buttons[12];
        double activate_dump_latch = joy_msg->buttons[14];
    }

    void move_belt_forward(){

    }

    void move_belt_reverse(){

    }

    /**
     * @brief Opens dumping latch at the back of the robot dumping conveyor belt
     * @details 2 dumping servos 
     */
    void open_dumping_latch(){

    }
    
    void getPostiion(){

    }

    void setSpeed(){

    }


    void Dumping::initMotors(){
        m_dumping_left.SetIdleMode(IdleMode::kCoast);
        m_dumping_left.SetMotorType(MotorType::kBrushless);
        m_dumping_left.SetDutyCycle(0.0);
        m_dumping_left.BurnFlash();
        
        m_dumping_right.SetIdleMode(IdleMode::kCoast);
        m_dumping_right.SetMotorType(MotorType::kBrushless);
        m_dumping_right.SetDutyCycle(0.0);
        m_dumping_right.BurnFlash();
        
        RCLCPP_INFO(get_logger(), "Dumping Subsystem Motors configured successfully");
    }



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dumping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}