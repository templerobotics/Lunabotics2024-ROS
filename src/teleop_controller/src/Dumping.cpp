/**
 * @brief Node pertaining to our Robot's dumping conveyor belt
 * @note Linear Actuators : Move the digging belt from : Rest angle ----> Digging angle ---> back to rest angle
 * @note Dumping Servo --> Hold a button down to hold the dumping door OPEN, let go of the button & the latch closes
 * @note Dumping Servos Connection Mechanism : 
 * @note getPosition() interval of [0,1]
 * @note setSpeed() interval of [-1,1]
 * @note Dumping Servos connected to roboRIO last year ---> Use Raspberry PIS
 * @todo ARDUINO INTEGRATION - 18 March 2025
 */
#include "core.hpp"
#include "Dumping.hpp"
#include <fcntl.h>

Dumping::Dumping()
    : Node("dumping_conveyor_belt")
    , m_dumping_left("can0", DUMPING_LEFT_CAN_ID)
    , m_dumping_right("can0", DUMPING_RIGHT_CAN_ID)
{ 
    initMotors();
    RCLCPP_INFO(this->get_logger(), "Dumping Subsystem ready to go!\n");
}


    void Dumping::cmd_open_dumplatch(double cmd_open_dumplatch){
        int arduino_fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
        if (arduino_fd < 0) {
            fprintf(stderr,"Arduino FD cant be negative! Value is [%d]",arduino_fd);
        }
        if(cmd_open_dumplatch) {write(arduino_fd,"o",1);}
    }      
    
    void Dumping::cmd_close_dumplatch(double cmd_close_dumplatch){
        int arduino_fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
        if (arduino_fd < 0) {
            fprintf(stderr,"Arduino FD cant be negative! Value is [%d]",arduino_fd);
        }
        if(cmd_close_dumplatch) {write(arduino_fd,"c",1);}
    }      
    
    /**
     * @brief
     * @var conveyor_belt_forward
     *  XBOX DPAD-RIGHT
     * @var conveyor_belt_backwards
     *  XBOX DPAD-LEFT
     * @var open_dump_latch
     *  XBOX DPAD_UP
     * @var close_dump_latch 
     * XBOX DPAD-DOWN
     */
    void Dumping::joy_callback_dumping(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
        double conveyor_belt_forward = joy_msg->buttons[14]; //dpad right
        double conveyor_belt_reverse = joy_msg->buttons[13]; //dpad left
        double open_dump_latch = joy_msg->buttons[11]; // d-pad up
        double close_dump_latch = joy_msg->buttons[12]; // d-pad down

        if(open_dump_latch){ cmd_open_dumplatch(open_dump_latch); } 
        if(close_dump_latch){ cmd_close_dumplatch(close_dump_latch); }
        
    }

    void move_belt_forward(){

    }

    void move_belt_reverse(){

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