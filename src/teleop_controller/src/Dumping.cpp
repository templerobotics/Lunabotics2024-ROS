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

    /*Change port to correct arduino port. Also, I have no idea if this code works or not*/
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
    
  
    void Dumping::joy_callback_dumping(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
        double conveyor_belt_control = joy_msg->axes[6]; 
        double dump_latch_control = joy_msg->axes[7]
        if(conveyor_belt_control < 0){ move_belt_forward();}                    // belt "forward"
        if(conveyor_belt_control > 0){ move_belt_reverse();  }                  // belt "backward/reverse"
        if(dump_latch_control < 0){ cmd_close_dumplatch(dump_latch_control); }  // close dump latch
        if(dump_latch_control > 0){ cmd_open_dumplatch(dump_latch_control); }   // open dump latch
        
    }
    // Conveyor Belt is actuated by both sparkmaxes regardless of forward or backwards. 
    // Duty cycle being (+) or (-) is irrelevant I think. Need real-life testing and observation
    void move_belt_forward(){
        m_dumping_left.SetDutyCycle(1.0);
        m_dumping_right.SetDutyCycle(1.0);
    }

    void move_belt_reverse(){
        m_dumping_left.SetDutyCycle(-1.0);
        m_dumping_right.SetDutyCycle(-1.0);
    }

    /*
    void getPostiion(){}
    void setSpeed(){}
    */

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