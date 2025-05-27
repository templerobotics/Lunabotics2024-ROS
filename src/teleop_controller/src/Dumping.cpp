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
        
        joy_sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Dumping::joy_callback_dumping, this, std::placeholders::_1));
        initMotors();
        RCLCPP_INFO(this->get_logger(), "Dumping Subsystem ready to go!\n");

        dumping_right_speed_pub = create_publisher<std_msgs::msg::Float64>("/dumping_motor/right/speed", 10);
        dumping_left_speed_pub = create_publisher<std_msgs::msg::Float64>("/dumping_motor/left/speed", 10);
        // dumping_right_temp_pub = create_publisher<std_msgs::msg::Float64>("/dumping_motor/right/temp", 30);
        // dumping_left_temp_pub = create_publisher<std_msgs::msg::Float64>("/dumping_motor/left/temp", 30);


        telemetry_timer = this->create_wall_timer(
            100ms, [this]() {
        try {
            auto msg = std_msgs::msg::Float64();
            msg.data = m_dumping_right.GetVelocity();
            dumping_right_speed_pub->publish(msg);
            msg.data = m_dumping_left.GetVelocity();
            dumping_left_speed_pub->publish(msg);
            // msg.data = m_dumping_right.GetTemperature();
            // dumping_right_temp_pub->publish(msg);
            // msg.data = m_dumping_left.GetTemperature();
            // dumping_left_temp_pub->publish(msg);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Failed to read motor metrics: %s", e.what());
            }
        });
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
        //double dpad_horizontal = joy_msg->axes[6];
        bool dpad_horizontal_left = joy_msg->buttons[13];
        bool dpad_horizontal_right = joy_msg->buttons[14];
        double dump_latch_control = joy_msg->axes[7];
        // int dpad_dumping = joy_msg->axes[6]; //when on the nuc
        // if(dpad_dumping == -1){
        //     if (dumping_belt_running) {
        //         stop_dumping_belt();
        //     } else {
        //         move_belt_forward();
        //         dumping_belt_running = true;
        //     }
        // }

        // if(dpad_dumping == 1){
        //     if(dumping_belt_running){
        //         stop_dumping_belt();
        //     } else {
        //         move_belt_reverse();
        //         dumping_belt_running = true;
        //     }
        // }
        // D-Pad Right → Forward (axes[6] == -1)
        if (dpad_horizontal_right && !last_dpad_right) {
            if (dumping_belt_running) {
                stop_dumping_belt();
            } else {
                move_belt_forward();
                dumping_belt_running = true;
            }
        }
        last_dpad_right = dpad_horizontal_right;
    
        // D-Pad Left → Reverse (axes[6] == 1)
        if (dpad_horizontal_left && !last_dpad_left) {
            if (dumping_belt_running) {
                stop_dumping_belt();
            } else {
                move_belt_reverse();
                dumping_belt_running = true;
            }
        }
        last_dpad_left = dpad_horizontal_left;
    
        // Dump latch control (leave this logic as is)
        if(dump_latch_control < 0) { cmd_close_dumplatch(dump_latch_control); }
        if(dump_latch_control > 0) { cmd_open_dumplatch(dump_latch_control); }
    }
    void Dumping::move_belt_forward(){
        m_dumping_left.SetDutyCycle(1.0);
        m_dumping_right.SetDutyCycle(-1.0);
        // m_dumping_left.SetDutyCycle(0.5);
        // m_dumping_right.SetDutyCycle(-0.5);
    }

    void Dumping::move_belt_reverse(){
        // m_dumping_left.SetDutyCycle(-1.0);
        // m_dumping_right.SetDutyCycle(1.0);
        m_dumping_left.SetDutyCycle(-0.25);
        m_dumping_right.SetDutyCycle(0.25);
    }

    /*
    void getPostiion(){}
    void setSpeed(){}
    */

    void Dumping::initMotors(){
        m_dumping_left.SetIdleMode(IdleMode::kCoast);
        m_dumping_left.SetMotorType(MotorType::kBrushless);
        m_dumping_left.SetDutyCycle(0.0);
        m_dumping_left.ClearStickyFaults();
        m_dumping_left.ResetFaults();
        m_dumping_left.SetPeriodicStatus3Period(0);
        m_dumping_left.SetPeriodicStatus4Period(0);
        m_dumping_left.BurnFlash();
        
        m_dumping_right.SetIdleMode(IdleMode::kCoast);
        m_dumping_right.SetMotorType(MotorType::kBrushless);
        m_dumping_right.SetDutyCycle(0.0);
        m_dumping_right.ClearStickyFaults();
        m_dumping_right.ResetFaults();
        m_dumping_right.SetPeriodicStatus3Period(0);
        m_dumping_right.SetPeriodicStatus4Period(0);
        m_dumping_right.BurnFlash();

        RCLCPP_INFO(get_logger(), "Dumping Subsystem Motors configured successfully");
    }

    void Dumping::stop_dumping_belt() {
        RCLCPP_INFO(get_logger(), "STOPPING DUMPING BELT MOTORS!");
        m_dumping_left.SetDutyCycle(0.0);
        m_dumping_right.SetDutyCycle(0.0);
        dumping_belt_running = false;
    }
    


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dumping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}