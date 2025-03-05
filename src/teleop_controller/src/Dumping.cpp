/**
 * @brief Node pertaining to our Robot's dumping conveyor belt
 * @note Conveyor belt speed should be LESS than mining belt speed, which is 10.
 * @details mining_belt_pub = create_publisher<Float64>("mining/belt_speed", 10);
 * @todo Review the function " void publishDumpingCommands() " in DrivebaseControl.cpp file. Might want to move functionality to this file. IDK yet
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
    void Dumping::joy_callback_dumping(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
        double activate_conveyor_belt = 
        double activate_dump_latch = 
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