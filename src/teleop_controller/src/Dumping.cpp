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
    /**
     * @brief sub to published topic from Drivebase Control
     */
    sub_dumping_conveyor_speed = this->create_subscription<Float64>("dumping/conveyor_speed", 10,std::bind(&Dumping::handleConveyorBeltSpeed, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Dumping belt initialized!\n");
}

void Dumping::handleConveyorBeltSpeed(const Float64Shared msg){
    RCLCPP_INFO(get_logger(),"DATA=%lf",msg->data);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dumping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}