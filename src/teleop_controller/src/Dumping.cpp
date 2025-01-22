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

void Dumping::handleConveyorBeltSpeed(const Float64Shared msg) {
    setConveyorBeltSpeed(msg->data);
}

/**
 * @brief setting speed of conveyor belt
 */
void Dumping::setConveyorBeltSpeed(double speed) {
    
    #ifdef HARDWARE_ENABLED
    m_dumping_left.SetDutyCycle(speed);
    m_dumping_right.SetDutyCycle(speed);
    #endif

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dumping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}