#pragma once
#include "core.hpp"

class Dumping : public rclcpp::Node {

public:
    Dumping();
    void handleConveyorBeltSpeed(const Float64Shared msg);
    void setConveyorBeltSpeed(double speed);
    SparkMax m_dumping_left;
    SparkMax m_dumping_right;
    XBOX_JOYSTICK_INPUT_t xbox_input;
    double current_speed{0.0};
    Float64Subscriber sub_dumping_conveyor_speed;
    
    /**
     * @todo Linear actuators needed for dumping & digging. Trying to decide where to put them
    */
    std::reference_wrapper<SparkMax> Motors_Dumping[2] = 
    {
        std::ref(m_dumping_left), 
        std::ref(m_dumping_right)
    };

    

};