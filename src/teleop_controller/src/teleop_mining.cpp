#include "core.hpp"

class Teleop_Mining : public rclcpp::Node{
    public:
        Teleop_Mining() : Node("teleop_mining")
        #ifdef HARDWARE_ENABLED
        actuator_left("can0",5),
        actuator_right("can0",6)
        #endif
        {
          
        }
    private:
        SparkMax actuator_left;
        SparkMax actuator_right;
        


};
