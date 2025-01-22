/**
 * @brief Node that initializes & streams live camera footage from our 4 Robot Cameras
 * @note 3 Cameras are "regular" cameras & 1 Intel Realsense D455 Depth Camera
 */

#include "core.hpp"

class Camera : public rclcpp::Node {
public:
    Camera() 
    : Node("camera_livestream")
    {
        
    }



};

