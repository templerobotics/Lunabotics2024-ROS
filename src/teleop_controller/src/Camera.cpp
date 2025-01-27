/**
 * @brief Node that initializes & streams live camera footage from our 4 Robot Cameras
 * @note 3 Cameras are "regular" cameras & 1 Intel Realsense D455 Depth Camera 
 * @todo cams connect via usb port i think? Save to a container?
 */

#include "core.hpp"

class Camera : public rclcpp::Node {
public:
    Camera() 
    : Node("camera_livestream")
    {
        cam_digging = create_subscription<sensor_msgs::msg::Image>("/usb_cam/image_raw", 10,std::bind(&Camera::image_callback_digging, this, std::placeholders::_1));
        cam_dumping = create_subscription<sensor_msgs::msg::Image>("/usb_cam/image_raw", 10,std::bind(&Camera::image_callback_dumping, this, std::placeholders::_1));
        cam_belt = create_subscription<sensor_msgs::msg::Image>("/usb_cam/image_raw", 10,std::bind(&Camera::image_callback_belt, this, std::placeholders::_1));
        cam_realsense_color = create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", 10,std::bind(&Camera::image_callback_color, this, std::placeholders::_1));
        cam_realsense_depth = create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw",10,std::bind(&Camera::image_callback_depth, this, std::placeholders::_1));
    }

private:
   
    void image_callback_digging(const sensor_msgs::msg::Image::SharedPtr msg) {
    }
    void image_callback_dumping(const sensor_msgs::msg::Image::SharedPtr msg){

    }
    void image_callback_belt(const sensor_msgs::msg::Image::SharedPtr msg){

    }

    void image_callback_color(const sensor_msgs::msg::Image::SharedPtr msg){

    }
    void image_callback_depth(const sensor_msgs::msg::Image::SharedPtr msg){

    }

    CameraImageSub cam_digging;
    CameraImageSub cam_dumping;
    CameraImageSub cam_belt;
    CameraImageSub cam_realsense_color;
    CameraImageSub cam_realsense_depth;

    std::array<CameraImageSub> cameras;

};

