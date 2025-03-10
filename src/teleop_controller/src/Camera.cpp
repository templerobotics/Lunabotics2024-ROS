/**
 * @brief Node that initializes & streams live camera footage from our 4 Robot Cameras
 * @note 3 Cameras are "regular" cameras & 1 Intel Realsense D455 Depth Camera. Depth camera is NOT needed for teleop
 * @note Articulated Robotics for regular camera pkg usb_cam. Many use openCV as well
 * @note OpenCV Camera Viewer example : https://github.com/Supernova1114/ROS2-Camera-Streamer/blob/main/camera_streamer/src/camera_viewer.cpp
 */

#include "core.hpp"

class Camera : public rclcpp::Node {
public:
    Camera() 
    : Node("cameras")
    {     
        config_cameras();
    }

private:

    void config_cameras(){
        this->declare_parameter("camera_digging",           "/dev/video0");
        this->declare_parameter("camera_dumping",           "/dev/video1");
        this->declare_parameter("camera_conveyor_belt",     "/dev/video2");
        this->declare_parameter("image_width",              640);
        this->declare_parameter("image_height",             480);
        this->declare_parameter("frame_rate",               30.0);
        this->declare_parameter("camera_frame_id",          "usb_cam");
        /**
         * @todo Remap all of these custom camera topics to "image_raw" in launch file. Then basically done I'd think. Can alter callbacks.
         * @note Could retain singular parameter declaration by get_parameters() in this file & declaring them in Launch file.
        */
        cam_digging = create_publisher<sensor_msgs::msg::Image>("/camera/digging/image_raw", 10,std::bind(&Camera::image_callback_digging, this, std::placeholders::_1));
        cam_dumping = create_publisher<sensor_msgs::msg::Image>("camera/dumping/image_raw", 10,std::bind(&Camera::image_callback_dumping, this, std::placeholders::_1));
        cam_conveyor_belt = create_publisher<sensor_msgs::msg::Image>("camera/conveyor_belt/image_raw", 10,std::bind(&Camera::image_callback_belt, this, std::placeholders::_1));
    }
    
    void image_callback_digging(const sensor_msgs::msg::Image::SharedPtr msg) {
        
    }
    void image_callback_dumping(const sensor_msgs::msg::Image::SharedPtr msg){

    }
    void image_callback_belt(const sensor_msgs::msg::Image::SharedPtr msg){

    }


    CameraImagePub cam_digging;
    CameraImagePub cam_dumping;
    CameraImagePub cam_conveyor_belt;

};

