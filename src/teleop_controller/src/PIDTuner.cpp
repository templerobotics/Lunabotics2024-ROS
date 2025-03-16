/**
 * @brief C++ PID tuning for REV Robotics Hardware for Lunabotics Competition
 */

 #include "core.hpp"

 class PIDTuner : public rclcpp::Node {
 public:
     PIDTuner() : Node("pid_tuned")
 
 };
 
 
 int main(int argc, char* argv[]) {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<PIDTuner>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
 }
 
 