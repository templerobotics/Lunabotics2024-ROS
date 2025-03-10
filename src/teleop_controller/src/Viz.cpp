#include "core.hpp"

class BMS : public rclcpp::Node {
public:
    BMS() : Node("ArduinoBMS")

};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BMS>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

