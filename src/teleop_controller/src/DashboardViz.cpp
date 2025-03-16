/**
 * @brief Dear ImGUI & ImPlot real-time robotics visualization dashboard
 * @details Needs : Lightweight + Metrics/Widgets for intuitive usage in competition.
 */

#include "core.hpp"

class DashboardViz : public rclcpp::Node {
public:
    DashboardViz() : Node("realtime_dashboard_visualizations")

};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DashboardViz>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

