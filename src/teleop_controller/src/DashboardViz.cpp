#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <imgui.h>         // ImGui header
#include <implot.h>        // ImPlot header
#include "GLFW/glfw3.h"    // GLFW header
#include "sensor_msgs/msg/joy.hpp"
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

// Structure to hold motor metrics data
struct MotorMetrics {
    std::vector<float> temperature;
    std::vector<float> voltage;
    std::vector<float> velocity;
    std::vector<float> duty_cycle;
    std::vector<float> position;
};

// ROS2 Node for Visualization
class DashboardViz : public rclcpp::Node {
public:
    DashboardViz() : Node("dashboard_viz"), metrics({}) {
        // Subscribe to the DrivebaseControl motor metrics topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "motor_metrics", 10, std::bind(&DashboardViz::metrics_callback, this, std::placeholders::_1)
        );

        // Initialize GLFW window
        if (!glfwInit()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GLFW");
            rclcpp::shutdown();
        }

        window_ = glfwCreateWindow(800, 600, "Motor Metrics Visualization", NULL, NULL);
        if (!window_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create GLFW window");
            glfwTerminate();
            rclcpp::shutdown();
        }
        glfwMakeContextCurrent(window_);

        // Initialize ImGui
        ImGui::CreateContext();
        ImPlot::CreateContext();
    }

    ~DashboardViz() {
        ImPlot::DestroyContext();
        ImGui::DestroyContext();
        glfwDestroyWindow(window_);
        glfwTerminate();
    }

    void run() {
        while (!glfwWindowShouldClose(window_) && rclcpp::ok()) {
            glfwPollEvents();
            renderUI();
            glfwSwapBuffers(window_);
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    GLFWwindow* window_;
    MotorMetrics metrics;

    // Callback function to process incoming motor metrics
    void metrics_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::stringstream ss(msg->data);
        float temp, volt, vel, duty, pos;
        ss >> temp >> volt >> vel >> duty >> pos;

        if (metrics.temperature.size() > 100) { metrics.temperature.erase(metrics.temperature.begin()); }
        if (metrics.voltage.size() > 100) { metrics.voltage.erase(metrics.voltage.begin()); }
        if (metrics.velocity.size() > 100) { metrics.velocity.erase(metrics.velocity.begin()); }
        if (metrics.duty_cycle.size() > 100) { metrics.duty_cycle.erase(metrics.duty_cycle.begin()); }
        if (metrics.position.size() > 100) { metrics.position.erase(metrics.position.begin()); }

        metrics.temperature.push_back(temp);
        metrics.voltage.push_back(volt);
        metrics.velocity.push_back(vel);
        metrics.duty_cycle.push_back(duty);
        metrics.position.push_back(pos);
    }

    // Function to render ImGui + ImPlot UI
    void renderUI() {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Motor Metrics Visualization");

        if (ImPlot::BeginPlot("Temperature")) {
            ImPlot::PlotLine("Temperature (°C)", metrics.temperature.data(), metrics.temperature.size());
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("Voltage")) {
            ImPlot::PlotLine("Voltage (V)", metrics.voltage.data(), metrics.voltage.size());
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("Velocity")) {
            ImPlot::PlotLine("Velocity (m/s)", metrics.velocity.data(), metrics.velocity.size());
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("Duty Cycle")) {
            ImPlot::PlotLine("Duty Cycle (%)", metrics.duty_cycle.data(), metrics.duty_cycle.size());
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("Position")) {
            ImPlot::PlotLine("Position", metrics.position.data(), metrics.position.size());
            ImPlot::EndPlot();
        }

        ImGui::End();
        ImGui::Render();
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DashboardViz>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
