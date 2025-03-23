#include "gui.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>

std::vector<float> frameData; // Store frame indices (x-axis)
std::vector<float> temperatureData; // Store temperature data (y-axis)
std::vector<float> voltageData; // Store voltage data (y-axis)
std::vector<float> velocityData; // Store velocity data (y-axis)
std::vector<float> dutyCycleData; // Store duty cycle data (y-axis)
std::vector<float> posData; // Store position data (y-axis)
void setupGUI(GLFWwindow* window) {
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 410");

    ImPlot::CreateContext();
}

void store_motor_metrics(float temperature, float voltage, float velocity, float duty_cycle, float pos) {
    frameData.push_back(frameData.size()); // Use frame index as x-axis value
    temperatureData.push_back(temperature);
    voltageData.push_back(voltage);
    velocityData.push_back(velocity);
    dutyCycleData.push_back(duty_cycle);
    posData.push_back(pos);
}


void renderGUI() {
    bool isOpen = true;


    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();



    ImGui::Begin("Parameters", &isOpen, ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize);
    ImGui::SetWindowPos(ImVec2(0,0));
    ImGui::SetWindowSize(ImVec2(500,300));
    ImGui::Text("Temperature: %.2f C", temperatureData.back());
    ImGui::Text("Voltage: %.2f V", voltageData.back());
    ImGui::Text("Velocity: %.2f m/s", velocityData.back());
    ImGui::Text("Duty Cycle: %.2f %%", dutyCycleData.back());
    ImGui::Text("Position: %.2f units", posData.back());


  // Adding ImPlot visualization
    ImGui::Begin("Count Window");
    ImGui::SetWindowPos(ImVec2(500,0));
    ImGui::SetWindowSize(ImVec2(300,300));
   
    if (ImPlot::BeginPlot("Count vs Frame Index")) {
        ImPlot::PlotLine("Temperature", frameData.data(), temperatureData.data(), frameData.size());
        ImPlot::PlotLine("Voltage", frameData.data(), voltageData.data(), frameData.size());
        ImPlot::PlotLine("Velocity", frameData.data(), velocityData.data(), frameData.size());
        ImPlot::PlotLine("Duty Cycle", frameData.data(), dutyCycleData.data(), frameData.size());
        ImPlot::PlotLine("Position", frameData.data(), posData.data(), frameData.size());
        // End the plot properly
        ImPlot::EndPlot();
    }
  
     ImGui::End();

    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    std::this_thread::sleep_for(std::chrono::millioseconds(100));

}

void cleanupGUI() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    ImPlot::DestroyContext();
}
