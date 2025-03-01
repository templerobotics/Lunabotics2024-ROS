#pragma once

#include "core.hpp"

class DiggingBelt : public rclcpp::Node {
public:
    DiggingBelt();
    void setBeltSpeed(double speed);
    void runBelt(bool reverse);
    void stopBelt();
    bool isRunning() const { return belt_running; }

private:
    //std::unique_ptr<PIDController> p_belt;
    
    /**
     * @brief state variables
     */
    double current_speed{0.0};
    double belt_speed{0.5};
    const double SPEED_INCREMENT{0.1};
    bool belt_running{false};  // Keep only one declaration
    XBOX_JOYSTICK_INPUT_t xbox_input;

    /**
    * @brief core functionality
    */
    void configureBelts();
    void configurePID();
    void reportSensors();
    void handleDiggingSpeed(const Float64Shared msg);
    void periodic();

protected:
    /**
     * @brief required hardware
     */
    SparkMax m_belt1;
    SparkMax m_belt2;
    SparkMax m_linear_left;
    SparkMax m_linear_right;
    std::reference_wrapper<SparkMax> Motors_Digging_Belt[4] = 
    {
        std::ref(m_belt1), 
        std::ref(m_belt2),
        std::ref(m_linear_left),
        std::ref(m_linear_right)
    };
    
    Float64Publisher velocity_pub;
    Float64Publisher temperature_pub;
    Float64Subscriber mining_belt_speed_sub;
    rclcpp::TimerBase::SharedPtr timer_diagnostics;
   
};