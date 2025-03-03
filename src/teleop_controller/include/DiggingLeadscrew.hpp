/**
 * @file DiggingLeadscrew.hpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief Leadscrew Header file --> Mimics FRC Java 
 * @version 0.1
 * @date 2025-01-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "core.hpp"



class DiggingLeadscrew : public rclcpp::Node {
public:
    DiggingLeadscrew();
    
    bool isTopLimitPressed();
    bool isBottomLimitPressed();
    void setLeadscrewSpeed(double speed);
    LeadscrewState getState() const;
    LeadscrewState leadscrew_state = LeadscrewState::Traveling;
    bool leadscrew_initialized{false};
    void configureLimitSwitches();
    bool checkFault(uint16_t faults, FaultBits bit);
    void periodic();
    void checkLimits();
    void publishState();
    void handleMiningLeadscrewSpeed(const Float64Shared msg);
    std::string stateToString(LeadscrewState state);

protected:
    SparkMax m_leadscrew1;
    SparkMax m_leadscrew2;
    std::reference_wrapper<SparkMax> Motors_Digging_Leadscrews[2] = 
    {
        std::ref(m_leadscrew1), 
        std::ref(m_leadscrew2)
    };
    
    rclcpp::TimerBase::SharedPtr timer_diagnostics;
    Float64Subscriber leadscrew_speed_sub;

};

