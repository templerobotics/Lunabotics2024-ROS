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

/**
 * @brief Fault IDs based on REV documentation
 * @todo Are these values anything more than an error library in FRC? I'd assume these are tied to physical hardware. Do research
 */
enum class FaultBits : uint16_t {
    kHardLimitFwd = 0,    // Forward limit switch
    kHardLimitRev = 1,    // Reverse limit switch
    kSoftLimitFwd = 2,    // Forward soft limit
    kSoftLimitRev = 3,    // Reverse soft limit
    kMotorFault = 4,      // Motor fault
    kSensorFault = 5,     // Sensor fault
    kStall = 6,           // Stall detected
    kEEPROMCRC = 7,       // EEPROM CRC error
    kCANTX = 8,           // CAN transmit error
    kCANRX = 9,           // CAN receive error
    kHasReset = 10,       // Has reset
    kDRVFault = 11,       // DRV fault
    kOtherFault = 12,     // Other fault
    kSoftLimitClamp = 13, // Soft limit clamp
    kBrownout = 14        // Brownout
};

enum class LeadscrewState {
    Extended,
    Retracted,
    Traveling,
    FullExtended,
    GivenCommand
};

class DiggingLeadscrew : public rclcpp::Node {
public:
    DiggingLeadscrew();
    
    bool isTopLimitPressed();
    bool isBottomLimitPressed();
    void setLeadscrewSpeed(double speed);
    LeadscrewState getState() const;

private:
    SparkMax m_leadscrew1;
    SparkMax m_leadscrew2;
    
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
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
    rclcpp::TimerBase::SharedPtr timer_diagnostics;
    Float64Subscriber leadscrew_speed_sub;

};

