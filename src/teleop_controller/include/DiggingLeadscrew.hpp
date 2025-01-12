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

#ifndef DIGGING_LEADSCREW_HPP
#define DIGGING_LEADSCREW_HPP

#include "core.hpp"

class DiggingLeadscrew {
public:
    /**
     * @todo Change leadscrew CAN ID to reflect their ACTUAL values
     * 
     */
    const uint8_t LEADSCREW_1_CAN_ID = 7;
    const uint8_t LEADSCREW_2_CAN_ID = 8;

    /**
     * @brief Fault IDs based on REV documentation
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

    DiggingLeadscrew();
    
    bool isTopLimitPressed();
    bool isBottomLimitPressed();
    void setLeadscrewSpeed(double speed);
    LeadscrewState getState() const;

private:
    SparkMax m_leadscrew1;
    SparkMax m_leadscrew2;
    
    LeadscrewState leadscrew_state = LeadscrewState::Traveling;
    bool leadscrew_initialized = false;
    const double LEADSCREW_MAX_ERROR = 0.1;
    const double LEADSCREW_MAX_TRAVEL = 10.0;

    void configureLimitSwitches();
    bool checkFault(uint16_t faults, FaultBits bit);
    void periodic();
    void checkLimits();
    void publishState();
    std::string stateToString(LeadscrewState state);

};

#endif