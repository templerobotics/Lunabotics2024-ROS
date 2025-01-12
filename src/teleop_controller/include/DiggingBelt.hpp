/**
 * @file DiggingBelt.hpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief Digging Belt Header file --> Mimics FRC Java
 * @version 0.1
 * @date 2025-01-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef DIGGING_BELT_HPP
#define DIGGING_BELT_HPP

#include "core.hpp"

class DiggingBelt {
public:

    const uint8_t BELT_1_CAN_ID = 5;
    const uint8_t BELT_2_CAN_ID = 6;
    bool belt_running = false;
    DiggingBelt();


    void setBeltSpeed(double speed);
    void runBelt(bool reverse);
    void stopBelt();
    bool isBeltRunning() const { return belt_running; }

private:
    SparkMax m_belt1;
    SparkMax m_belt2;
    
    int belt_speed = 11000;  // Default speed
    
    /**
     * @todo Add PID constants
     */
    const double BELT_VELOCITY_SCALAR = 1.0;
    const double BELT_kP = 0.0; 
    const double BELT_kI = 0.0;
    const double BELT_kD = 0.0;
    const double BELT_kIZ = 0.0;
    const double BELT_kFF = 0.0;

    void configureBelts();
    void periodic();
    void reportSensors();
    void configurePID();

};


#endif
