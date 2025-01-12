/**
 * @file MotorControllerGroup.hpp
 * @author Jaden Howard (jaseanhow@gmail.com or tun85812@temple.edu)
 * @brief Motor Controller Group Header file --> Mimic FRC Java
 * @version 0.1
 * @date 2025-01-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef MOTOR_CONTROLLER_GROUP_HPP
#define MOTOR_CONTROLLER_GROUP_HPP

#include "core.hpp"

// Drivebase diff drive Motor Groups
class MotorControllerGroup {
private:
    SparkMax& motor1;
    SparkMax& motor2;

public:
    MotorControllerGroup(SparkMax& m1, SparkMax& m2) : motor1(m1), motor2(m2) {}
     /**
     * @brief Set the Speed object
     * @note Equivalent to the FRC JAVA "set(double speed) -> [-1,1] from the MotorControllerGroup documentation
     * @param speed bounded duty cycle speed value
     */
    void setSpeed(double speed) {
        motor1.SetDutyCycle(speed);
        motor2.SetDutyCycle(speed);
    }
     /**
     * @brief Set the Inverted object
     * @todo delete this function. Don't think this function is needed
     * @param inverted 
     */
    void setInverted(bool inverted) {
        motor1.SetInverted(inverted);
        motor2.SetInverted(inverted);
    }

    void stop_motor_groups() {
        motor1.SetDutyCycle(0.0);
        motor1.SetVoltage(0);
        motor2.SetDutyCycle(0.0);
        motor2.SetVoltage(0);
    }
};


#endif