#include "core.hpp"

/**
 * @file MotorControllerGroup.cpp
 * @brief Drivebase utility that groups & considers the robot's left/right side wheels as 1 unit.
 * @note 4 Robot Drivebase motors form 2 groups for DIFF DRIVE robot. Mirrored after our FRC Java drivebase.java file that uses the MotorControllerGroup class
 * @version 0.1
 * @copyright Copyright (c) 2025
 */
 
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

    void stop_motor_groups(){
        motor1.SetDutyCycle(0.0);
        motor1.SetVoltage(0);
        motor2.SetDutyCycle(0.0);
        motor2.SetVoltage(0);
    }
    

};



