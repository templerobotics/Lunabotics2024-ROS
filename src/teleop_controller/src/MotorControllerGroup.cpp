#include "core.hpp"

/*
    4 Robot Drivebase motors form 2 groups for DIFF DRIVE robot
    FRC Java drivebase file has MotorControllerGroup class
    This is ROS not FRC. 
    We set() motor speeds w/ setVoltage() & setDutyCycle() 
    at the lowest level. Review how we do that in the code to make sure

*/

class MotorControllerGroup {
private:
    SparkMax& motor1;
    SparkMax& motor2;

public:
    MotorControllerGroup(SparkMax& m1, SparkMax& m2) : motor1(m1), motor2(m2) {}
    
    //Equivalent to the FRC JAVA "set(double speed) -> [-1,1] from the MotorControllerGroup docs
    void setSpeed(double speed) {
        motor1.SetDutyCycle(speed);
        motor2.SetDutyCycle(speed);
    }
    
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

