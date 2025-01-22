# How I went about programming teleop. Note:: (NOT TESTED with XBOX Controller as of Jan 22 2025)

## Essentially adapted Java FRC code to ROS2 Humble C++, 

###  I considered the JAVA FRC code first, tried to understand what is was doing & the tools(enums/methods/constants) it used to achieve its goals, then I adapted it to ROS2 Humble C++. 


### Like I stated before the code is NOT TESTED with an XBOX Controller yet, but our motors(3) that I've tested, linear actuators all work successfully from a SetVoltage() command from the Sparkcan library


### Major considerations : Commanding motors should be possible by calling SetDutyCycle() by itself without needing to call SetVoltage() as well 

### 2 Lanes : Teleoperation & Electrical Box

### Teleoperation : At its core is simoply using an XBOX Controller and its buttons to actuate motors on our robot that move the robot's parts

### Electrical Box : Goal is to protect the electrical components inside & allow for easy maintenance of parts that need to come in/out of box frequently ie the robot battery

### Sparkcan Repo. Also mentioned in Acknowledgements.MD file --> https://github.com/grayson-arendt/sparkcan
