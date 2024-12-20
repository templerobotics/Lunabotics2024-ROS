#pragma once

/*  ROS2 Headers  */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen.h>
#include "hardware_interface/system_interface.hpp"
#include <cstddef>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/utilities.hpp"
#include "rclcpp/logging.hpp"
#include <cassert>


/*  Sparkcan Headers    */
#include <SparkBase.hpp> 
#include <SparkFlex.hpp>
#include <SparkMax.hpp>


/* Custom Headers */
#include "mining_bot_base/diffbot_system.hpp"

/* Aliases & Type defs*/



/* Drivebase Configuration */
struct Drivebase_Config{
  std::string left_front_wheel_name = "";
  std::string right_front_wheel_name = "";
  std::string left_back_wheel_name = "";
  std::string right_back_wheel_name = "";                          
  const std::string DEVICE_ID = "can0";                 // Sparkmaxes CAN-bus are chained together, w/ each one having a UNIQUE CAN ID
  const static int RAMP_RATE_SECONDS = 1;
  int ENC_COUNTS_PER_rEV = 4096; 
  const static int LOOP_RATE = 30;
  //Update these to reflect actual robot values
  const static int CAN_ID_LEFT_FRONT = 9;                         
  const static int CAN_ID_LEFT_BACK = 2; 
  const static int CAN_ID_RIGHT_FRONT = 3; 
  const static int CAN_ID_RIGHT_BACK = 4;
  //Current Limits
  const static int CURRENT_LIMIT_STALL = 40;
  const static int CURRENT_LIMIT_FREE = 30;
  const static int SECONDARY_CURRENT_LIMIT = 50; 

  int PID_kP = 0;
  int PID_kI = 0;
  int PID_kD = 0;
  
  /* 
  
  Note: This info should be taken care of in a .xacro file filled with param names for our ROS2 Control file
  final static double DRIVE_WHEEL_RADIUS = 0.01;  meters -- Subject to change
  static double DRIVE_GEARBOX_RATIO = 125;        Subject to change
  
  */

};


