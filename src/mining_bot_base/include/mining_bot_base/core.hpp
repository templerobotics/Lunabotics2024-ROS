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




