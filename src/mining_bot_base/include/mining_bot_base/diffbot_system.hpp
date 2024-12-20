#ifndef DIFFBOT_SYSTEM_HPP
#define DIFFBOT_SYSTEM_HPP

#include "core.hpp"

/* 
  
  Idea : Define this struct in the core.hpp header file, then just make a variable of it inside this class?
  ie: Differential_Config diffbotconfigvariable; [Struct_Name][variable_name]
  In FRC Java code, we have a "Constants.java" file defining all of this sparkmax information, but obviously it's not in ROS2 Control

*/

namespace diffdrive_turobotics{
class DrivebaseSystemHardware : public hardware_interface::SystemInterface{

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DrivebaseSystemHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  Drivebase_Config drivebase_config;
  // Command and state interfaces
  std::vector<double> hw_commands;    // Velocity commands
  std::vector<double> hw_positions;   // Encoder positions
  std::vector<double> hw_velocities;  // Encoder velocities
  
};

}  // namespace diffdrive_turobotics

#endif  //DIFFBOT_SYSTEM_HPP

