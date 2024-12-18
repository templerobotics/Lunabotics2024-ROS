#include "core.hpp"

namespace diffdrive_turobotics{

hardware_interface::CallbackReturn DiffBotSystemHardware::on_init( const hardware_interface::HardwareInfo & info ){
  if ( hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS ){ return hardware_interface::CallbackReturn::ERROR; }

  /*Youtube tutorial used a struct at this spot. Recall why. Need to configure for our robot's config*/


  for (const hardware_interface::ComponentInfo & joint : info_.joints){
    assert(joint.command_interfaces.size() == 1 && "Joint must have exactly one command interface");
    if (joint.command_interfaces.size() != 1) {
        throw std::runtime_error("Joint '" + std::string(joint.name.c_str()) + "' has " + std::to_string(joint.command_interfaces.size()) + " command interfaces. 1 expected.");
    }

    assert(joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY && "Command interface must be velocity type");
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
        throw std::runtime_error( "Joint '" + std::string(joint.name.c_str()) + "' command interface must be velocity");
    }

    assert(joint.state_interfaces.size() == 2 && "Joint must have exactly two state interfaces");
    if (joint.state_interfaces.size() != 2) {
        throw std::runtime_error("Joint '" + std::string(joint.name.c_str()) + "' has " + std::to_string(joint.state_interfaces.size()) + " state interfaces. 2 expected.");
    }

    assert(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION && "First state interface must be position");
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        throw std::runtime_error("Joint '" + std::string(joint.name.c_str()) + "' first state interface must be position");
    }

  }
 
  return hardware_interface::CallbackReturn::SUCCESS;

}



}  // namespace diffdrive_turobitics


PLUGINLIB_EXPORT_CLASS(
  diffdrive_turobotics::DiffBotSystemHardware, hardware_interface::SystemInterface)