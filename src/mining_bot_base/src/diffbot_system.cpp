#include "core.hpp"

namespace diffdrive_turobotics{
  
hardware_interface::CallbackReturn DrivebaseSystemHardware::on_init( const hardware_interface::HardwareInfo & info ){
  if ( hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS ){ return hardware_interface::CallbackReturn::ERROR; }

  //Load Parameters --> Remember this class acts as the Hardware Plugin that directly controls physical Robot motors
  drivebase_config.left_front_wheel_name = info_.hardware_parameters["left_front_wheel_name"];
  drivebase_config.left_back_wheel_name = info_.hardware_parameters["left_back_wheel_name"];
  drivebase_config.right_front_wheel_name = info_.hardware_parameters["right_front_wheel_name"];
  drivebase_config.right_back_wheel_name = info_.hardware_parameters["right_back_wheel_name"];
  drivebase_config.LOOP_RATE = std::stoi(info_.hardware_parameters["LOOP_RATE"]);
  drivebase_config.ENC_COUNTS_PER_REV = std::stoi(info_.hardware_parameters["ENC_COUNTS_PER_REV"]);


  //Initialize Sparkmaxes
  std::unique_ptr<SparkMax> left_front_motor = std::make_unique<SparkMax>(drivebase_config.DEVICE_ID , drivebase_config.CAN_ID_LEFT_FRONT);
  std::unique_ptr<SparkMax> left_back_motor = std::make_unique<SparkMax>(drivebase_config.DEVICE_ID , drivebase_config.CAN_ID_LEFT_BACK);
  std::unique_ptr<SparkMax> right_front_motor = std::make_unique<SparkMax>(drivebase_config.DEVICE_ID , drivebase_config.CAN_ID_RIGHT_FRONT);
  std::unique_ptr<SparkMax> right_back_motor = std::make_unique<SparkMax>(drivebase_config.DEVICE_ID , drivebase_config.CAN_ID_RIGHT_BACK);
  
  /*Need to write the description/.xacro to create a [Hardware Interface plugin] that the middleman Controller Manager will use*/

  for (const hardware_interface::ComponentInfo & joint : info_.joints){
    assert(joint.command_interfaces.size() == 1 && "Joint must have exactly 1 command interface");
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

    assert(joint.state_interfaces[1].name == hardware_interface::HW_IF_VELOCITY && "2nd state interface must be velocity");
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
        throw std::runtime_error("Joint '" + std::string(joint.name.c_str()) + "' 2nd state interface must be velocity");
    }


  }
 
  return hardware_interface::CallbackReturn::SUCCESS;

}



}  // namespace diffdrive_turobitics


PLUGINLIB_EXPORT_CLASS(
  diffdrive_turobotics::DrivebaseSystemHardware, hardware_interface::SystemInterface)