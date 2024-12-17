#ifndef SPARKMAX_HARDWARE_INTERFACE_HPP
#define SPARKMAX_HARDWARE_INTERFACE_HPP


std::vector<hardware_interface::StateInterface> export_state_interfaces() override
std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override

#endif