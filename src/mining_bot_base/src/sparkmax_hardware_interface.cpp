#include "core.hpp"


class SparkMaxHardware : public hardware_interface::SystemInterface {

private:

    std::unique_ptr<SparkMax> front_left_motor;
    std::unique_ptr<SparkMax> front_right_motor;
    std::unique_ptr<SparkMax> back_left_motor;
    std::unique_ptr<SparkMax> back_right_motor;

    // State and command interfaces
    std::vector<double> hw_velocities;
    std::vector<double> hw_positions;
    std::vector<double> hw_commands;

public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        /*  4 motor Motors  */
        hw_velocities_.resize(4, 0.0);  
        hw_positions_.resize(4, 0.0);
        hw_commands_.resize(4, 0.0);
        
        try {
                /*  Initialize Sparkmaxes & Motors 
                    These values are ARBITRARY. Use REV Hardware Client to verify!
                 */
            front_left_motor = std::make_unique<SparkMax>("can0", 1);
            front_right_motor = std::make_unique<SparkMax>("can0",2);
            back_left_motor = std::make_unique<SparkMax>("can0",3);
            back_right_motor = std::make_unique<SparkMax>("can0",4);

             for (auto controller : {front_left_motor_.get(), front_right_motor_.get(), back_left_motor_.get(), back_right_motor_.get()}) {
                controller->SetMotorType(MotorType::kBrushless);
                controller->SetIdleMode(IdleMode::kCoast);
                controller->SetInverted(true);
                controller->BurnFlash();
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SparkMax"), "Failed to initialize: %s", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        // Export state interfaces for position and velocity
        std::vector<hardware_interface::StateInterface> state_interfaces;
        
        // Left wheel
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "left_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[0]));
            
        // Right wheel
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[1]));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        // Export command interfaces for velocity
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            "left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            "right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]));

        return command_interfaces;
    }

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
        // Read current states from motors
        try {
            hw_velocities_[0] = left_motor_->GetVelocity();
            hw_velocities_[1] = right_motor_->GetVelocity();
            
            hw_positions_[0] = left_motor_->GetPosition();
            hw_positions_[1] = right_motor_->GetPosition();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SparkMaxHardware"), "Failed to read: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
        // Write commands to motors
        try {
            left_motor_->SetVelocity(hw_commands_[0]);
            right_motor_->SetVelocity(hw_commands_[1]);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SparkMaxHardware"), "Failed to write: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
};