#include "minimal_example_ros2_control/minimal_example_system.hpp"
#include "rcutils/time.h"

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(minimal_example_system::MinimalExampleSystem, hardware_interface::SystemInterface)


namespace minimal_example_system {
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    MinimalExampleSystem::on_init(const hardware_interface::HardwareInfo &hardware_info) {
        if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("MinimalExampleSystem"), "%s initialised", this->get_name().c_str());

        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    MinimalExampleSystem::on_configure(const rclcpp_lifecycle::State &previous_state) {
        static_cast<void>(previous_state);  // Unused variable

        RCLCPP_INFO(rclcpp::get_logger("MinimalExampleSystem"), "%s configured", this->get_name().c_str());

        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    MinimalExampleSystem::on_activate(const rclcpp_lifecycle::State &previous_state) {
        static_cast<void>(previous_state);  // Unused variable

        RCLCPP_INFO(rclcpp::get_logger("MinimalExampleSystem"), "%s activated", this->get_name().c_str());

        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    MinimalExampleSystem::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        static_cast<void>(previous_state);  // Unused variable


        RCLCPP_INFO(rclcpp::get_logger("MinimalExampleSystem"), "%s deactivated", this->get_name().c_str());

        return CallbackReturn::SUCCESS;
    }


    std::vector<hardware_interface::CommandInterface> MinimalExampleSystem::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // Joints' state interfaces
        command_interfaces.emplace_back(info_.joints[0].name,
                                        hardware_interface::HW_IF_EFFORT,
                                        &joint1_command_);
        command_interfaces.emplace_back(info_.joints[1].name,
                                        hardware_interface::HW_IF_EFFORT,
                                        &joint2_command_);
        return command_interfaces;
    }

    std::vector<hardware_interface::StateInterface> MinimalExampleSystem::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(info_.joints[0].name,
                                      hardware_interface::HW_IF_POSITION,
                                      &joint1_state_);
        state_interfaces.emplace_back(info_.joints[1].name,
                                      hardware_interface::HW_IF_POSITION,
                                      &joint2_state_);
        return state_interfaces;
    }

    hardware_interface::return_type MinimalExampleSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
        static_cast<void>(time);  // Unused parameter
        static_cast<void>(period);  // Unused parameter

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MinimalExampleSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        static_cast<void>(time);  // Unused parameter
        static_cast<void>(period);  // Unused parameter

        return hardware_interface::return_type::OK;
    }

} // namespace minimal_example_system
