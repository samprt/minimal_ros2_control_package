/*
 * Copyright (C) 2022 Heracles Robotics.
 */

#ifndef MINIMAL_EXAMPLE_ROS2_CONTROL__MINIMAL_EXAMPLE_SYSTEM_HPP_
#define MINIMAL_EXAMPLE_ROS2_CONTROL__MINIMAL_EXAMPLE_SYSTEM_HPP_

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "asio.hpp"

#include "visibility_control.h"

// Comment the following line to not use asio
#define USING_ASIO

namespace minimal_example_system {

    class MinimalExampleSystem : public hardware_interface::SystemInterface {
    public:
#ifdef USING_ASIO
        MinimalExampleSystem() : hardware_interface::SystemInterface(),
                            work_guard_(asio::make_work_guard(io_context_)),
                            io_thread_([this]() { this->io_context_.run(); }) { }
#else
        MinimalExampleSystem() : hardware_interface::SystemInterface() { }
#endif

        ~MinimalExampleSystem() override {
            this->stop_tcp();
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &previous_state) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo &hardware_info) override;

        std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

        std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;

        hardware_interface::return_type
        read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type
        write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        std::map<std::string, uint8_t> joint_ids_;

#ifdef USING_ASIO
        /// Asio context
        asio::io_context io_context_;
        /// Stop the context from finishing before the start of async operations
        asio::executor_work_guard<asio::io_context::executor_type> work_guard_;
        /// Background thread for io completion handling
        std::thread io_thread_;
#endif

        double joint1_command_;
        double joint2_command_;
        double joint1_state_;
        double joint2_state_;

        /**
         * Stop TCP
         */
        void stop_tcp() {
#ifdef USING_ASIO
            this->io_context_.stop();
            this->io_thread_.join();
#endif
        }
    };
} // namespace minimal_example_system
#endif // MINIMAL_EXAMPLE_ROS2_CONTROL__EXCAVATOR_SYSTEM_HPP_
