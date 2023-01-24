#ifndef MINIMAL_EXAMPLE_ROS2_CONTROL__MINIMAL_EXAMPLE_CONTROLLER_HPP_
#define MINIMAL_EXAMPLE_ROS2_CONTROL__MINIMAL_EXAMPLE_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "visibility_control.h"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "minimal_example_controller_parameters.hpp"

namespace minimal_example_controller {

    class MinimalExampleController : public controller_interface::ControllerInterface {

    public:
        MinimalExampleController() = default;

        ~MinimalExampleController() override = default;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State &previous_state) override;


        controller_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State &previous_state) override;


        controller_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State &previous_state) override;


        controller_interface::return_type update(
                const rclcpp::Time &time, const rclcpp::Duration &period) override;

    protected:

        std::shared_ptr<ParamListener> param_listener_;
        Params params_;

        std::vector<std::string> joint_names_;
        std::string interface_name_;
        std::vector<std::string> command_interface_types_;

        std::vector<double> commands_;

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joints_command_subscriber_;

    };

} // namespace minimal_example_controller

#endif // MINIMAL_EXAMPLE_ROS2_CONTROL__MINIMAL_EXAMPLE_CONTROLLER_HPP_