#include "minimal_example_ros2_control/minimal_example_controller.hpp"

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(minimal_example_controller::MinimalExampleController, controller_interface::ControllerInterface)

namespace minimal_example_controller {


    controller_interface::CallbackReturn MinimalExampleController::on_init() {
        // Load params
        try
        {
            param_listener_ = std::make_shared<ParamListener>(get_node());
        }
        catch (const std::exception & e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::CallbackReturn MinimalExampleController::on_configure(const rclcpp_lifecycle::State &previous_state) {
        if (!param_listener_)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
            return controller_interface::CallbackReturn::ERROR;
        }
        params_ = param_listener_->get_params();

        if (params_.joints.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (params_.interface_name.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        for (const auto & joint : params_.joints)
        {
            command_interface_types_.push_back(joint + "/" + params_.interface_name);
        }

        joints_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
                "~/commands", rclcpp::QoS(1).best_effort().durability_volatile(),
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    this->commands_ = msg->data;
                    std::cout << "Received command" << std::endl;
                });

        RCLCPP_INFO(get_node()->get_logger(), "configure successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration MinimalExampleController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        command_interfaces_config.names = this->command_interface_types_;

        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration MinimalExampleController::state_interface_configuration() const {
        return controller_interface::InterfaceConfiguration{
                controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::CallbackReturn MinimalExampleController::on_activate(const rclcpp_lifecycle::State &previous_state) {
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
        if (!controller_interface::get_ordered_interfaces(
                command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
            command_interface_types_.size() != ordered_interfaces.size()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                         command_interface_types_.size(), ordered_interfaces.size());
            return controller_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_node()->get_logger(), "activate successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn MinimalExampleController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        this->joints_command_subscriber_.reset();

        // reset commands
        for (auto &command: this->commands_) {
            command = 0.;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type MinimalExampleController::update(const rclcpp::Time & /*time*/,
                                                             const rclcpp::Duration & /*period*/) {
        auto joint_commands = this->commands_;

        // no command received yet
        if (joint_commands.empty()) {
            return controller_interface::return_type::OK;
        }

        if (joint_commands.size() != command_interfaces_.size()) {
            RCLCPP_ERROR_THROTTLE(
                    get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                    "command size (%zu) does not match number of interfaces (%zu)",
                    joint_commands.size(), command_interfaces_.size());
            return controller_interface::return_type::ERROR;
        }

        std::cout << "Commands : [" << joint_commands[0] << " " << joint_commands[1] << "]" << std::endl;

        for (auto index = 0ul; index < command_interfaces_.size(); ++index) {
            command_interfaces_[index].set_value(joint_commands[index]);
        }

        return controller_interface::return_type::OK;
    }

} // namespace minimal_example_controller
