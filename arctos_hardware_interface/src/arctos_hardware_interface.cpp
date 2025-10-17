#include "arctos_hardware_interface/arctos_hardware_interface.hpp"
#include <algorithm>
#include <thread>

namespace arctos_hardware_interface
{
    ArctosHardwareInterface::ArctosHardwareInterface()
        : driver_(std::make_unique<CanDriver>()) {}

    ArctosHardwareInterface::~ArctosHardwareInterface()
    {
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Destructor called");
        disconnect_hardware();
    }

    CallbackReturn ArctosHardwareInterface::on_init(const hw::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // Initialize joint data
        num_joints_ = info_.joints.size();
        position_commands_.resize(num_joints_, 0.0);
        position_states_.resize(num_joints_, 0.0);
        velocity_states_.resize(num_joints_, 0.0);
        prev_position_commands_.resize(num_joints_, 0.0);

        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                    "Initialized with %zu joints", num_joints_);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        try
        {
            std::string can_interface = info_.hardware_parameters["can_interface"];
            vel_ = std::stod(info_.hardware_parameters["vel"]);
            accel_ = std::stod(info_.hardware_parameters["accel"]);

            // Get CAN interface from parameters if available
            if (info_.hardware_parameters.find("can_interface") != info_.hardware_parameters.end())
            {
                can_interface = info_.hardware_parameters.at("can_interface");
            }

            if (!driver_->connect(can_interface))
            {
                RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                             "Failed to connect to CAN interface");
                return CallbackReturn::ERROR;
            }

            RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                        "Connected to CAN interface: %s", can_interface.c_str());
            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                         "Configuration error: %s", e.what());
            return CallbackReturn::ERROR;
        }
    }

    CallbackReturn ArctosHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Activating hardware...");

        if (!driver_ || !driver_->is_connected())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                         "Driver not connected! Cannot activate.");
            return CallbackReturn::ERROR;
        }

        if (!driver_->activate())
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                         "Failed to activate servos");
            return CallbackReturn::ERROR;
        }

        // Read initial positions
        std::vector<double> initial_positions;
        if (driver_->read_positions(initial_positions))
        {
            for (size_t i = 0; i < num_joints_ && i < initial_positions.size(); ++i)
            {
                position_states_[i] = initial_positions[i];
                position_commands_[i] = initial_positions[i];
                prev_position_commands_[i] = initial_positions[i];
            }
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Deactivating hardware...");
        return disconnect_hardware();
    }

    CallbackReturn ArctosHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"), "Shutting down...");
        return disconnect_hardware();
    }

    std::vector<hw::StateInterface> ArctosHardwareInterface::export_state_interfaces()
    {
        std::vector<hw::StateInterface> state_interfaces;

        for (size_t i = 0; i < num_joints_; ++i)
        {
            state_interfaces.emplace_back(
                info_.joints[i].name,
                hw::HW_IF_POSITION,
                &position_states_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name,
                hw::HW_IF_VELOCITY,
                &velocity_states_[i]);
        }

        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                    "Exported %zu state interfaces", state_interfaces.size());
        return state_interfaces;
    }

    std::vector<hw::CommandInterface> ArctosHardwareInterface::export_command_interfaces()
    {
        std::vector<hw::CommandInterface> command_interfaces;

        for (size_t i = 0; i < num_joints_; ++i)
        {
            command_interfaces.emplace_back(
                info_.joints[i].name,
                hw::HW_IF_POSITION,
                &position_commands_[i]);
        }

        RCLCPP_INFO(rclcpp::get_logger("ArctosHardwareInterface"),
                    "Exported %zu command interfaces", command_interfaces.size());
        return command_interfaces;
    }

    hw::return_type ArctosHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        if (!driver_ || !driver_->is_connected())
        {
            return hw::return_type::ERROR;
        }

        std::vector<double> hw_positions;
        driver_->read_positions(hw_positions);

        // Calculate velocity from position change
        double dt = period.seconds();
        for (size_t i = 0; i < num_joints_; ++i)
        {
            double prev_pos = position_states_[i];
            position_states_[i] = hw_positions[i];
            velocity_states_[i] = (dt > 0) ? (position_states_[i] - prev_pos) / dt : 0.0;
        }

        return hw::return_type::OK;
    }

    hw::return_type ArctosHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!driver_ || !driver_->is_connected())
        {
            return hw::return_type::ERROR;
        }

        // Only send if commands changed significantly
        bool commands_changed = false;
        for (size_t i = 0; i < position_commands_.size(); ++i)
        {
            if (std::abs(position_commands_[i] - prev_position_commands_[i]) > 0.01)
            {
                commands_changed = true;
                break;
            }
        }

        if (!commands_changed)
        {
            return hw::return_type::OK;
        }

        try
        {
            if (driver_->send_commands(position_commands_, vel_, accel_)) // in radians
            {
                prev_position_commands_ = position_commands_;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArctosHardwareInterface"),
                         "Write command failed: %s", e.what());
            return hw::return_type::ERROR;
        }

        return hw::return_type::OK;
    }

    CallbackReturn ArctosHardwareInterface::disconnect_hardware()
    {
        if (driver_ && driver_->is_connected())
        {
            try
            {
                driver_->deactivate();
                driver_->disconnect();
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(rclcpp::get_logger("ArctosHardwareInterface"),
                            "Disconnect error: %s", e.what());
                return CallbackReturn::ERROR;
            }
        }
        return CallbackReturn::SUCCESS;
    }

} // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arctos_hardware_interface::ArctosHardwareInterface, hw::SystemInterface)