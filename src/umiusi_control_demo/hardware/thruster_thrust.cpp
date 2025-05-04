#include "umiusi_control_demo/hardware/thruster_thrust.hpp"

namespace ucdhw = umiusi_control_demo::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto ucdhw::ThrusterThrust::on_init(const hif::HardwareInfo & /*info*/) -> hif::CallbackReturn {
    this->command = 0.0;
    this->state = 0.0;
    return hif::CallbackReturn::SUCCESS;
}

auto ucdhw::ThrusterThrust::on_export_state_interfaces()
  -> std::vector<hif::StateInterface::ConstSharedPtr> {
    auto interfaces_to_export = std::vector<hif::StateInterface::ConstSharedPtr>{};
    auto gpio = this->info_.gpios.front();
    auto interface = gpio.state_interfaces.front();
    auto state_interface =
      std::make_shared<hif::StateInterface>(gpio.name, interface.name, &this->state);
    interfaces_to_export.push_back(state_interface);
    return interfaces_to_export;
}

auto ucdhw::ThrusterThrust::on_export_command_interfaces()
  -> std::vector<hif::CommandInterface::SharedPtr> {
    auto interfaces_to_export = std::vector<hif::CommandInterface::SharedPtr>{};
    auto gpio = this->info_.gpios.front();
    auto interface = gpio.command_interfaces.front();
    auto command_interface =
      std::make_shared<hif::CommandInterface>(gpio.name, interface.name, &this->command);
    interfaces_to_export.push_back(command_interface);
    return interfaces_to_export;
}

auto ucdhw::ThrusterThrust::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  -> hif::return_type {
    return hif::return_type::OK;
}

auto ucdhw::ThrusterThrust::write(const rclcpp::Time & /*time*/,
                                  const rclcpp::Duration & /*period*/) -> hif::return_type {
    this->state = this->command;
    return hif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(umiusi_control_demo::hardware::ThrusterThrust,
                       hardware_interface::ActuatorInterface)
