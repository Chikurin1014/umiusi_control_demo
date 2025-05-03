#include "umiusi_control_demo/hardware/thruster_thrust.hpp"

namespace ucdhw = umiusi_control_demo::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto ucdhw::ThrusterThrust::on_init(const hif::HardwareInfo & /*info*/) -> hif::CallbackReturn {
    this->command = 0.0;
    this->state = 0.0;
    return hif::CallbackReturn::SUCCESS;
}

auto ucdhw::ThrusterThrust::export_state_interfaces() -> std::vector<hif::StateInterface> {
    auto interfaces_to_export = std::vector<hif::StateInterface>{};
    auto gpio = this->info_.gpios.front();
    auto interface = gpio.state_interfaces.front();
    interfaces_to_export.emplace_back(hif::StateInterface(gpio.name, interface.name, &this->state));
    return interfaces_to_export;
}

auto ucdhw::ThrusterThrust::export_command_interfaces() -> std::vector<hif::CommandInterface> {
    auto interfaces_to_export = std::vector<hif::CommandInterface>{};
    auto gpio = this->info_.gpios.front();
    auto interface = gpio.command_interfaces.front();
    interfaces_to_export.emplace_back(
      hif::CommandInterface(gpio.name, interface.name, &this->command));
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
