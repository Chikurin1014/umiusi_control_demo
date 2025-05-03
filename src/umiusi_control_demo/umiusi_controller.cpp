#include "umiusi_control_demo/umiusi_controller.hpp"

namespace ucdhw = umiusi_control_demo;
namespace rlc = rclcpp_lifecycle;
namespace hif = hardware_interface;
namespace cif = controller_interface;

auto ucdhw::UmiusiController::command_interface_configuration() const
  -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
      cif::interface_configuration_type::ALL, // すべてのインターフェースを扱う。
      {},
    };
}

auto ucdhw::UmiusiController::state_interface_configuration() const -> cif::InterfaceConfiguration {
    return cif::InterfaceConfiguration{
      cif::interface_configuration_type::ALL, // すべてのインターフェースを扱う。
      {},
    };
}

auto ucdhw::UmiusiController::on_init() -> cif::CallbackReturn {
    this->reference = 0.0;
    return cif::CallbackReturn::SUCCESS;
}

auto ucdhw::UmiusiController::on_configure(const rlc::State & /*pervious_state*/)
  -> cif::CallbackReturn {
    // ユーザーからの入力を受け取るためのSubscriber。
    this->subscriber = this->get_node()->create_subscription<std_msgs::msg::Float64>(
      "user_command",
      rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr input) { this->last_msg = *input; });
    this->set_chained_mode(false);

    return cif::CallbackReturn::SUCCESS;
}

auto ucdhw::UmiusiController::on_activate(const rlc::State & /*previous_state*/)
  -> cif::CallbackReturn {
    for (size_t i = 0; i < 4; i++) {
        auto interface_name = "thruster" + std::to_string(i + 1) + "/joint/" + hif::HW_IF_POSITION;

        // `CommandInterface`にアクセスしやすいよう、メンバ変数にポインタを持っておく。
        // (※上位クラスの`ControllerInterfaceBase`が持っている`command_interfaces_`は`std::vector`で実装されていてわかりづらいため。)
        {
            auto it =
              std::find_if(this->command_interfaces_.begin(),
                           this->command_interfaces_.end(),
                           [&](const auto &ifc) { return ifc.get_name() == interface_name; });
            if (it != this->command_interfaces_.end()) {
                this->thruster_joint_cmd[i].reset(it.base());
            } else {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                             "Failed to find command interface: %s",
                             interface_name.c_str());
            }
        }

        // `StateInterface`にアクセスしやすいよう、メンバ変数にポインタを持っておく。
        // (※上位クラスの`ControllerInterfaceBase`が持っている`state_interfaces_`は`std::vector`で実装されていてわかりづらいため。)
        {
            auto it =
              std::find_if(this->state_interfaces_.begin(),
                           this->state_interfaces_.end(),
                           [&](const auto &ifc) { return ifc.get_name() == interface_name; });
            if (it != this->state_interfaces_.end()) {
                this->thruster_joint_st[i].reset(it.base());
            } else {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                             "Failed to find state interface: %s",
                             interface_name.c_str());
            }
        }
    }
    return cif::CallbackReturn::SUCCESS;
}

auto ucdhw::UmiusiController::on_deactivate(const rlc::State & /*previous_state*/)
  -> cif::CallbackReturn {
    return cif::CallbackReturn::SUCCESS;
}

auto ucdhw::UmiusiController::on_export_reference_interfaces()
  -> std::vector<hif::CommandInterface> {
    this->reference_interfaces_.resize(1); // おまじない

    // 目標値(メンバ変数)を`CommandInterface`として公開する。
    auto interfaces = std::vector<hif::CommandInterface>{};
    interfaces.emplace_back(
      hif::CommandInterface(this->get_node()->get_name() + std::string("/reference"),
                            hif::HW_IF_POSITION,
                            &this->reference));
    return interfaces;
}

auto ucdhw::UmiusiController::on_export_state_interfaces() -> std::vector<hif::StateInterface> {
    // 内部状態(メンバ変数)を`StateInterface`として公開する。
    auto interfaces = std::vector<hif::StateInterface>{};
    interfaces.emplace_back(hif::StateInterface(
      this->get_node()->get_name() + std::string("/state"), hif::HW_IF_POSITION, &this->reference));
    return interfaces;
}

auto ucdhw::UmiusiController::on_set_chained_mode(bool chained_mode) -> bool {
    if (chained_mode) {
        this->subscriber.reset();
    } else if (!this->subscriber) {
        this->subscriber = this->get_node()->create_subscription<std_msgs::msg::Float64>(
          "user_command",
          rclcpp::SystemDefaultsQoS(),
          [this](const std_msgs::msg::Float64::SharedPtr input) { this->last_msg = *input; });
    }
    return true;
}

auto ucdhw::UmiusiController::update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                                                const rclcpp::Duration & /*period*/)
  -> cif::return_type {
    // chained_modeであれば、何もしなくて良い。基本的には`update_and_write_commands`で処理する。
    if (!this->is_in_chained_mode()) {
        this->reference = this->last_msg.data;
    }
    return cif::return_type::OK;
}

auto ucdhw::UmiusiController::update_and_write_commands(const rclcpp::Time & /*time*/,
                                                        const rclcpp::Duration & /*period*/)
  -> cif::return_type {
    for (auto &cmd : this->thruster_joint_cmd) {
        auto res = cmd->set_value(this->reference);
        if (!res) {
            RCLCPP_WARN(this->get_node()->get_logger(),
                        "Failed to set command interface value: %s",
                        cmd->get_name().c_str());
        }
    }
    return cif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(umiusi_control_demo::UmiusiController,
                       controller_interface::ChainableControllerInterface)
