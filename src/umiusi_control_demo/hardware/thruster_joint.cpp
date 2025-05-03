#include "umiusi_control_demo/hardware/thruster_joint.hpp"

namespace ucdhw = umiusi_control_demo::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto ucdhw::ThrusterJoint::on_init(const hif::HardwareInfo & /*info*/) -> hif::CallbackReturn {
    // 引数`info`には、URDFから読み取った情報が格納されている。
    // (init以降は`this->info_`に書き込まれるのでそちらを使う。)

    // 値の初期化
    this->command = 0.0;
    this->state = 0.0;
    return hif::CallbackReturn::SUCCESS;
}

auto ucdhw::ThrusterJoint::export_state_interfaces() -> std::vector<hif::StateInterface> {
    auto interfaces_to_export = std::vector<hif::StateInterface>{};
    auto joint = this->info_.joints.front();         // ※Jointが1つしかない前提
    auto interface = joint.state_interfaces.front(); // ※StateInterfaceが1つしかない前提
    interfaces_to_export.emplace_back(
      hif::StateInterface(joint.name,     // `thruster1_joint`など
                          interface.name, // `position`など
                          &this->state)); // Interfaceを介して公開する変数のアドレス
    return interfaces_to_export;
}

auto ucdhw::ThrusterJoint::export_command_interfaces() -> std::vector<hif::CommandInterface> {
    auto interfaces_to_export = std::vector<hif::CommandInterface>{};
    auto joint = this->info_.joints.front();           // ※Jointが1つしかない前提
    auto interface = joint.command_interfaces.front(); // ※CommandInterfaceが1つしかない前提
    interfaces_to_export.emplace_back(
      hif::CommandInterface(joint.name,       // `thruster1_joint`など
                            interface.name,   // `position`など
                            &this->command)); // Interfaceを介して公開する変数のアドレス
    return interfaces_to_export;
}

auto ucdhw::ThrusterJoint::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
  -> hif::return_type {
    // 本来はここでセンサの値を読み取り、内部状態を更新する
    return hif::return_type::OK;
}

auto ucdhw::ThrusterJoint::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  -> hif::return_type {
    // 本来はここでアクチュエータにコマンドを送る
    this->state = this->command;
    return hif::return_type::OK;
}

// `pluginlib`を使って`umiusi_control_demo::hardware::ThrusterJoint`クラスをプラグインとして登録する。
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(umiusi_control_demo::hardware::ThrusterJoint,
                       hardware_interface::ActuatorInterface)
