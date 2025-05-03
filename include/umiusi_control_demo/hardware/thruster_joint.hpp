#ifndef UMIUSI_CONTROL_DEMO_HARDWARE_THRUSTER_JOINT_HPP
#define UMIUSI_CONTROL_DEMO_HARDWARE_THRUSTER_JOINT_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "rclcpp/macros.hpp"

namespace umiusi_control_demo {

namespace hardware {

/**
 * @brief
 *
 * ## UMIUSIのスラスタの根元関節
 *
 * ROS2-controlのハードウェアコンポーネントを実装したもの。
 *
 * スラスタの根元関節の角度を内部状態として持ち、
 * `CommandInterface`経由で`Controller`からの命令を受け取って実行する。
 * また、`StateInterface`経由で`Controller`に状態を報告する。
 *
 * `CommandInterface`や`StateInterface`は内部でこのクラスのメンバ変数へのポインタを持つため、
 * トピックを経由せずに直接このクラスのメンバを書き換える。
 */
class ThrusterJoint : public hardware_interface::ActuatorInterface {
private:
    double command; // CommandInterfaceを介して公開する変数
    double state;   // StateInterfaceを介して公開する変数

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ThrusterJoint) // `shared_ptr`を生成する関数の定義。おまじない。

    ThrusterJoint() = default;

    /**
     * @brief ハードウェアの初期化時に呼ばれる関数。一回だけ呼ばれる。
     * @param info ハードウェア情報。URDFから読み取られた内容が入っている。
     * @return 成功した場合は`CallbackReturn::SUCCESS`、失敗した場合は`CallbackReturn::ERROR`
     */
    auto on_init(const hardware_interface::HardwareInfo &info)
      -> hardware_interface::CallbackReturn override;
    /**
     * @brief `StateInterface`を生成する関数
     * @return 生成した`StateInterface`の配列
     */
    auto export_state_interfaces() -> std::vector<hardware_interface::StateInterface> override;
    /**
     * @brief `CommandInterface`を生成する関数
     * @return 生成した`CommandInterface`の配列
     */
    auto export_command_interfaces() -> std::vector<hardware_interface::CommandInterface> override;
    /**
     * @brief 内部状態を更新する関数。通常は実際のハードウェアとの通信などが行われる。
     * @param time 現在時刻
     * @param period 前回の更新からの経過時間
     * @return
     * 成功した場合は`hardware_interface::return_type::OK`、失敗した場合は`hardware_interface::return_type::ERROR`
     */
    auto read(const rclcpp::Time &time, const rclcpp::Duration &period)
      -> hardware_interface::return_type override;
    /**
     * @brief 内部状態をする関数。通常は実際のハードウェアとの通信などが行われる。
     * @param time 現在時刻
     * @param period 前回の更新からの経過時間
     * @return
     * 成功した場合は`hardware_interface::return_type::OK`、失敗した場合は`hardware_interface::return_type::ERROR`
     */
    auto write(const rclcpp::Time &time, const rclcpp::Duration &period)
      -> hardware_interface::return_type override;
};

} // namespace hardware

} // namespace umiusi_control_demo

#endif // UMIUSI_CONTROL_DEMO_HARDWARE_THRUSTER_JOINT_HPP
