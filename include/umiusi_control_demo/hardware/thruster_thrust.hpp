#ifndef UMIUSI_CONTROL_DEMO_HARDWARE_THRUSTER_THRUST_HPP
#define UMIUSI_CONTROL_DEMO_HARDWARE_THRUSTER_THRUST_HPP

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
 * ## UMIUSIのスラスタの推力出力
 *
 * ROS2-controlのハードウェアコンポーネントを実装したもの。
 * 構成は`umiusi_control_demo::hardware::ThrusterJoint`クラスと全く一緒なので
 * 詳しくはそちらを参照のこと。
 */
class ThrusterThrust : public hardware_interface::ActuatorInterface {
private:
    double command;
    double state;

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ThrusterThrust)

    ThrusterThrust() = default;

    auto on_init(const hardware_interface::HardwareInfo &info)
      -> hardware_interface::CallbackReturn override;
    auto export_state_interfaces() -> std::vector<hardware_interface::StateInterface> override;
    auto export_command_interfaces() -> std::vector<hardware_interface::CommandInterface> override;
    auto read(const rclcpp::Time &time, const rclcpp::Duration &period)
      -> hardware_interface::return_type override;
    auto write(const rclcpp::Time &time, const rclcpp::Duration &period)
      -> hardware_interface::return_type override;
};

} // namespace hardware

} // namespace umiusi_control_demo

#endif // UMIUSI_CONTROL_DEMO_HARDWARE_THRUSTER_THRUST_HPP
