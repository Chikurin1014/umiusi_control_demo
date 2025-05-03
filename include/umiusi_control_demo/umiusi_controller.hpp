#ifndef UMIUSI_CONTROL_DEMO_UMIUMSI_CONTROLLER
#define UMIUSI_CONTROL_DEMO_UMIUMSI_CONTROLLER

#include <memory>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64.hpp"

namespace umiusi_control_demo {

/**
 * @brief
 *
 * ## UMIUSIの制御器
 *
 * ROS2-controlのコントローラを実装したもの。
 *
 * 別で定義されているhardwareの`StateInterface`の値をもとに計算し、
 * その結果を`CommandInterface`に書き込む。
 *
 * その際、`**Interface`はそれぞれ`Loaned**Interface`として扱われ、
 * ある程度リアルタイム性が確保される。(書き込みの際の排他制御など)
 *
 * このコントローラは、UMIUSIの4つのスラスターの根元関節を制御するためのもの。
 * (入力値を出力に横流しするだけ)
 *
 * @class UmiusiController
 */
class UmiusiController : public controller_interface::ChainableControllerInterface {
private:
    // ユーザーからのコマンドを受け取るためのSubscriber
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber;
    // subscribeしたトピックの格納先
    std_msgs::msg::Float64 last_msg;

    // `CommandInterface`のエイリアスとして使う。
    std::array<std::unique_ptr<hardware_interface::LoanedCommandInterface>, 4> thruster_joint_cmd;
    // `StateInterface`のエイリアスとして使う。
    std::array<std::unique_ptr<hardware_interface::LoanedStateInterface>, 4> thruster_joint_st;

    // 目標値。これをこのコントローラが受け取る入力とする。
    double reference;

public:
    UmiusiController() = default;

    /**
     * @brief 扱う`CommandInterface`を指定する関数。余計なものをスコープに入れないようにする。
     * @return 扱う`CommandInterface`のリスト
     */
    auto command_interface_configuration() const
      -> controller_interface::InterfaceConfiguration override;
    /**
     * @brief 扱う`StateInterface`を指定する関数。余計なものをスコープに入れないようにする。
     * @return 扱う`StateInterface`のリスト
     */
    auto state_interface_configuration() const
      -> controller_interface::InterfaceConfiguration override;
    /**
     * @brief 初期化時に呼ばれる関数。一回だけ呼ばれる。
     * @return 成功した場合は`CallbackReturn::SUCCESS`、失敗した場合は`CallbackReturn::ERROR`
     */
    auto on_init() -> CallbackReturn override;
    /**
     * @brief 構成時に呼ばれる関数。一回だけ呼ばれる。
     * @param previous_state
     * @return 成功した場合は`CallbackReturn::SUCCESS`、失敗した場合は`CallbackReturn::ERROR`
     */
    auto on_configure(const rclcpp_lifecycle::State &previous_state) -> CallbackReturn override;
    /**
     * @brief アクティブ化時に呼ばれる関数。アクティブ化するたびに呼ばれる。
     * @param previous_state
     * @return 成功した場合は`CallbackReturn::SUCCESS`、失敗した場合は`CallbackReturn::ERROR`
     */
    auto on_activate(const rclcpp_lifecycle::State &previous_state) -> CallbackReturn override;
    /**
     * @brief 非アクティブ化時に呼ばれる関数。非アクティブ化するたびに呼ばれる。
     * @param previous_state
     * @return 成功した場合は`CallbackReturn::SUCCESS`、失敗した場合は`CallbackReturn::ERROR`
     */
    auto on_deactivate(const rclcpp_lifecycle::State &previous_state) -> CallbackReturn override;
    /**
     * @brief 制御入力(目標値など)を`CommandInterface`として公開する関数
     * @return 公開する`CommandInterface`のリスト
     */
    auto on_export_reference_interfaces()
      -> std::vector<hardware_interface::CommandInterface> override;
    /**
     * @brief 内部状態を`StateInterface`として公開する関数
     * @return 公開する`StateInterface`のリスト
     */
    auto on_export_state_interfaces() -> std::vector<hardware_interface::StateInterface> override;
    /**
     * @brief 制御入力をTopicとして受け取るかどうかを切り替えたときに呼ばれる関数
     * @param chained_mode
     * @return 成功した場合は`true`、失敗した場合は`false`
     */
    auto on_set_chained_mode(bool chained_mode) -> bool override;
    /**
     * @brief トピックから制御入力を更新するための関数。ループ毎に呼ばれる。
     * @param time 現在時刻
     * @param period 前回のループからの経過時間
     * @return 成功した場合は`return_type::OK`、失敗した場合は`return_type::ERROR`
     */
    auto update_reference_from_subscribers(const rclcpp::Time &time, const rclcpp::Duration &period)
      -> controller_interface::return_type override;
    /**
     * @brief コントローラの状態を更新する関数。ループ毎に呼ばれる。
     * @param time 現在時刻
     * @param period 前回のループからの経過時間
     * @return 成功した場合は`return_type::OK`、失敗した場合は`return_type::ERROR`
     */
    auto update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period)
      -> controller_interface::return_type override;
};
}

#endif // UMIUSI_CONTROL_DEMO_UMIUMSI_CONTROLLER
