#ifndef AK60_ROBOT_DRIVER_CONTROL__ROCKER_BOGIE_CONTROLLER_HPP_
#define AK60_ROBOT_DRIVER_CONTROL__ROCKER_BOGIE_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <cmath>
#include <array>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace ak60_robot_driver_control
{

class RockerBogieController : public controller_interface::ControllerInterface
{
public:
  RockerBogieController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  
  // Cache các interface khi bắt đầu chạy
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  double ramp(double target, double current, double max_step);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::Twist last_cmd_vel_;
  bool cmd_received_ = false;

  double wheel_base_front_;
  double wheel_base_rear_;
  double track_width_;
  double wheel_radius_;
  
  double max_accel_linear_ = 1.0;
  double max_accel_angular_ = 1.0;

  double current_vx_ = 0.0;
  double current_vy_ = 0.0;
  double current_wz_ = 0.0;

  double x_ = 0.0;
  double y_ = 0.0;
  double heading_ = 0.0;

  // [TỐI ƯU HÓA] Biến thành viên lưu tọa độ (Tránh cấp phát lại)
  double wheel_coords_[6][2];

  // ===========================================================================================
  // [TỐI ƯU HÓA] Sử dụng duy nhất mảng con trỏ (std::array) cho tốc độ truy xuất O(1)
  std::array<hardware_interface::LoanedStateInterface*, 6> state_vel_;
  std::array<hardware_interface::LoanedStateInterface*, 6> state_steer_;

  std::array<hardware_interface::LoanedCommandInterface*, 6> cmd_vel_;
  std::array<hardware_interface::LoanedCommandInterface*, 6> cmd_steer_;
  // ===========================================================================================

  std::array<double, 6> last_steer_ = {0, 0, 0, 0, 0, 0};

  // ===== Params PRO =====
  double max_wheel_speed_ = 20.0;     // rad/s
  double max_steer_rate_ = 2.5;       // rad/s
  double cmd_timeout_ = 0.5;

  // ===== Time =====
  rclcpp::Time last_cmd_time_;

  // ================= STATE MACHINE =================
  enum class SteerState
  {
    IDLE,
    ALIGN,
    DRIVE,
    ROTATE,
    LATERAL
  };

  SteerState state_ = SteerState::IDLE;
  SteerState last_state_ = SteerState::IDLE;

  double last_vy_ = 0.0;

  // hysteresis
  double enter_rotate_wz_ = 0.025;
  double exit_rotate_wz_  = 0.015;

  double enter_move_speed_ = 0.06;
  double exit_move_speed_  = 0.04;
};

}  // namespace ak60_robot_driver_control

#endif  // AK60_ROBOT_DRIVER_CONTROL__ROCKER_BOGIE_CONTROLLER_HPP_

