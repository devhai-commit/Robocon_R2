#include "ak60_robot_driver_control/rocker_bogie_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <algorithm> 

namespace ak60_robot_driver_control
{

controller_interface::CallbackReturn RockerBogieController::on_init()
{
  auto node = get_node();
  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&RockerBogieController::cmdVelCallback, this, std::placeholders::_1));

  node->get_parameter_or("wheel_base_front", wheel_base_front_, 0.28);
  node->get_parameter_or("wheel_base_rear",  wheel_base_rear_,  0.28);
  node->get_parameter_or("track_width",      track_width_,      0.47);
  node->get_parameter_or("wheel_radius",     wheel_radius_,     0.0635);

  // [TỐI ƯU HÓA] Gán giá trị tọa độ 6 bánh 1 lần duy nhất lúc khởi tạo
  double Lf = wheel_base_front_, Lr = wheel_base_rear_, W = track_width_;
  wheel_coords_[0][0] = Lf;  wheel_coords_[0][1] = W/2.0;  // FL
  wheel_coords_[1][0] = 0.0; wheel_coords_[1][1] = W/2.0;  // ML
  wheel_coords_[2][0] = -Lr; wheel_coords_[2][1] = W/2.0;  // RL
  wheel_coords_[3][0] = Lf;  wheel_coords_[3][1] = -W/2.0; // FR
  wheel_coords_[4][0] = 0.0; wheel_coords_[4][1] = -W/2.0; // MR
  wheel_coords_[5][0] = -Lr; wheel_coords_[5][1] = -W/2.0; // RR

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RockerBogieController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  return controller_interface::CallbackReturn::SUCCESS;
}

// ==========================================================
// [SỬA ĐỔI] HÀM ON_ACTIVATE: QUÉT VÀ LƯU PHẦN CỨNG
// ==========================================================
controller_interface::CallbackReturn RockerBogieController::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (int i = 1; i <= 6; ++i) {
    std::string vel_name = "wheel_joint_" + std::to_string(i) + "/velocity";
    std::string steer_name = "wheel_steer_" + std::to_string(i) + "/position";

    for (auto & interface : state_interfaces_) {
      if (interface.get_name() == vel_name)
        state_vel_[i-1] = &interface;
      if (interface.get_name() == steer_name)
        state_steer_[i-1] = &interface;
    }

    for (auto & interface : command_interfaces_) {
      if (interface.get_name() == vel_name)
        cmd_vel_[i-1] = &interface;
      if (interface.get_name() == steer_name)
        cmd_steer_[i-1] = &interface;
    }
  }

  cmd_received_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

void RockerBogieController::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_vel_ = *msg;
  cmd_received_ = true;
  last_cmd_time_ = get_node()->now();
}

controller_interface::InterfaceConfiguration RockerBogieController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 6; ++i) {
    config.names.push_back("wheel_joint_" + std::to_string(i) + "/velocity");
    config.names.push_back("wheel_steer_" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration RockerBogieController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 6; ++i) {
    config.names.push_back("wheel_joint_" + std::to_string(i) + "/velocity"); 
    config.names.push_back("wheel_steer_" + std::to_string(i) + "/position"); 
  }
  return config;
}

double RockerBogieController::ramp(double target, double current, double max_step)
{
  return current + std::clamp(target - current, -max_step, max_step);
}

// ==========================================================
// HÀM UPDATE: SIÊU NHẸ, KHÔNG CÓ BẤT KỲ XỬ LÝ CHUỖI NÀO NỮA
// ==========================================================
controller_interface::return_type RockerBogieController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double dt = period.seconds();

  // ================================
  // 1. ODOMETRY (STABLE)
  // ================================
  double sum_vx = 0.0, sum_vy = 0.0, sum_omega = 0.0;

  for (int i = 0; i < 6; ++i) {
    double wheel_vel = state_vel_[i]->get_value();
    double steer = state_steer_[i]->get_value();

    double v = wheel_vel * wheel_radius_;
    double vx = v * cos(steer);
    double vy = v * sin(steer);

    sum_vx += vx;
    sum_vy += vy;

    double x = wheel_coords_[i][0];
    double y = wheel_coords_[i][1];
    double r2 = x*x + y*y;

    if (r2 > 1e-3) {
      sum_omega += (vy * x - vx * y) / r2;
    }
  }

  double robot_vx = sum_vx / 6.0;
  double robot_vy = sum_vy / 6.0;
  double robot_w = sum_omega / 6.0;

  heading_ += robot_w * dt;

  x_ += (robot_vx * cos(heading_) - robot_vy * sin(heading_)) * dt;
  y_ += (robot_vx * sin(heading_) + robot_vy * cos(heading_)) * dt;

  // ================================
  // 2. ODOM PUBLISH + TF
  // ================================
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;

  tf2::Quaternion q;
  q.setRPY(0, 0, heading_);

  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = robot_vx;
  odom.twist.twist.linear.y = robot_vy;
  odom.twist.twist.angular.z = robot_w;

  // covariance basic
  odom.pose.covariance[0] = 0.01;
  odom.pose.covariance[7] = 0.01;
  odom.pose.covariance[35] = 0.02;

  odom_pub_->publish(odom);

  // geometry_msgs::msg::TransformStamped t;
  // t.header.stamp = time;
  // t.header.frame_id = "odom";
  // t.child_frame_id = "base_link";

  // t.transform.translation.x = x_;
  // t.transform.translation.y = y_;
  // t.transform.rotation = odom.pose.pose.orientation;

  // tf_broadcaster_->sendTransform(t);

  // ================================
  // 3. CMD TIMEOUT (SAFETY)
  // ================================
  double target_vx = 0, target_vy = 0, target_wz = 0;

  if (cmd_received_ &&
      (time - last_cmd_time_).seconds() < cmd_timeout_) {
    target_vx = last_cmd_vel_.linear.x;
    target_vy = last_cmd_vel_.linear.y;
    target_wz = last_cmd_vel_.angular.z;
  }

  target_vx = std::clamp(target_vx, -0.8, 0.8);
  target_vy = std::clamp(target_vy, -0.8, 0.8);
  target_wz = std::clamp(target_wz, -1.0, 1.0);

  // ramp body
  double step_lin = max_accel_linear_ * dt;
  double step_ang = max_accel_angular_ * dt;

  current_vx_ = ramp(target_vx, current_vx_, step_lin);
  current_vy_ = ramp(target_vy, current_vy_, step_lin);
  current_wz_ = ramp(target_wz, current_wz_, step_ang);

  // ==========================================================
  // 4. INVERSE KINEMATICS VẠN NĂNG (BẢN FIX URDF LIMITS)
  // ==========================================================

  double raw_speeds[6];
  double target_steers[6];
  double max_steer_error = 0.0; // Dùng để tính hệ số đồng bộ chung sau này

  for (int i = 0; i < 6; ++i)
  {
    double x = wheel_coords_[i][0];
    double y = wheel_coords_[i][1];

    double vx_w = current_vx_ - current_wz_ * y;
    double vy_w = current_vy_ + current_wz_ * x;

    double speed = std::hypot(vx_w, vy_w) / wheel_radius_;
    double current = state_steer_[i]->get_value();
    double target_steer = current;

    if (speed < 0.001) 
    {
      speed = 0.0;
    } 
    else 
    {
      double desired_steer = atan2(vy_w, vx_w);

      // 1. Tìm góc lệch (Diff) thuần túy
      double diff = atan2(sin(desired_steer - current), cos(desired_steer - current));
      target_steer = current + diff;

      // 2. TỐI ƯU ±90 ĐỘ (Shortest Path)
      if (std::abs(diff) > M_PI / 2.0) 
      {
        target_steer = (diff > 0) ? target_steer - M_PI : target_steer + M_PI;
        speed *= -1.0;
      }

      // 3. XỬ LÝ KẸT CƠ KHÍ (URDF JOINT LIMITS) - NGUYÊN NHÂN CHÍNH
      // Giới hạn vật lý trục lái Rocker-Bogie thường không vượt quá +-90 độ.
      // Nếu target vượt ngưỡng an toàn, bắt buộc phải lật 180 độ và đảo chiều motor
      // để đưa góc lái quay về vùng an toàn [-90, 90].
      double joint_limit = 1.74; // ~100 độ (Cho phép lố 1 chút để bù nhiễu khi đi ngang)
      
      if (target_steer > joint_limit) {
        target_steer -= M_PI;
        speed *= -1.0; 
      } else if (target_steer < -joint_limit) {
        target_steer += M_PI;
        speed *= -1.0;
      }

      // 4. TÍNH SAI SỐ GÓC SAU KHI ĐÃ ÉP LIMIT
      double err = std::abs(atan2(sin(target_steer - current), cos(target_steer - current)));
      if (err > max_steer_error) {
          max_steer_error = err;
      }
    }

    // Lưu tạm tốc độ thô và góc đích vào mảng
    raw_speeds[i] = speed;
    target_steers[i] = target_steer;
  }

  // --- BƯỚC CHUYỂN TIẾP: TÍNH HỆ SỐ ĐỒNG BỘ CHUNG (GLOBAL SYNC) ---
  // Toàn bộ 6 bánh sẽ dùng chung một tỷ lệ giảm tốc độ dựa trên bánh bị lệch nặng nhất
  double global_steer_scale = std::clamp(1.0 - (max_steer_error / (M_PI / 4.0)), 0.0, 1.0);

  // --- VÒNG LẶP 2: XUẤT LỆNH XUỐNG ĐỘNG CƠ CÙNG LÚC ---
  for (int i = 0; i < 6; ++i)
  {
    cmd_vel_[i]->set_value(raw_speeds[i] * global_steer_scale);
    cmd_steer_[i]->set_value(target_steers[i]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace ak60_robot_driver_control

PLUGINLIB_EXPORT_CLASS(ak60_robot_driver_control::RockerBogieController, 
  controller_interface::ControllerInterface)
