#ifndef ROBOT_HW_INTERFACE__HW_ROBOT_INTERFACE_HPP_
#define ROBOT_HW_INTERFACE__HW_ROBOT_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <array> // [TỐI ƯU HÓA 1]: Dùng Array thay cho Map
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace robot_hw_interface
{

// [TỐI ƯU HÓA 2]: Phân loại Joint bằng Enum để check bằng số nguyên O(1) thay vì String
enum class JointType {
  UNKNOWN = 0,
  WHEEL_CAN,
  ARM_CAN,
  STEER_SERIAL,
  GRIPPER_WAVESHARE
};

class Ak60RobotHwInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Ak60RobotHwInterface)

  ~Ak60RobotHwInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr servo_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr servo_fb_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr waveshare_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr waveshare_fb_sub_;

  std::vector<size_t> can_joint_indices_;
  std::vector<size_t> serial_joint_indices_;
  std::vector<size_t> waveshare_joint_indices_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;

  std::unordered_map<int, double> servo_id_offsets_;

  std::vector<double> servo_offset_ = {0.0, -10.0, 0.0, 10.0, -10.0, 0.0, 10.0, 0.0, 0.0};

  // [TỐI ƯU HÓA 2]: Mảng Cache để lưu trước loại Joint và ID
  std::vector<JointType> joint_types_cache_;
  std::vector<int> joint_ids_cache_;

  // [TỐI ƯU HÓA 1]: Dùng mảng tĩnh 32 phần tử (do ID động cơ lớn nhất là 16)
  struct MotorState { double pos = 0.0; double vel = 0.0; double eff = 0.0; bool updated = false; };
  std::array<MotorState, 32> can_rx_buffer_; 
  
  std::vector<int32_t> serial_rx_buffer_;
  std::vector<int32_t> gripper_rx_buffer_;

  // [TỐI ƯU HÓA 3]: Pre-allocate bản tin ROS để không cấp phát mảng động khi write
  std_msgs::msg::Int32MultiArray serial_cmd_msg_;
  std_msgs::msg::Int32MultiArray waveshare_cmd_msg_;
  
  std::mutex data_mutex_;

  // Callbacks
  void can_rx_callback(const can_msgs::msg::Frame::SharedPtr msg);
  void servo_fb_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void gripper_fb_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  int float_to_uint(float x, float x_min, float x_max, int bits);
  float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

}  // namespace robot_hw_interface

#endif  // ROBOT_HW_INTERFACE__HW_ROBOT_INTERFACE_HPP_