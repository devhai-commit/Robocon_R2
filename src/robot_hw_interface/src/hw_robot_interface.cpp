#include "robot_hw_interface/hw_robot_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath> 
#include <string>
#include <algorithm>

#define CAN_PACKET_SET_MIT 8 
#define CAN_PACKET_SET_POS_SPD 6

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define T_MIN -15.0f
#define T_MAX 15.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

namespace robot_hw_interface
{

int get_can_id(const std::string& joint_name) {
  if (joint_name == "wheel_joint_1") return 11;
  if (joint_name == "wheel_joint_2") return 12;
  if (joint_name == "wheel_joint_3") return 13;
  if (joint_name == "wheel_joint_4") return 14;
  if (joint_name == "wheel_joint_5") return 15;
  if (joint_name == "wheel_joint_6") return 16;
  if (joint_name == "arm_joint_1") return 104;
  return -1; 
}

int get_serial_id(const std::string& joint_name) {
  if (joint_name == "wheel_steer_1") return 1;
  if (joint_name == "wheel_steer_2") return 2;
  if (joint_name == "wheel_steer_3") return 3;
  if (joint_name == "wheel_steer_4") return 4;
  if (joint_name == "wheel_steer_5") return 5;
  if (joint_name == "wheel_steer_6") return 6;
  if (joint_name == "arm_joint_2")   return 7; 
  if (joint_name == "arm_joint_3")   return 8; 
  return -1; 
}

int get_gripper_id(const std::string& joint_name) {
  if (joint_name == "gripper_joint") return 1; 
  return -1;
}

hardware_interface::CallbackReturn Ak60RobotHwInterface::on_init(const hardware_interface::HardwareInfo & info){
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  size_t num_joints = info_.joints.size();
  hw_commands_.resize(num_joints, 0.0);
  hw_states_position_.resize(num_joints, 0.0);
  hw_states_velocity_.resize(num_joints, 0.0);
  hw_states_effort_.resize(num_joints, 0.0);

  joint_types_cache_.resize(num_joints, JointType::UNKNOWN);
  joint_ids_cache_.resize(num_joints, -1);

  serial_cmd_msg_.data.resize(8, 0); 
  waveshare_cmd_msg_.data.resize(1, 0); 
  
  serial_rx_buffer_.resize(8, 0);
  gripper_rx_buffer_.resize(1, 0);

  for (size_t i = 0; i < num_joints; ++i) {
    std::string j_name = info_.joints[i].name;

    int can_id = get_can_id(j_name);
    if (can_id != -1) {
      can_joint_indices_.push_back(i);
      joint_ids_cache_[i] = can_id;
      
      if (j_name.find("arm_joint") != std::string::npos) {
        joint_types_cache_[i] = JointType::ARM_CAN;
      } else {
        joint_types_cache_[i] = JointType::WHEEL_CAN;
      }
    }
    else {
      int serial_id = get_serial_id(j_name);
      if (serial_id != -1) {
        serial_joint_indices_.push_back(i);
        joint_ids_cache_[i] = serial_id;
        joint_types_cache_[i] = JointType::STEER_SERIAL;
      }
      else {
        int gripper_id = get_gripper_id(j_name);
        if (gripper_id != -1) {
          waveshare_joint_indices_.push_back(i);
          joint_ids_cache_[i] = gripper_id;
          joint_types_cache_[i] = JointType::GRIPPER_WAVESHARE;
          hw_commands_[i] = 0.7;
        }
      }
    }

    if (info_.joints[i].parameters.find("offset") != info_.joints[i].parameters.end() &&
        info_.joints[i].parameters.find("motor_id") != info_.joints[i].parameters.end()) 
    {
      int id = std::stoi(info_.joints[i].parameters.at("motor_id"));
      double offset_val = std::stod(info_.joints[i].parameters.at("offset"));
      
      // Lưu vào danh bạ: ID -> Offset
      servo_id_offsets_[id] = offset_val;
      
      RCLCPP_INFO(rclcpp::get_logger("Ak60RobotHwInterface"), 
                  "Đã nạp Offset cho Servo ID %d: %.1f xung", id, offset_val);
    }
  }

  node_ = rclcpp::Node::make_shared("robot_hw_interface_node");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------
// CÁC HÀM BỊ THIẾU ĐÃ ĐƯỢC PHỤC HỒI
// -------------------------------------------------------------

hardware_interface::CallbackReturn Ak60RobotHwInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  can_tx_pub_ = node_->create_publisher<can_msgs::msg::Frame>("/canfd/tx", 10);
  can_rx_sub_ = node_->create_subscription<can_msgs::msg::Frame>("/canfd/rx", rclcpp::SensorDataQoS(), std::bind(&Ak60RobotHwInterface::can_rx_callback, this, std::placeholders::_1));

  servo_cmd_pub_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>("/servo/cmd_count", 10);
  servo_fb_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>("/servo/fb_count", 10, std::bind(&Ak60RobotHwInterface::servo_fb_callback, this, std::placeholders::_1));

  waveshare_cmd_pub_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>("/gripper/cmd_pos", 10);
  waveshare_fb_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>("/gripper/feedback", 10, std::bind(&Ak60RobotHwInterface::gripper_fb_callback, this, std::placeholders::_1));
    
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Ak60RobotHwInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & state_interface : info_.joints[i].state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
      } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
      } else if (state_interface.name == hardware_interface::HW_IF_EFFORT) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Ak60RobotHwInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (!info_.joints[i].command_interfaces.empty()) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, info_.joints[i].command_interfaces[0].name, &hw_commands_[i]));
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn Ak60RobotHwInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  // ====================================================================================================================
  // BỔ SUNG: SET ORIGIN CHO ARM_JOINT_1 NGAY LÚC KHỞI ĐỘNG
  // ============================================================================================================
  int arm_1_id = get_can_id("arm_joint_1"); // Lấy tự động ID của Arm 1

  can_msgs::msg::Frame origin_msg;
  origin_msg.id = arm_1_id | (5 << 8); // 5 là CAN_PACKET_SET_ORIGIN_HERE
  origin_msg.is_extended = true;
  origin_msg.dlc = 1;
  
  // Quan trọng: Dùng 0 (Temporary Origin) thay vì 1 (Permanent). 
  // Việc lưu vĩnh viễn (1) mỗi lần bật máy sẽ làm giảm tuổi thọ bộ nhớ Flash của motor.
  origin_msg.data[0] = 0; 

  if (can_tx_pub_) {
    can_tx_pub_->publish(origin_msg);
    RCLCPP_INFO(rclcpp::get_logger("Ak60RobotHwInterface"), "Đã gửi lệnh Set Origin Here tới Arm 1 (ID: %d)", arm_1_id);
  }

  // Nghỉ 100ms để đợi Motor lưu xong điểm 0 trước khi nhận lệnh điều khiển
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  // =============================================================================================================

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ak60RobotHwInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // RCLCPP_INFO(rclcpp::get_logger("Ak60RobotHwInterface"), "Đang thu Arm 1 về 0 độ an toàn...");

  // // 1. GỬI LỆNH MODE 6 (POS & SPEED) YÊU CẦU ARM 1 VỀ GÓC 0
  // int arm_1_id = get_can_id("arm_joint_1");

  // can_msgs::msg::Frame can_msg;
  // can_msg.id = arm_1_id | (6 << 8); // 6 là CAN_PACKET_SET_POS_SPD
  // can_msg.is_extended = true;  
  // can_msg.dlc = 8;

  // int32_t pos_int32 = 0; // Vị trí 0 độ
  
  // // Tốc độ và gia tốc thu về (Để chậm cho an toàn, ví dụ 1000 ERPM)
  // int16_t spd_int16 = static_cast<int16_t>(1000.0f / 10.0f);
  // int16_t rpa_int16 = static_cast<int16_t>(1000.0f / 10.0f);

  // can_msg.data[0] = (pos_int32 >> 24) & 0xFF;
  // can_msg.data[1] = (pos_int32 >> 16) & 0xFF;
  // can_msg.data[2] = (pos_int32 >> 8) & 0xFF;
  // can_msg.data[3] = pos_int32 & 0xFF;
  // can_msg.data[4] = (spd_int16 >> 8) & 0xFF;
  // can_msg.data[5] = spd_int16 & 0xFF;
  // can_msg.data[6] = (rpa_int16 >> 8) & 0xFF;
  // can_msg.data[7] = rpa_int16 & 0xFF;

  // if (can_tx_pub_) {
  //   can_tx_pub_->publish(can_msg);
  // }

  // ===============================================================
  // 2. BẮT BUỘC PHẢI CÓ DÒNG NÀY: ÉP TIẾN TRÌNH ĐỨNG CHỜ 2 GIÂY
  // Giữ cổng CAN và nguồn điện sống đủ lâu để tay máy kịp chạy về 0
  // ===============================================================
  // rclcpp::sleep_for(std::chrono::milliseconds(2000)); 

  // RCLCPP_INFO(rclcpp::get_logger("Ak60RobotHwInterface"), "Đã thu Arm 1 xong! Tắt phần cứng an toàn.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

int Ak60RobotHwInterface::float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x = x_max;
  else if(x < x_min) x = x_min;
  return (int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float Ak60RobotHwInterface::uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// -------------------------------------------------------------
// CALLBACKS VÀ READ/WRITE
// -------------------------------------------------------------

void Ak60RobotHwInterface::can_rx_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (msg->is_extended && msg->dlc >= 8) {
    int motor_id = msg->id & 0xFF; 
    
    if(motor_id >= 0 && motor_id < 32) {
      int16_t pos_int = (msg->data[0] << 8) | msg->data[1];
      int16_t spd_int = (msg->data[2] << 8) | msg->data[3];
      int16_t cur_int = (msg->data[4] << 8) | msg->data[5];

      float pos_deg = pos_int * 0.1f;    
      float spd_erpm = spd_int * 10.0f;   
      float cur_A   = cur_int * 0.01f;   

      std::lock_guard<std::mutex> lock(data_mutex_);
      can_rx_buffer_[motor_id].pos = pos_deg ;
      can_rx_buffer_[motor_id].vel = spd_erpm;
      can_rx_buffer_[motor_id].eff = static_cast<double>(cur_A);
      can_rx_buffer_[motor_id].updated = true;
    }
  }
}

void Ak60RobotHwInterface::servo_fb_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  serial_rx_buffer_ = msg->data; 
}

void Ak60RobotHwInterface::gripper_fb_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  gripper_rx_buffer_ = msg->data;
}

hardware_interface::return_type Ak60RobotHwInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // const double KT_NM_A = 0.135; 
  const double DEG_TO_RAD = M_PI / 180.0;

  // ==========================================================
  // 1. CAN BUS (Cập nhật liên tục ở 50Hz)
  // ==========================================================
  for (size_t j_idx : can_joint_indices_) {
    int motor_id = joint_ids_cache_[j_idx];
    JointType j_type = joint_types_cache_[j_idx]; // Đọc loại Joint để phân loại tính toán
    
    if (motor_id >= 0 && motor_id < 32 && can_rx_buffer_[motor_id].updated) {
      
      // Dữ liệu thô lấy từ Callback (Đang là Degree và ERPM)
      double raw_pos_deg = can_rx_buffer_[motor_id].pos;
      double raw_vel_erpm = can_rx_buffer_[motor_id].vel; 
      double raw_eff_amp = can_rx_buffer_[motor_id].eff;

      // ===========================================
      // XỬ LÝ CHO BÁNH XE (WHEEL_CAN)
      // ===========================================
      if (j_type == JointType::WHEEL_CAN) {
        if (motor_id == 11 || motor_id == 12 || motor_id == 13) {
          raw_pos_deg = -raw_pos_deg;
          raw_vel_erpm = -raw_vel_erpm;
          raw_eff_amp = -raw_eff_amp;
        }
        
        // 1. Vị trí (Radian) = (Độ thô / Hộp số 6.0) * (PI / 180)
        double wheel_rad = (raw_pos_deg / 6.0) * (M_PI / 180.0);
        // 2. Vận tốc (Rad/s) = (ERPM / PolePairs 14.0) * (2PI / 60) / Hộp số 6.0
        double wheel_rad_s = (raw_vel_erpm / 14.0) * (2.0 * M_PI / 60.0) / 6.0;
        
        hw_states_position_[j_idx] = wheel_rad;        
        hw_states_velocity_[j_idx] = wheel_rad_s;   
        hw_states_effort_[j_idx]   = raw_eff_amp * 0.135; 
      }
      // ===========================================
      // XỬ LÝ CHO CÁNH TAY TỊNH TIẾN (ARM_CAN - ID 3)
      // ===========================================
      else if (j_type == JointType::ARM_CAN) {
        // 1. Đổi Vị trí từ Độ sang Radian
        double arm_rad = (raw_pos_deg/6.0) * (M_PI / 180.0);
        
        // 2. Đổi Vị trí từ Radian sang Mét (Prismatic R = 40.0mm)
        // Mét = Radian * R(0.04 m)
        double arm_meters = arm_rad * (40.0 / 1000.0);

        // 3. Đổi Vận tốc sang m/s
        // Vận tốc góc (rad/s) của AK60 (Giả sử 14 Pole pairs, không qua hộp số phụ)
        double arm_rad_s = (raw_vel_erpm / 14.0) * (2.0 * M_PI / 60.0);
        double arm_m_s = arm_rad_s * (40.0 / 1000.0);

        hw_states_position_[j_idx] = arm_meters;        
        hw_states_velocity_[j_idx] = arm_m_s;   
        hw_states_effort_[j_idx]   = raw_eff_amp * 0.135; // Lực Momen hiện tại (N.m)
      }

      can_rx_buffer_[motor_id].updated = false; 
    }
  }

  // ==========================================================
  // 2. NHÁNH TẦN SỐ THẤP (SERIAL & WAVESHARE) - Đồng bộ 25Hz
  // ==========================================================
  static int slow_read_prescaler = 0;
  slow_read_prescaler++;
  
  if (slow_read_prescaler >= 2) {  // 50Hz / 2 = 25Hz
    
    // Đọc trạng thái Serial Servo
    for (size_t j_idx : serial_joint_indices_) {
      int servo_id = joint_ids_cache_[j_idx];
      if (servo_id >= 1 && servo_id <= (int)serial_rx_buffer_.size()) {
        double raw_tick = static_cast<double>(serial_rx_buffer_[servo_id - 1]);

        // raw_tick += servo_offset_[servo_id];
        if (servo_id_offsets_.count(servo_id)) {
            raw_tick -= servo_id_offsets_[servo_id];
        }

        double center = (servo_id == 7) ? 365.0 : 500.0;
        double rad = (raw_tick - center) * 0.24 * DEG_TO_RAD;

        if (servo_id >= 1 && servo_id <= 6) rad = -rad;
        hw_states_position_[j_idx] = rad;
      }
    }

    // Đọc trạng thái Tay gắp Waveshare
    for (size_t j_idx : waveshare_joint_indices_) {
      int gripper_id = joint_ids_cache_[j_idx];
      if (gripper_id >= 1 && gripper_id <= (int)gripper_rx_buffer_.size()) {
        double raw_tick = static_cast<double>(gripper_rx_buffer_[gripper_id - 1]);
        hw_states_position_[j_idx] = ((raw_tick - 0.0) / 4096.0) * (2.0 * M_PI);
      }
    }

    slow_read_prescaler = 0; // Đặt lại chu kỳ
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type Ak60RobotHwInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t j_idx : can_joint_indices_) {
    int motor_id = joint_ids_cache_[j_idx];
    JointType j_type = joint_types_cache_[j_idx];

    can_msgs::msg::Frame can_msg;
    bool should_publish = false;

    if (j_type == JointType::WHEEL_CAN) {
      float v_des = static_cast<float>(hw_commands_[j_idx]); 
      if (motor_id == 11 || motor_id == 12 || motor_id == 13) v_des = -v_des;
      
      int p_int = float_to_uint(0.0f, P_MIN, P_MAX, 16);
      int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
      int kp_int = float_to_uint(0.0f, KP_MIN, KP_MAX, 12);
      int kd_int = float_to_uint(3.0f, KD_MIN, KD_MAX, 12);
      int t_int = float_to_uint(0.0f, T_MIN, T_MAX, 12);

      can_msg.id = motor_id | (CAN_PACKET_SET_MIT << 8); 
      can_msg.is_extended = true; 
      can_msg.dlc = 8;
      
      can_msg.data[0] = kp_int >> 4;
      can_msg.data[1] = ((kp_int & 0xF) << 4) | (kd_int >> 8);
      can_msg.data[2] = kd_int & 0xFF;
      can_msg.data[3] = p_int >> 8;
      can_msg.data[4] = p_int & 0xFF;
      can_msg.data[5] = v_int >> 4;
      can_msg.data[6] = ((v_int & 0xF) << 4) | (t_int >> 8);
      can_msg.data[7] = t_int & 0xFF;

      should_publish = true;
    } 

    else if (j_type == JointType::ARM_CAN) {
      // 1. Lấy lệnh gốc từ Controller và lấy thông số hiện tại
      double target_cmd = hw_commands_[j_idx];

      target_cmd = std::clamp(target_cmd, 0.0, 0.76);

      float target_rad_from_meters = (static_cast<float>(target_cmd) * 1000.0f) / 40.0f;
      
      // Đổi Radian sang Độ
      float pos_deg = target_rad_from_meters * (180.0f / M_PI);
      
      int32_t pos_int32 = static_cast<int32_t>(pos_deg * 10000.0f);
      int16_t spd_int16 = static_cast<int16_t>(2000.0f / 10.0f);
      int16_t rpa_int16 = static_cast<int16_t>(2000.0f / 10.0f);

      can_msg.id = motor_id | (CAN_PACKET_SET_POS_SPD << 8); 
      can_msg.is_extended = true;  
      can_msg.dlc = 8;

      can_msg.data[0] = (pos_int32 >> 24) & 0xFF;
      can_msg.data[1] = (pos_int32 >> 16) & 0xFF;
      can_msg.data[2] = (pos_int32 >> 8) & 0xFF;
      can_msg.data[3] = pos_int32 & 0xFF;
      can_msg.data[4] = (spd_int16 >> 8) & 0xFF;
      can_msg.data[5] = spd_int16 & 0xFF;
      can_msg.data[6] = (rpa_int16 >> 8) & 0xFF;
      can_msg.data[7] = rpa_int16 & 0xFF;

      should_publish = true;
    }

    if (should_publish) {
      can_tx_pub_->publish(can_msg);
    }
  }

  // ==========================================================
  // 2. NHÁNH TẦN SỐ THẤP (SERIAL & WAVESHARE) - Đồng bộ 25Hz
  // ==========================================================
  static int slow_write_prescaler = 0;
  slow_write_prescaler++;

  if (slow_write_prescaler >= 2) { // 50Hz / 2 = 25Hz
    
    // --- Gói lệnh Serial ---
    for (size_t j_idx : serial_joint_indices_) {
      int servo_id = joint_ids_cache_[j_idx];
      if (servo_id < 1 || servo_id > 8) continue;

      double cmd_rad = hw_commands_[j_idx];
      if (servo_id >= 1 && servo_id <= 6) cmd_rad = -cmd_rad;

      double center = (servo_id == 7) ? 365.0 : 500.0;

      double raw_cmd = (cmd_rad * (180.0 / M_PI) / 0.24) + center;

      // raw_cmd -= servo_offset_[servo_id];

      if (servo_id_offsets_.count(servo_id)) {
          raw_cmd += servo_id_offsets_[servo_id];
      }

      if (servo_id >= 1 && servo_id <= 6) {
        raw_cmd = std::clamp(raw_cmd, 110.0, 890.0);
      } else if (servo_id == 7) { 
        raw_cmd = std::clamp(raw_cmd, 340.0, 875.0);
      } else if (servo_id == 8) {
        raw_cmd = std::clamp(raw_cmd, 125.0, 875.0);
      }
      
      serial_cmd_msg_.data[servo_id - 1] = static_cast<int32_t>(std::round(raw_cmd));
    }
    
    if(!serial_joint_indices_.empty()) {
      servo_cmd_pub_->publish(serial_cmd_msg_);
    }

    // --- Gói lệnh Waveshare ---
    for (size_t j_idx : waveshare_joint_indices_) {
      int gripper_id = joint_ids_cache_[j_idx];
      if (gripper_id != 1) continue; 

      double cmd_rad = hw_commands_[j_idx];
      double cmd_raw = cmd_rad * 4096.0 / (2.0 * M_PI);
      cmd_raw = std::clamp(cmd_raw, 0.0, 2050.0);
      
      waveshare_cmd_msg_.data[0] = static_cast<int32_t>(std::round(cmd_raw));
    }
    
    if (!waveshare_joint_indices_.empty()) {
      waveshare_cmd_pub_->publish(waveshare_cmd_msg_);
    }

    slow_write_prescaler = 0; // Đặt lại chu kỳ
  }

  return hardware_interface::return_type::OK;
}

// ==========================================================
// THÊM HÀM HỦY (DESTRUCTOR) CHỐNG LỖI KHI BẤM CTRL+C
// ==========================================================
Ak60RobotHwInterface::~Ak60RobotHwInterface()
{
  // 1. Dừng các cờ (flags) vòng lặp while bên trong thread (nếu bạn có dùng)
  // Ví dụ: is_running_ = false;

  if (executor_) {
    executor_->cancel();
  }
  
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

}  // namespace robot_hw_interface

PLUGINLIB_EXPORT_CLASS(
  robot_hw_interface::Ak60RobotHwInterface, hardware_interface::SystemInterface)
  