# Tóm Tắt Project: AK60 Robot – ABU Robocon 2026

## 1. Tổng Quan

Robot tự hành thi đấu **ABU Robocon 2026 – "Kung Fu Quest"** chạy trên **ROS 2**, triển khai trên **Jetson** (Linux 5.15-tegra). Robot có kiến trúc **Rocker-Bogie 6 bánh**, cánh tay gắp nhiều khớp, và hệ thống nhận thức YOLO + RealSense D400. Toàn bộ luồng nhiệm vụ được điều phối bởi một **Behavior Tree** (py_trees + py_trees_ros).

---

## 2. Tech Stack

| Lớp | Công nghệ |
|-----|-----------|
| Middleware | ROS 2 (ament_cmake, rclpy, rclcpp) |
| Ngôn ngữ | Python 3 (logic BT, action servers) · C++ (driver điều khiển) |
| Điều khiển | ros2_control – ControllerInterface |
| Tay máy | MoveIt 2 (KDL solver) |
| Navigation | Nav2 (AMCL, nav2_params) |
| SLAM | SLAM Toolbox (slam_config.yaml) |
| Localisation | robot_localization EKF (fuse Odom + IMU) |
| Camera | Intel RealSense D400 (realsense-ros, align depth) |
| LiDAR | YDLidar (ydlidar_ros2_driver) |
| Giao tiếp HW | CAN-FD (Waveshare ControlCAN-FD) · Serial (ESP32) |
| AI | YOLO (yolo26s_best — .pt / .onnx / .engine TensorRT) |
| BT | py_trees / py_trees_ros |
| Build | colcon + CMake |

---

## 3. Cấu Trúc Thư Mục

```
ros_ws/
├── src/                              ← Tất cả ROS 2 packages
│   ├── ak60_bringup/                 ← Package chủ – launch, action defs, BT
│   │   ├── action/                   ← Định nghĩa ROS Actions (.action)
│   │   ├── config/ekf.yaml           ← Tham số bộ lọc EKF
│   │   ├── launch/                   ← File launch chính
│   │   ├── maps/                     ← Bản đồ (my_map.png + .yaml)
│   │   └── scripts/                  ← Action server nodes + Behavior Tree
│   │       ├── r2_bt/                ← Toàn bộ BT logic (xem mục 5)
│   │       ├── arm_action_server.py
│   │       ├── esp32_arm_action_server.py
│   │       ├── odom_nav_server.py
│   │       ├── step_climb_server.py
│   │       ├── tracking_server.py
│   │       └── wall_alignment_server.py
│   │
│   ├── ak60_description/             ← URDF/Xacro mô tả robot
│   │   └── urdf/                     ← chassis, wheel, manipulator, ros2_control
│   │
│   ├── ak60_moveit_config/           ← Cấu hình MoveIt 2
│   │   └── config/                   ← SRDF, kinematics.yaml, controllers, joint_limits
│   │
│   ├── ak60_robot_driver_control/    ← C++ ros2_control Controller (Rocker-Bogie)
│   │   ├── include/                  ← rocker_bogie_controller.hpp
│   │   └── src/                      ← rocker_bogie_controller.cpp
│   │
│   ├── ak60_robot_driver_io/         ← Python – giao tiếp phần cứng thấp
│   │   └── ak60_robot_driver_io/     ← can_bridge, servo_driver, imu, steer_node
│   │
│   ├── my_robot_navigation/          ← Nav2 bringup (nav2_params.yaml)
│   ├── my_robot_slam/                ← SLAM Toolbox (slam_config.yaml)
│   ├── robot_hw_interface/           ← C++ ros2_control hardware interface
│   │   └── config/servo_offsets.yaml ← Offset 6 bánh steer + 2 khớp tay
│   ├── realsense-ros/                ← Third-party: Intel RealSense ROS 2 driver
│   ├── ydlidar_ros2_driver/          ← Third-party: YDLidar ROS 2 driver
│   └── scripts/                      ← Script tiện ích (keyboard, depth_reader, usb_reset)
│
├── app_manageger/                    ← Prototype BT cũ (standalone, không ROS)
│   ├── app_manager_main.py
│   ├── bt_nodes.py
│   └── bt_sequences.py
│
├── yolo26n_best.pt / .onnx / .engine ← Model AI nhận diện hộp KFS (YOLO)
├── label_class.txt                   ← Mapping class ID → tên nhãn YOLO
├── cmd.txt                           ← Các lệnh chạy thường dùng
└── doc.md                            ← Chi tiết logic mission.py (BT flow)
```

---

## 4. Các ROS Action Interfaces (`src/ak60_bringup/action/`)

| File | Mục đích | Goal chính |
|------|----------|------------|
| `ArmSequence.action` | Chạy chuỗi tư thế cánh tay (5 khớp) | pose_name, target_pos[5], target_eff[5], duration |
| `MoveArm.action` | Di chuyển cánh tay đến tọa độ cụ thể | arm1(mm), arm2/arm3(deg), gripper(deg) |
| `ClimbStep.action` | Leo 1 bậc Meihua forest | velocity |
| `FollowTarget.action` | Bám mục tiêu YOLO theo class ID | target_id, desired_distance_mm |
| `WallAlignment.action` | Căn chỉnh hướng/khoảng cách với tường (LiDAR) | window_degrees, goal_distance |

---

## 5. Behavior Tree (`src/ak60_bringup/scripts/r2_bt/`)

```
r2_bt/
├── config.py           ← Hằng số sân, bản đồ độ cao, CLASS_LABELS, FIELD_CONFIGS
├── main.py             ← Entry point: init ROS node, load BT, tick loop (0.1s)
├── behaviors/
│   ├── blackboard_ops.py   ← ROSTopicToBB, BlackboardLoggerBehavior
│   ├── navigation.py       ← GoToRelativePoseBehavior, TurnToTargetCell, MoveRelativeOdom
│   ├── climb.py            ← ClimbStepBehavior, HasElevationChangeCondition, DynamicClimb
│   ├── hardware.py         ← FollowTargetBehavior, WallAlignmentBehavior,
│   │                          ArmSequenceBTNode, MoveArmBehavior, DeltaPickSequence
│   └── grid_logic.py       ← DecideNextGridCell, IsAtExitCell, WaitForStartSignal,
│                              IncrementRealCount, MarkAsVisited, MarkAsObstacle
└── trees/
    ├── mission.py          ← Cây nhiệm vụ chính R2 (xem doc.md để biết chi tiết)
    ├── mission_test.py     ← Cây test/debug
    └── builders.py         ← build_tool_assembly_sequence, build_move_subtree,
                               build_pick_and_place_sequence
```

### Luồng Nhiệm Vụ Tổng Quát

```
BT Root (Parallel: SuccessOnSelected[Main_Mission])
├── Topics2BB  ─ subscribe ROS topics → blackboard
├── Main_Mission (Sequence)
│   ├── Phase 0: Standby
│   │   ├── ArmSequenceBTNode("Tool_Arm_Home")  ← ESP32 home_pose_{side}
│   │   ├── MoveArmBehavior("Box_Arm_Home")      ← MoveIt [250,0,0,0]
│   │   └── WaitForStartSignalBehavior           ← Chờ GUI (priority_col, target_boxes_list)
│   ├── Phase 1: Init & Tool Pickup
│   │   ├── Tool_Arm_Approach + Tien_Lay_Dung_Cu (dx=0.42, dy=lat×1.05)
│   │   ├── build_tool_assembly_sequence         ← Lấy dụng cụ qua ESP32
│   │   ├── Tien_Ra_Cua_Cell (dx=1.7, dy=-lat×2.5)
│   │   ├── Set_Target_2_1  ← BB[target_cell]=(2,1)
│   │   └── Align_Cua_Cell  ← WallAlign window=20°, dist=0.0
│   └── Phase 2: Grid Mission Loop
│       └── KeepRunningUntilSuccess
│            └── Mode_Selector
│                 ├── EXIT Goal (target_boxes_list==[] AND real_box≥2 AND at_exit_cell) → SUCCESS
│                 ├── FIND_Exit (target_boxes_list==[] AND real_box≥2) → DecideNext(EXIT) → move
│                 └── HUNT → DecideNext(EXPLORE) → build_move_subtree → FAIL(loop)
└── BB_Logger  ─ log blackboard mỗi 30 tick
```

Mỗi `build_move_subtree` thực hiện: **TurnToCell → FollowTarget(AI) → GraspLogic → Climb → MoveIntoCell → Reset_Just_Picked**

Lưu ý: WallAlign trước vision và WallAlign trước leo bậc đã được **tắt** (commented out).

---

## 6. C++ Controllers & Hardware Interface

### `ak60_robot_driver_control` – RockerBogieController
- Kế thừa `controller_interface::ControllerInterface` (ros2_control)
- Subscribe `/cmd_vel` (Twist) → tính tốc độ 6 bánh theo kinematics Rocker-Bogie
- Publish `/odom` + TF `odom → base_link`
- Có **velocity ramp** để tránh giật cơ
- Spawn sequence: `joint_state_broadcaster` → `rocker_bogie_controller` → `arm_controller`

### `robot_hw_interface` – C++ ros2_control Hardware Interface
- Kết nối low-level với servo thực tế
- `servo_offsets.yaml`: offset calibration 6 bánh steer + 2 khớp tay

### `ak60_robot_driver_io` – Python hardware nodes

| Node/File | Nhiệm vụ |
|-----------|----------|
| `can_bridge.py` | Giao tiếp CAN-FD qua Waveshare ControlCAN-FD (`.so` vendor) |
| `ak_can_protocol.py` | Định nghĩa frame protocol AK60 motor |
| `servo_driver.py` | Điều khiển servo qua Serial |
| `steer_node.py` | ROS node điều khiển lái |
| `read_imu_node.py` | Đọc IMU → publish `/imu/data` |

---

## 7. Bản Đồ Sân Thi Đấu (Meihua Forest Grid)

Sân 3×4 ô (col 1-3, row 1-4). Mỗi ô có độ cao khác nhau (200–600mm).

```
         col1   col2   col3
row 1:   400    200    400   mm
row 2:   200    400    600   mm   ← RED layout
row 3:   400    600    400   mm
row 4:   200    400    200   mm
```

Sân BLUE là bản đối xứng của RED (col1 ↔ col3).  
Tham số `field_color` (red/blue) truyền lúc launch → BT tự chọn `lat` sign và `post_ramp_yaw`.

---

## 8. Cảm Biến & Localisation Pipeline

```
IMU (/imu/data)     ┐
Wheel Odom (/odom)  ┤── EKF (robot_localization) ──► /odometry/filtered
                    ┘
LiDAR (YDLidar)    ──► SLAM Toolbox / Nav2 AMCL
RealSense D400     ──► align_depth → YOLO tracking_server → FollowTarget action
```

---

## 9. Launch Files Chính

| File | Mục đích |
|------|----------|
| `ak60_odom.launch.py` | **Launch chính** – toàn bộ hệ thống (hardware + vision + BT) |
| `action_server.launch.py` | Chỉ khởi action servers (debug) |
| `hardwear.launch.py` | Chỉ khởi hardware nodes |
| `robot_test.launch.py` | Môi trường test robot |

### Trình tự khởi động (`ak60_odom.launch.py`)

```
t=0s   RSP, IMU, CAN, Steer, EKF, RealSense, LiDAR
t=1s   Controller Manager + Action Servers
       → (auto) joint_state_broadcaster
       → (auto) rocker_bogie_controller
       → (auto) arm_controller
t=20s  App Manager BT (chờ action servers sẵn sàng)
```

---

## 10. Script Gốc Workspace (root `ros_ws/`)

Các file Python ở thư mục gốc là **công cụ test/debug độc lập**, không phải ROS package chính thức:

| File | Chức năng |
|------|-----------|
| `arm_action_client.py` | Client test cánh tay (gửi ArmSequence action) |
| `arm_diagnostic_client.py` | Kiểm tra trạng thái cánh tay |
| `tracking_client.py` | Test FollowTarget action |
| `wall_alignment_client.py` | Test WallAlignment action |
| `teleop_keyboard.py` / `teleop_omni.py` | Điều khiển tay bằng bàn phím |
| `teleop_test_arm.py` / `test_esp32_arm.py` | Test cánh tay ESP32 |
| `yolo_ros_node.py` | Node YOLO độc lập (inference, publish detection) |
| `kfs_vision_node.py` / `r2_kfs_align_node.py` | Vision pipeline KFS align |
| `kfs_align_grasp_server.py` | Server gắp hộp KFS (test riêng) |
| `robot_control_node.py` | Node điều khiển cấp cao (thử nghiệm) |
| `pt_to_onnx.py` | Chuyển đổi model PyTorch → ONNX |
| `target_action_server.py` | Server định vị mục tiêu |
| `demo_kfs_3d.py` / `test_kfs_*.py` | Script test vision 3D/2D |

---

## 11. Lệnh Hay Dùng

```bash
# Build workspace
cd ~/ros_ws && colcon build --symlink-install && source install/setup.bash

# Launch toàn bộ hệ thống
ros2 launch ak60_bringup ak60_odom.launch.py field_color:=red   # Sân đỏ
ros2 launch ak60_bringup ak60_odom.launch.py field_color:=blue  # Sân xanh

# Launch riêng action servers
ros2 launch ak60_bringup action_server.launch.py

# Debug Behavior Tree Blackboard
py-trees-blackboard-watcher --list
py-trees-blackboard-watcher --visited --activity
py-trees-blackboard-watcher odometry
```

---

---

## 13. Chi Tiết Từng File trong `src/ak60_bringup/scripts/`

---

### 13.1 `arm_action_server.py` — Cánh Tay MoveIt (ros2_control)

**Class:** `RobotArmActionServer` | **Action:** `move_arm` (MoveArm)

**Mục đích:** Điều khiển cánh tay 4 khớp thông qua ros2_control (arm_controller). Tính toán nội suy tuyến tính từ setpoint hiện tại đến target để chuyển động mượt.

**Topics:**

| Hướng | Topic | Msg Type | Mục đích |
|-------|-------|----------|----------|
| Sub | `/dynamic_joint_states` | `DynamicJointState` | Đọc vị trí thực từ encoder |
| Pub | `/arm_controller/commands` | `Float64MultiArray` | Gửi setpoint đến ros2_control |

**Pipeline nội suy:**

```
Goal (mm/deg) → clamp limits → interpolate loop (20Hz)
  ├─ arm1: step 10 mm/loop  → publish mm → m  (÷1000)
  ├─ arm2: step 3 deg/loop  → publish rad (×π/180)
  ├─ arm3: step 3 deg/loop  → publish rad
  └─ gripper: step 20 deg/loop → publish rad
```

**Giới hạn khớp:**

| Khớp | Min | Max | Đơn vị |
|------|-----|-----|--------|
| arm1 (linear) | 0 | 760 | mm |
| arm2 | -10 | 90 | deg |
| arm3 | -90 | 90 | deg |
| gripper | -20 | 90 | deg |

**Cơ chế an toàn:** Chỉ đồng bộ setpoint 1 lần từ vị trí thực tế lúc khởi động (`is_initialized`). Mọi lệnh tiếp theo tiếp nối từ setpoint của lệnh trước — tránh giật cơ.

---

### 13.2 `esp32_arm_action_server.py` — Cánh Tay ESP32 (UART Binary)

**Class:** `Esp32ArmServer` | **Action:** `arm_sequence_controller` (ArmSequence)

**Mục đích:** Điều khiển cánh tay phụ 5 khớp (J1–CAN, J2/J3–DC, J4–Arm, J5–Gripper) giao tiếp qua ESP32 qua UART `/dev/esp32_arm` ở 115200 baud.

**Giao thức UART (Binary Frame):**

```
TX Frame: [0xAA][0x55][LEN][0x01][J1_pos,J2,J3,J4,J5, J1_eff,J2,J3,J4,J5][CS]
              header       cmd  10 floats × 4 bytes = 40 bytes payload        XOR
RX Frame: [0xAA][0x55][LEN][0x02][10 floats][CS]
              header       state_feedback (same layout)
```

**Luồng xử lý:**

```
rx_task (daemon thread, 50Hz)
  ├─ Parse binary frame (state machine: IDLE→H1→H2→LEN→CMD→PAYLOAD→CS)
  ├─ Unpack 10 floats → state_pos[5] (degrees, as-is from ESP32)
  └─ Publish /joint_states (j1_can, j2_dc, j3_dc, j4_arm, j5_gripper)

tx_task (ROS timer, 20Hz)
  └─ Nếu active_control=True → send_uart_frame() (target_pos + target_eff)

keep_alive_timer (1Hz)
  └─ Nếu không có goal > 4s → enter_sleep_mode() (target_eff=0 → khóa cơ khí)
```

**3 kịch bản execute_callback:**

| Kịch bản | pose_name | Hành động |
|----------|-----------|-----------|
| `assemble_sequence` | "assemble_sequence" | Đẩy J3 max (-1.0 eff) → chờ lò xo đẩy về (+0.5°) → mở J5 gripper → 10s |
| Homing | "home" | Kéo J2 và J3 về vị trí âm (−0.3 eff), chờ < −0.5°, timeout 10s |
| Waypoint | tên pose / "custom" | Nội suy tuyến tính theo `duration` (20 steps/s), chờ đến đích (sai số < 5°) |

**PREDEFINED_POSES** (trích lọc):

```python
"approach_j3_right":  pos=[0,0,0,40,90], eff=[0,0,-1.0,0,0]   # hướng J3 xuống
"grasp_right":        pos=[0,0,0,40,30], eff=[0,0, 0.0,0,0]   # đóng gripper
"move_back_J4_right": pos=[0,0,0,105,30], eff=[0,0, 0.0,0,0]  # rút J4 lên
```

---

### 13.3 `tracking_server.py` — YOLO Vision Tracking

**Class:** `IntegratedTrackingServer` | **Action:** `follow_target` (FollowTarget)

**Mục đích:** Nhận diện và bám theo hộp KFS bằng YOLO TensorRT + RealSense D400. Điều khiển robot đến đúng khoảng cách và căn giữa mục tiêu.

**Khởi tạo:**
- Nạp model `yolo26n_best.engine` (TensorRT)
- Warm-up 1 frame ảnh đen 480×640 để ép cấp phát GPU trước khi nhận lệnh

**Subscriptions:**

| Topic | Msg Type | Xử lý |
|-------|----------|-------|
| `/camera/color/image_raw` | `Image` | Callback nhanh – chỉ copy vào buffer, không xử lý |
| `/camera/aligned_depth_to_color/image_raw` | `Image` | Copy depth buffer (16UC1) |

**Pipeline 2 giai đoạn (20Hz):**

```
PHASE_ALIGN_X  (AI bật)
  │
  ├─ YOLO inference (frame 640×480)
  ├─ Lọc detection trong ROI: error_x ≤ 200px AND depth < 1200mm
  ├─ Phân loại: real / fake / empty
  │    ├─ real:  target_label khớp detection → Kalman filter (x, depth) → strafe (cmd_vel.y)
  │    ├─ fake:  class khác → đếm lost_frames
  │    └─ empty: không có gì → đếm lost_frames
  ├─ Nếu lost_frames ≥ 10 → Abort với message = "fake"/"empty"
  └─ Nếu aligned_frames ≥ 10 (error_x ≤ 10px) → chuyển PHASE_APPROACH
  
PHASE_APPROACH  (AI tắt, chỉ đọc Depth)
  │
  ├─ Đọc patch 10×10 quanh (img_center_x, last_target_y) từ depth image
  ├─ Lấy median của valid pixels → distance_mm
  ├─ P-controller: v_x = kp_linear_mm × (distance_mm − desired_distance_mm)
  │   kp=0.003, max 0.2 m/s, min-speed clamp 0.05 m/s
  └─ Nếu success_frames ≥ 10 (error_dist ≤ 10mm) → Succeed "real"
```

**Kalman Filter (4 trạng thái: x, depth, vx, vdepth):**
- Process noise: 0.05 (mượt nhưng không lag)
- Measurement noise: 5.0 (tin sensor vừa phải)

**Điều khiển:**

| Lệnh | Giai đoạn | Công thức |
|------|-----------|-----------|
| `cmd_vel.y` (strafe) | ALIGN_X | `kp_lateral × error_x`, max 0.2 m/s |
| `cmd_vel.x` (forward) | APPROACH | `kp_linear_mm × error_dist`, max 0.2 m/s |

---

### 13.4 `wall_alignment_server.py` — Căn Tường bằng LiDAR

**Class:** `WallAlignmentServer` | **Action:** `align_and_approach_wall` (WallAlignment)

**Mục đích:** 3 giai đoạn căn chỉnh với tường: xoay vuông góc → chốt góc yaw → tiến áp sát.

**Subscriptions:**

| Topic | Dùng cho |
|-------|----------|
| `/scan` (LaserScan) | Đo góc lệch và khoảng cách tường |
| `/odometry/filtered` | Đọc yaw hiện tại để snap |

**`get_wall_state(scan, window_rad)`:**

```
1. Lọc các tia LiDAR trong góc ±window_rad/2 phía trước
   (offset 90° vì LiDAR lắp xoay so với base_link)
2. Chiếu thành điểm (x, y) trong hệ robot
3. Hồi quy tuyến tính (linear regression) → độ dốc m → deviation = atan(m)
4. Intercept c = khoảng cách đến tường
5. Cần ≥ 5 điểm hợp lệ để tính
```

**Pipeline 3 giai đoạn (20Hz):**

```
PHASE 1: ALIGNING
  ├─ Lặp gọi get_wall_state()
  ├─ P-controller: angular.z = -kp_angular × deviation_rad
  │   kp=2.5, clamp [-0.1, +0.1] rad/s
  └─ Dừng khi |deviation| < 1.5° → sang Phase 2

PHASE 2: SNAPPING_YAW
  ├─ Làm tròn yaw hiện tại về bội số gần nhất của 90°
  │   target = round(yaw / π/2) × π/2
  └─ Publish /set_pose (PoseWithCovarianceStamped) → EKF cập nhật yaw chính xác
  
PHASE 3: APPROACHING  (bỏ qua nếu goal_distance = 0)
  ├─ P-controller: linear.x = kp_linear × (current_dist − goal_dist)
  │   kp=1.5, clamp [-0.2, +0.2] m/s
  └─ Dừng khi |error_dist| < 0.01m
```

---

### 13.5 `odom_nav_server.py` — Di Chuyển Holonomic (Odometry)

**Class:** `OdomNavSwerveServer` | **Action:** `navigate_to_pose` (NavigateToPose)

**Mục đích:** Navigation thuần odometry cho robot omnidirectional (không dùng Nav2 stack). Thực hiện tịnh tiến + xoay tại chỗ.

**Pipeline 2 giai đoạn (20Hz):**

```
PHASE 1: HOLONOMIC MOVE
  ├─ dx = target_x − current_x  ,  dy = target_y − current_y
  ├─ vel_global = kp_xy × (dx, dy)  [kp=1.5]
  ├─ Chuyển sang frame robot: xoay theo -current_yaw
  ├─ Kẹp tốc độ theo vector magnitude (không kẹp từng trục riêng lẻ)
  │     if |vel_vector| > max_vel_xy: scale down cả hai trục
  ├─ heading_error = normalize(initial_yaw − current_yaw) → angular.z = kp×error
  └─ Dừng khi distance < 0.05m

PHASE 2: ROTATE IN PLACE
  ├─ yaw_error = normalize(target_yaw_final − current_yaw)
  ├─ angular.z = kp_angular × yaw_error  [kp=1.0, max=0.6 rad/s]
  └─ Dừng khi |yaw_error| < 0.05 rad (~2.9°)
```

**Đặc điểm Holonomic:** Giữ nguyên `initial_yaw` trong Phase 1 (robot tịnh tiến mà không xoay), chỉ xoay trong Phase 2. Phù hợp với Rocker-Bogie swerve drive.

---

### 13.6 `step_climb_server.py` — Leo Bậc Địa Hình

**Class:** `StepClimbServer` | **Action:** `climb_step` (ClimbStep)

**Mục đích:** Tự động leo 1 bậc Meihua Forest dựa trên cảm biến pitch từ odometry IMU-fused.

**Pipeline:**

```
Nhận goal.velocity → tiến thẳng với v=velocity (Twist.linear.x)

Stage Machine (20Hz):
  stage 0: Chờ |pitch| > 5° → robot bắt đầu leo bậc
  stage 1: Chờ |pitch| < 1.5° → robot đã vượt qua đỉnh bậc
           → Succeed

Timeout: 15s → Abort nếu không leo xong
Cancel: Chấp nhận, dừng robot ngay
```

**Đọc pitch từ quaternion odometry:**
```
sinp = 2(w·y − z·x)
pitch = asin(sinp)   [giới hạn ±90°]
```

---

### 13.7 `app_manager_bt.py` — Entry Point Behavior Tree

File wrapper thuần túy (11 dòng), thêm thư mục scripts vào sys.path rồi gọi `r2_bt.main.main()`. Đây là executable được ROS 2 launch gọi.

---

### 13.8 `r2_bt/` — Behavior Tree Core

#### `config.py`

- `FIELD_CONFIGS` dict: cấu hình sân `red`/`blue` — bản đồ độ cao 3×4 ô, hệ số `lat` (dấu dy), `post_ramp_yaw`, `tool_arm_side`, `ai_target_id` (class4=red, class5=blue)
- `CLASS_LABELS` / `CLASS_PICK_INTENT`: mapping class ID YOLO → tên → hành động (pick/avoid/skip)
- **Tuning params tập trung** (chỉnh ở đây, không cần vào từng file):

| Dict | Tham số quan trọng |
|------|--------------------|
| `CLIMB_PARAMS` | up_velocity=0.3, down_velocity=0.1, pitch_threshold=5°, flat_threshold=1.5° |
| `WALL_ALIGN_PARAMS` | kp_angular=2.5, tolerance_deg=1.5°, lidar_yaw_offset=90° |
| `NAV_PARAMS` | flat_dist=0.1m, climb_dist=0.1m, follow_dist_mm=320mm |
| `TRACKING_PARAMS` | kp_linear_mm=0.003, kp_lateral=0.0005 |

- `PICK_PRESETS`: pose dict cho 4 kịch bản gắp (low_1, low_2, high_1, high_2) — mỗi preset gồm {home, pre_grasp, close, lift, retract, [hold]}
- `PICK_STEPS_COUNT_1/2`: chuỗi bước arm cho hộp thứ nhất (không hold) và hộp thứ hai (có bước hold@500mm để tránh hộp đã đặt)

#### `main.py`

Entry point ROS 2: khởi tạo Node, declare parameters từ `FIELD_CONFIGS`, import và build cây BT, chạy tick loop 10Hz trong thread riêng, spin ROS 2 trên main thread.

#### `behaviors/blackboard_ops.py`

- `ROSTopicToBB`: subscribe 1 topic, ghi raw string vào blackboard (dùng để nhận tín hiệu GUI)
- `BlackboardLoggerBehavior`: log toàn bộ blackboard để debug

#### `behaviors/navigation.py`

| Class | Gọi Action | Mục đích |
|-------|-----------|----------|
| `GoToRelativePoseBehavior` | `navigate_to_pose` | Di chuyển dx,dy trong frame robot (tính absolute target từ odom) |
| `TurnToTargetCellBehavior` | `navigate_to_pose` | Đọc `robot_pos`, `target_cell`, `just_picked` → nếu `just_picked=="yes"` và di chuyển ngang, logic backup 0.2m tồn tại (phase=0) nhưng hiện **tắt** — update() luôn gọi `_update_turn()`. Góc xoay cố định 0° cho ô (2,1),(1,4),(3,4). |
| `MoveRelativeOdomBehavior` | `navigate_to_pose` | Di chuyển thẳng `climb_dist`/`flat_dist` theo hướng `curr_yaw` (không dùng grid geometry). Cập nhật `robot_pos=target_cell` và `visit_path` khi thành công. |

Tất cả dùng pattern **non-blocking polling**: gửi goal trong `initialise()`, kiểm tra future trong `update()` (RUNNING/SUCCESS/FAILURE).

#### `behaviors/climb.py`

| Class | Mục đích |
|-------|----------|
| `ClimbStepBehavior` | Wrapper gọi action `climb_step` |
| `HasElevationChangeCondition` | Đọc `robot_pos`→`target_cell` từ blackboard, tra bản đồ độ cao → SUCCESS nếu chênh lệch > ngưỡng |
| `DynamicClimbStepBehavior` | Chọn velocity leo (lên/xuống) dựa trên delta cao độ, gọi `ClimbStepBehavior` |

#### `behaviors/hardware.py`

| Class | Gọi Action / Topic | Mục đích |
|-------|-------------------|----------|
| `WallAlignmentBehavior` | `align_and_approach_wall` | Wrapper với **timeout** tích hợp (mặc định 5s → FAILURE để không block) |
| `MoveArmBehavior` | `move_arm` | Di chuyển cánh tay ros2_control |
| `ArmSequenceBTNode` | `arm_sequence_controller` | Chạy chuỗi tư thế ESP32 arm |
| `FollowTargetBehavior` | `follow_target` | Bám hộp YOLO, trả về FAILURE nếu "fake"/"empty" |
| `DeltaPickSequenceBehavior` | Combo gắp | Gọi chuỗi: pre_grasp → grasp → lift → retract (dựa trên preset cao độ) |

#### `behaviors/grid_logic.py`

| Class | Mục đích |
|-------|----------|
| `WaitForStartSignalBehavior` | Parse JSON từ `gui_start_raw` → ghi `priority_col`, `target_boxes_list` (route queue) |
| `DecideNextGridCellAction` | Thuật toán chọn ô tiếp theo — 2 mode: **EXPLORE**: nếu `target_boxes_list` không rỗng thì bước gần nhất về phía `t_boxes[0]` (route-following); nếu rỗng thì ưu tiên `priority_col`, dùng manhattan distance. **EXIT**: chọn ô gần exit cell nhất. Tự pop front của queue khi ô đó đã visited/obstacle. |
| `IsAtExitCellCondition` | Kiểm tra `robot_pos` ∈ EXIT_CELLS = [(1,4), (3,4)] |
| `MarkAsVisitedAction` | Cập nhật `grid_status[cell]['visited'] = True` |
| `MarkAsObstacleAction` | Cập nhật `grid_status[cell]['state'] = 'obstacle'` |
| `IncrementRealCountAction` | `real_box_count += 1` |
| `KeepRunningUntilSuccess` | Decorator: trả RUNNING cho đến khi child SUCCESS HOẶC robot đang ở exit cell |

#### `trees/builders.py`

Các hàm factory xây dựng cây BT con:

| Hàm | Trả về |
|-----|--------|
| `build_tool_assembly_sequence(side)` | Sequence: Approach_J3 → Grasp → Move_Back_J4 → Move_Back_J1 → Assemble (20s) |
| `build_move_subtree(ros_node, ai_target_id)` | Sequence: Turn → FollowTarget(FailureIsSuccess) → Box_Logic → Climb → Move → Reset_Just_Picked. WallAlign trước và sau TẮTM. |
| `build_pick_and_place_sequence(mode)` | Selector: Count2 branch (real_box≥1 → ArmSeq với hold@500) / Count1 branch. Dùng `PICK_PRESETS[mode_1/2]` và `PICK_STEPS_COUNT_1/2`. |
| `build_pick_and_place_sequence_dynamic(ros_node)` | Selector: `IsClimbingUpCondition` → mode='high' / mode='low'. Tự chọn khi runtime. |
| `_build_arm_seq(ros_node, preset_key, steps)` | Sequence: mỗi bước = `MoveArmBehavior` + `RosWaitBehavior(wait_sec)`. |

#### `trees/mission.py` (xem chi tiết ở `doc.md`)

Cây nhiệm vụ chính. Import từ `builders.py` và `behaviors/`, lắp ráp thành cây hoàn chỉnh theo 3 Phase đã mô tả ở mục 5.

---

## 14. Pipeline Tổng Thể — Từ GUI đến Hành Động

```
GUI Panel (tablet/PC)
    │ publish /gui_start_signal (JSON: {command:"START", priority_col:2, target_boxes:[...]})
    ▼
ROSTopicToBB → blackboard["gui_start_raw"]
    │
    ▼
WaitForStartSignalBehavior
    │ parse JSON → BB["priority_col"], BB["target_boxes_list"] (route queue)
    ▼
DecideNextGridCellAction (mode=EXPLORE)
    │ Nếu target_boxes_list không rỗng → bước về phía t_boxes[0] (route-following)
    │ Nếu rỗng → ưu tiên priority_col, chọn ô chưa visited gần nhất
    ▼
TurnToTargetCellBehavior ──► odom_nav_server ──► /cmd_vel → RockerBogieController
    │ (navigate_to_pose: xoay tại chỗ về hướng ô đích)
    ▼
FollowTargetBehavior (FailureIsSuccess) ─► tracking_server
    │ YOLO (TRT, ai_target_id per field color) nhận diện hộp
    │ Kalman filter → strafe ngang (cmd_vel.y)
    │ Depth sensor → tiến vào 320mm (cmd_vel.x)
    │ Trả về: "REAL" / "FAKE" / "EMPTY" → BB[latest_box_detection]
    ▼
Box_Logic (Selector):
  ├─ [REAL]  build_pick_and_place_sequence_dynamic
  │              ├─ IsClimbingUpCondition → mode=high/low
  │              ├─ Count ≥ 1 → ArmSeq_*_2 (lift@520 + hold@500)
  │              └─ Count = 0 → ArmSeq_*_1 (lift@250)
  │              → Box_Arm_Home [325,0,0,0]
  │              → Set_Just_Picked="yes"
  │              → IncrementRealCountAction (real_box_count++)
  ├─ [FAKE]  MarkAsObstacleAction
  └─ [EMPTY] MarkAsVisitedAction
    ▼
DynamicClimbStepBehavior ───► step_climb_server
    │ Tiến thẳng v = ±0.3/0.1 m/s (lên/xuống)
    │ Đợi pitch > 5° rồi < 1.5°
    ▼
MoveRelativeOdomBehavior ───► odom_nav_server (climb_dist=0.1m hoặc flat_dist=0.1m)
    │ Cập nhật BB[robot_pos] = target_cell
    │ → Reset_Just_Picked="no"
    ▼ (vòng lặp: DecideNextGridCell → ...)
    
  Khi target_boxes_list==[] AND real_box_count ≥ 2 → chuyển FIND_EXIT → về (1,4) hoặc (3,4)
```

## 12. Sơ Đồ Kiến Trúc Tổng Thể

```
┌─────────────────────────────────────────────────────────────┐
│                  BEHAVIOR TREE (py_trees)                    │
│   Standby → Init → GridMissionLoop (Hunt/Pick/Climb/Move)   │
└───────────────────┬─────────────────────────────────────────┘
                    │  ROS 2 Action calls
     ┌──────────────┼───────────────────┐
     ▼              ▼                   ▼
odom_nav_server  tracking_server   arm_action_server
step_climb_server wall_align_server esp32_arm_server
     │              │                   │
     ▼              ▼                   ▼
 RockerBogie    RealSense+YOLO       CAN-FD / Serial
 Controller     (YOLOv8n TRT)        (AK60 motors + ESP32)
     │
     ▼
 EKF Localisation
 (Odom + IMU → /odometry/filtered)
     │
 YDLidar → SLAM / Nav2
```
