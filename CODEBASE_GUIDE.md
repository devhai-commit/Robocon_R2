# Hướng dẫn Codebase — Robot AK60 (Robocon)

## Tổng quan

Đây là **ROS 2 workspace** cho robot thi đấu **Robocon**, tên package là **AK60**. Robot có:
- **Di chuyển**: Khung Rocker-Bogie 6 bánh (omnidirectional)
- **Tay máy**: 3 khớp + kẹp, điều khiển qua CAN bus
- **Cảm biến**: LiDAR (YDLidar), Camera Depth (RealSense D4xx)
- **AI**: YOLO để nhận dạng và bám đuổi vật thể

---

## Cấu trúc thư mục gốc `/home/robocon/ros_ws/`

### Scripts ở thư mục gốc

| File | Chức năng |
|------|-----------|
| `robot_control_node.py` | Node điều khiển robot đơn giản: subscribe tọa độ target từ YOLO → tính P-control → publish `/cmd_vel` để robot bám theo vật thể |
| `yolo_server.py` | Node YOLO hybrid: mở camera, chạy YOLOv8, publish tọa độ tâm vật thể lên `/target_coordinates`. Có service `/toggle_yolo` để bật/tắt từ xa giúp tiết kiệm GPU |
| `yolo_ros_node.py` | Phiên bản khác của YOLO node (bản cũ hơn hoặc thử nghiệm) |
| `arm_action_client.py` | Client test tay máy: gửi chuỗi lệnh pick-and-place đến Action Server `move_arm`, có preset vị trí sẵn |
| `arm_diagnostic_client.py` | Client chẩn đoán tay máy |
| `tracking_client.py` | Client test AI tracking: gửi lệnh bám vật thể đến Action Server `/follow_target`, truyền ID và khoảng cách mong muốn |
| `wall_alignment_client.py` | Client test căn chỉnh tường: gửi lệnh quét LiDAR và áp sát tường đến Action Server `align_and_approach_wall` |
| `target_action_server.py` | Phiên bản đơn giản của tracking server: subscribe `/target_coordinates` từ YOLO → điều khiển robot bằng P-control cho đến khi vật vào tâm ảnh |
| `teleop_keyboard.py` | Teleop đầy đủ: điều khiển bánh xe + cánh tay + kẹp bằng bàn phím (w/s/a/d/q/e cho bánh, u/j/i/k/o/l cho tay) |
| `teleop_omni.py` | Teleop đơn giản: chỉ điều khiển bánh xe omni (tiến/lùi/ngang/xoay) |
| `teleop_test_arm.py` | Teleop cánh tay: đồng bộ vị trí thực từ phần cứng rồi cho phép điều khiển tăng/giảm dần, có clamp giới hạn an toàn |
| `test_esp32_arm.py` | Test tay máy qua ESP32 |
| `app_manager_bt_master.py` | Entrypoint master của Behavior Tree (phiên bản trong thư mục gốc) |
| `pt_to_onnx.py` | Chuyển đổi model: từ `.pt` (PyTorch) → `.onnx` → `.engine` (TensorRT cho Jetson) |
| `yolo26n_best.pt` | Model YOLO gốc (PyTorch) |
| `yolo26n_best.onnx` | Model YOLO đã export sang ONNX |
| `yolo26n_best.engine` | Model YOLO đã compile sang TensorRT (chạy nhanh nhất trên Jetson) |

---

## Folder `app_manageger/` — Bộ não điều phối robot (Behavior Tree)

Đây là **Behavior Tree (BT)** điều phối toàn bộ nhiệm vụ thi đấu của robot.

| File | Chức năng |
|------|-----------|
| `app_manager_main.py` | **Entry point chính**: tạo cây BT, khởi tạo ROS executor đa luồng, khởi động vòng tick cây. Định nghĩa blackboard (bộ nhớ chia sẻ: vị trí robot, trạng thái ô lưới, số hộp thật) |
| `bt_nodes.py` | **Thư viện các Node hành vi** (~30 class con của `py_trees.Behaviour`): `GoToRelativePoseBehavior`, `MoveArmBehavior`, `WallAlignmentBehavior`, `FollowTargetBehavior`, `DynamicClimbStepBehavior`, `TurnToTargetCellBehavior`, v.v. Mỗi node gọi một Action Server tương ứng |
| `bt_sequences.py` | **Thư viện các chuỗi hành vi**: ghép các node thành sequence như `assembly_sequence_right()` (lấy dụng cụ phải), `build_move_subtree()` (tiến vào ô lưới + xử lý hộp), `build_pick_and_place_sequence()` |

**Luồng nhiệm vụ BT**:
```
Lấy dụng cụ
  → Tiến ra cửa lưới
  → Lặp: quét ô lưới → nhận dạng hộp (REAL/FAKE/EMPTY)
       → [REAL]  gắp hộp + tính điểm
       → [FAKE]  đánh dấu chướng ngại vật
       → [EMPTY] đánh dấu đã thăm
  → Leo bậc lên ô tiếp theo
  → Standby
```

---

## Folder `src/` — ROS 2 Packages

### `src/ak60_bringup/` — Package trung tâm

Package quan trọng nhất: chứa định nghĩa Action, launch file, và các Action Server.

#### `action/` — Giao diện Action (interface)

| File | Chức năng |
|------|-----------|
| `ArmSequence.action` | Chạy chuỗi động tác tay máy theo tên preset (home, grasp, assemble...) |
| `ClimbStep.action` | Leo bậc thang (truyền độ cao bậc) |
| `FollowTarget.action` | Bám đuổi vật thể bằng AI (truyền ID vật và khoảng cách mong muốn) |
| `MoveArm.action` | Di chuyển tay máy đến tọa độ cụ thể [mm, độ, độ, độ] |
| `WallAlignment.action` | Căn chỉnh góc và áp sát tường bằng LiDAR |

#### `scripts/` — Các Action Server chạy thực tế

| File | Chức năng |
|------|-----------|
| `arm_action_server.py` | Server `move_arm`: nhận lệnh vị trí [mm/độ] → nội suy tuyến tính → publish `/arm_controller/commands`. Đồng bộ vị trí từ phần cứng lúc khởi động để tránh giật |
| `esp32_arm_action_server.py` | Phiên bản arm server giao tiếp qua ESP32 thay vì CAN |
| `tracking_server.py` | Server `follow_target`: dùng YOLOv8 TensorRT + RealSense depth → Kalman Filter → PID điều khiển robot bám vật. Phân biệt REAL/FAKE/EMPTY box |
| `wall_alignment_server.py` | Server `align_and_approach_wall`: đọc LiDAR → xác định góc lệch tường → P-control xoay robot thẳng hàng → áp sát đến khoảng cách định sẵn |
| `step_climb_server.py` | Server `climb_step`: điều phối trình tự leo bậc thang (hạ chân, tiến, nâng chân) |
| `odom_nav_server.py` | Server di chuyển theo odometry tương đối (dx, dy, yaw) |
| `app_manager_bt.py` | Phiên bản BT chạy trong môi trường ROS package |

#### `launch/` — File khởi động

| File | Chức năng |
|------|-----------|
| `ak60.launch.py` | Launch toàn bộ hệ thống robot (main launch file) |
| `hardwear.launch.py` | Launch phần cứng: CAN bridge, IMU, servo driver |
| `action_server.launch.py` | Launch tất cả Action Server |
| `ak60_odom.launch.py` | Launch odometry + robot_localization EKF |
| `robot_test.launch.py` | Launch môi trường test |

#### `config/`

| File | Chức năng |
|------|-----------|
| `ekf.yaml` | Cấu hình Extended Kalman Filter: fuse IMU + wheel odometry → `/odometry/filtered` |

#### `maps/`

Bản đồ sân thi đấu đã vẽ sẵn (`my_map.yaml` + `my_map.png`) dùng cho Nav2 navigation.

---

### `src/ak60_description/` — Mô tả hình học robot (URDF)

```
urdf/
  robot.urdf.xacro     : File URDF chính (ghép tất cả xacro lại)
  chassis.xacro        : Mô tả thân xe Rocker-Bogie 6 bánh
  manipulator.xacro    : Mô tả tay máy 3 khớp + kẹp
  wheel.xacro          : Mô tả một bánh xe (dùng lại 6 lần)
  properties.xacro     : Hằng số (kích thước, khối lượng, màu sắc)
  ros2_control.xacro   : Khai báo hardware interface cho ros2_control
  older_robot.urdf.xacro : Phiên bản URDF cũ

meshes/                : File 3D mesh (.stl/.dae) để hiển thị trong RViz
rviz/robot_view.rviz   : Cấu hình giao diện RViz (camera, display, topic)
```

---

### `src/ak60_moveit_config/` — Cấu hình MoveIt2

Được tạo tự động bằng MoveIt Setup Assistant, dùng để lập kế hoạch chuyển động tay máy.

```
config/
  ak60_robot.srdf          : Nhóm khớp (manipulator group), trạng thái home, collisions
  kinematics.yaml          : Bộ giải IK (KDL hoặc TRAC-IK)
  joint_limits.yaml        : Giới hạn vị trí/vận tốc/gia tốc từng khớp
  moveit_controllers.yaml  : Kết nối MoveIt với ros2_controllers
  ros2_controllers.yaml    : Khai báo arm_controller, gripper_controller
  pilz_cartesian_limits.yaml : Giới hạn chuyển động Cartesian (Pilz planner)

launch/
  demo.launch.py           : Demo đầy đủ với RViz + MoveIt
  move_group.launch.py     : Khởi động MoveIt move_group node
  moveit_rviz.launch.py    : Chỉ RViz với MoveIt plugin
  rsp.launch.py            : Robot State Publisher
  spawn_controllers.launch.py : Spawn ros2_controllers
```

---

### `src/ak60_robot_driver_control/` — Controller bánh xe Rocker-Bogie (C++)

```
src/rocker_bogie_controller.cpp : Plugin ros2_control:
                                   - Nhận /cmd_vel (vx, vy, wz)
                                   - Tính toán vận tốc từng bánh (6 bánh độc lập)
                                   - Xử lý động học Rocker-Bogie
                                   - Ghi lệnh xuống hardware interface

config/controllers.yaml          : Khai báo controller type, tham số PID bánh xe
plugin.xml                       : Khai báo plugin cho pluginlib
```

---

### `src/ak60_robot_driver_io/` — Giao tiếp phần cứng CAN bus (Python)

Lớp thấp nhất, giao tiếp trực tiếp với phần cứng qua CAN bus.

```
ak60_robot_driver_io/
  can_bridge.py            : Node bridge CAN — subscribe /canfd/tx → gửi lên bus phần cứng,
                             nhận dữ liệu từ bus → publish /canfd/rx
                             Dùng Waveshare CANFD USB adapter

  ak_can_protocol.py       : Thư viện encode/decode giao thức MIT mode của động cơ AK60-6:
                             - pack_mit(): tạo gói tin 8-byte [kp, kd, pos, vel, torque]
                             - parse_feedback(): đọc phản hồi [pos, speed, current, temp, error]

  servo_driver.py          : Driver điều khiển servo (tay máy): nhận lệnh vị trí →
                             chuyển đổi → gửi lệnh CAN đến động cơ servo

  steer_node.py            : Node điều khiển hướng bánh lái (steering joints)
  read_imu_node.py         : Node đọc dữ liệu IMU từ CAN bus
  debug_arm_can.py         : Tool debug: in raw CAN frame của tay máy ra terminal
  waveshare_controlcanfd.py: Wrapper Python cho thư viện C (.so) của Waveshare

vendor/
  libcontrolcanfd.so       : Thư viện binary Waveshare (giao tiếp phần cứng CANFD USB)
```

---

### `src/my_robot_navigation/` — Navigation Stack (Nav2)

```
config/nav2_params.yaml       : Tham số đầy đủ Nav2:
                                - BT Navigator (behavior tree navigator)
                                - Planner Server (NavFn, Smac)
                                - Controller Server (DWB, RPP)
                                - Costmap 2D (global + local)
                                - Recovery behaviors

launch/nav_bringup.launch.py  : Khởi động Nav2 với bản đồ tĩnh (localize + navigate)
maps/                         : Bản đồ sân thi đấu
```

---

### `src/my_robot_slam/` — SLAM (vẽ bản đồ tự động)

```
config/slam_config.yaml       : Cấu hình SLAM Toolbox (online async mode,
                                resolution, loop closure threshold)

launch/my_slam.launch.py      : Khởi động SLAM Toolbox + LiDAR để vẽ bản đồ
                                (chạy trước khi dùng Navigation)
```

---

### `src/realsense-ros/` — Driver Intel RealSense (thư viện ngoài)

Driver chính thức của Intel cho camera RealSense D4xx (D435, D455...).

**Topics cung cấp**:
- `/camera/color/image_raw` — ảnh RGB
- `/camera/depth/image_rect_raw` — ảnh depth (mm)
- `/camera/depth/color/points` — pointcloud 3D

---

### `src/ydlidar_ros2_driver/` — Driver YDLidar (thư viện ngoài)

Driver chính thức cho LiDAR YDLidar.

**Topic cung cấp**: `/scan` (sensor_msgs/LaserScan)

```
params/
  ydlidar.yaml   : Cấu hình đang dùng (baud rate, angle range, frequency)
  G1/G2/X4.yaml  : Cấu hình cho các model LiDAR khác
```

---

### `src/robot_hw_interface/` — Hardware Interface ros2_control (C++)

```
src/hw_robot_interface.cpp    : Plugin ros2_control::SystemInterface:
                                - on_init(): đọc tham số từ URDF
                                - read(): đọc joint state từ CAN bus feedback
                                - write(): ghi lệnh joint xuống CAN bus
                                Cầu nối giữa ros2_controllers và phần cứng thực

config/servo_offsets.yaml     : Offset bù góc zero cho từng servo (hiệu chỉnh cơ học)
urdf/test_robot.urdf          : URDF test đơn giản để test hardware interface
```

---

### `src/scripts/` — Script tiện ích

| File | Chức năng |
|------|-----------|
| `ak60_keyboard.py` | Teleop keyboard thêm |
| `depth_reader.py` | Đọc và hiển thị dữ liệu depth từ RealSense (debug tool) |
| `usb_auto_reset.py` | Tự động reset USB port khi thiết bị (LiDAR/Camera) mất kết nối |

---

## Kiến trúc tổng thể — Luồng dữ liệu

```
┌─────────────────────────────────────────────────────────────┐
│                    TẦNG ĐIỀU PHỐI                           │
│   app_manager_main.py                                       │
│   └── Behavior Tree (py_trees)                              │
│         ├── bt_nodes.py   (Action Clients)                  │
│         └── bt_sequences.py (Chuỗi nhiệm vụ)               │
└──────────────────────────┬──────────────────────────────────┘
                           │ ROS 2 Actions
┌──────────────────────────▼──────────────────────────────────┐
│                    TẦNG LOGIC / SERVER                      │
│   arm_action_server.py     — Điều khiển tay máy            │
│   tracking_server.py       — AI bám vật (YOLO + Depth)     │
│   wall_alignment_server.py — Căn tường (LiDAR)             │
│   step_climb_server.py     — Leo bậc                       │
│   odom_nav_server.py       — Di chuyển theo odom           │
└────────────┬──────────────────────┬─────────────────────────┘
             │ /cmd_vel             │ /arm_controller/commands
┌────────────▼──────────┐  ┌───────▼─────────────────────────┐
│  rocker_bogie_        │  │  arm_controller (ros2_control)  │
│  controller (C++)     │  │  gripper_controller             │
└────────────┬──────────┘  └───────┬─────────────────────────┘
             │                     │
┌────────────▼─────────────────────▼─────────────────────────┐
│                    TẦNG PHẦN CỨNG                           │
│   robot_hw_interface.cpp  (ros2_control SystemInterface)    │
│   └── can_bridge.py       (ROS ↔ CAN bus)                  │
│         └── waveshare_controlcanfd.py                       │
│               └── libcontrolcanfd.so                        │
│                     └── [USB CANFD Adapter]                 │
│                           └── [Động cơ AK60 + Servo]       │
└─────────────────────────────────────────────────────────────┘

SENSORS:
  [YDLidar]   ──/scan──────────► wall_alignment_server
  [RealSense] ──/depth──────────► tracking_server
  [IMU]       ──/imu/data───────► EKF → /odometry/filtered
```

---

## Topics quan trọng

| Topic | Type | Chiều | Ý nghĩa |
|-------|------|-------|---------|
| `/cmd_vel` | Twist | → Robot | Lệnh vận tốc tổng hợp |
| `/scan` | LaserScan | ← LiDAR | Dữ liệu quét laser |
| `/odometry/filtered` | Odometry | ← EKF | Vị trí robot đã lọc |
| `/arm_controller/commands` | Float64MultiArray | → Controller | Lệnh vị trí tay máy [m, rad, rad, rad] |
| `/dynamic_joint_states` | DynamicJointState | ← Controller | Trạng thái thực tế các khớp |
| `/canfd/tx` | can_msgs/Frame | → CAN bus | Gói tin gửi xuống động cơ |
| `/canfd/rx` | can_msgs/Frame | ← CAN bus | Phản hồi từ động cơ |
| `/target_coordinates` | Point | ← YOLO | Tọa độ tâm vật thể trên ảnh |

---

## Lệnh hay dùng

```bash
# Build workspace
cd ~/ros_ws && colcon build --symlink-install

# Source
source install/setup.bash

# Launch toàn bộ robot
ros2 launch ak60_bringup ak60.launch.py

# Launch phần cứng (CAN, IMU)
ros2 launch ak60_bringup hardwear.launch.py

# Teleop bánh xe
python3 teleop_omni.py

# Teleop bánh + tay máy
python3 teleop_keyboard.py

# Teleop chỉ tay máy
python3 teleop_test_arm.py

# SLAM (vẽ bản đồ)
ros2 launch my_robot_slam my_slam.launch.py

# Navigation (đi theo bản đồ)
ros2 launch my_robot_navigation nav_bringup.launch.py

# Chạy Behavior Tree (nhiệm vụ thi đấu)
cd app_manageger && python3 app_manager_main.py

# Test AI tracking
python3 tracking_client.py --id 6 --dist 360

# Test căn tường
python3 wall_alignment_client.py

# Test tay máy
python3 arm_action_client.py
```
