#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray

import sys
import select
import termios
import tty
import threading
import math

msg = """
=== ĐIỀU KHIỂN CÁNH TAY BẰNG BÀN PHÍM ===
Đang chờ đồng bộ vị trí thực tế từ mạch...

Phím điều khiển (Tăng / Giảm):
  - Arm 1 (Base - mm)   :  [q] / [a]
  - Arm 2 (Shoulder - °):  [w] / [s]
  - Arm 3 (Elbow - °)   :  [e] / [d]
  - Gripper (Kẹp - °)   :  [r] / [f]

[Space] : Trở về Home (180, 0, 0, 0)
CTRL-C để thoát.
"""

# Khai báo mức độ thay đổi cho mỗi lần bấm phím
move_bindings = {
    'q': (5.0, 0.0, 0.0, 0.0),   # +5 mm
    'a': (-5.0, 0.0, 0.0, 0.0),  # -5 mm
    'w': (0.0, 5.0, 0.0, 0.0),   # +5 độ
    's': (0.0, -5.0, 0.0, 0.0),  # -5 độ
    'e': (0.0, 0.0, -5.0, 0.0),   # +5 độ
    'd': (0.0, 0.0, 5.0, 0.0),  # -5 độ
    'r': (0.0, 0.0, 0.0, 5.0),   # +5 độ
    'f': (0.0, 0.0, 0.0, -5.0),  # -5 độ
}

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_test')

        self.command_pub = self.create_publisher(
            Float64MultiArray, 
            '/arm_controller/commands', 
            10
        )

        self.joint_state_sub = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.joint_states_callback,
            10
        )

        # Lưu vị trí hiện tại dạng (mm, độ, độ, độ)
        self.target_pos = [0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'gripper_joint']
        self.is_synced = False

    def joint_states_callback(self, msg_data):
        # Chỉ đồng bộ 1 lần lúc mới bật lên để không bị giật tay máy
        if not self.is_synced:
            temp_pos = [None] * 4
            for i, name in enumerate(msg_data.joint_names):
                if name in self.joint_names:
                    idx = self.joint_names.index(name)
                    
                    interface_value_msg = msg_data.interface_values[i]
                    if 'position' in interface_value_msg.interface_names:
                        pos_idx = interface_value_msg.interface_names.index('position')
                        sys_val = interface_value_msg.values[pos_idx]

                        # ====================================================
                        # CHUYỂN NGƯỢC TỪ HỆ THỐNG (MÉT/RAD) VỀ NGƯỜI DÙNG (MM/ĐỘ)
                        # ====================================================
                        if idx == 0:
                            # Prismatic Arm 1: Mét -> mm
                            temp_pos[idx] = sys_val * 1000.0
                        else:
                            # Arm 2, 3, Gripper: Radian -> Độ
                            temp_pos[idx] = math.degrees(sys_val)
            
            if all(p is not None for p in temp_pos):
                self.target_pos = temp_pos
                self.is_synced = True
                print("\r\n[OK] Đã đồng bộ vị trí! Bắt đầu điều khiển được rồi.\n")
                self.print_status()

    def clamp(self, val, min_val, max_val):
        return max(min(float(val), float(max_val)), float(min_val))

    def update_target(self, deltas):
        if not self.is_synced:
            return 

        # Cộng dồn góc mới và giới hạn (Clamp)
        self.target_pos[0] = self.clamp(self.target_pos[0] + deltas[0], 0.0, 780.0)
        self.target_pos[1] = self.clamp(self.target_pos[1] + deltas[1], -5.0, 90.0)
        self.target_pos[2] = self.clamp(self.target_pos[2] + deltas[2], -90.0, 90.0)
        self.target_pos[3] = self.clamp(self.target_pos[3] + deltas[3], 0.0, 240.0)

        self.publish_command()

    def reset_home(self):
        if self.is_synced:
            self.target_pos = [0.0, -5.0, 0.0, 45.0]
            self.publish_command()

    def publish_command(self):
        # ====================================================
        # CHUYỂN TỪ NGƯỜI DÙNG (MM/ĐỘ) XUỐNG HỆ THỐNG (MÉT/RAD)
        # ====================================================
        sys_target = [0.0, 0.0, 0.0, 0.0]
        
        # Arm 1: mm -> Mét
        sys_target[0] = self.target_pos[0] / 1000.0  
        
        # Arm 2, 3, Gripper: Độ -> Radian
        sys_target[1] = math.radians(self.target_pos[1])
        sys_target[2] = math.radians(self.target_pos[2])
        GRIPPER_OFFSET = -30.0 
        sys_target[3] = math.radians(self.target_pos[3] + GRIPPER_OFFSET)

        cmd_msg = Float64MultiArray()
        cmd_msg.data = sys_target
        self.command_pub.publish(cmd_msg)
        
        self.print_status()

    def print_status(self):
        sys.stdout.write(f"\rVị trí -> Base: {self.target_pos[0]:.1f}mm | Shoulder: {self.target_pos[1]:.1f}° | Elbow: {self.target_pos[2]:.1f}° | Grip: {self.target_pos[3]:.1f}°    ")
        sys.stdout.flush()


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TeleopKeyboardNode()

    # Chạy ROS spin ngầm để nhận callback đồng bộ
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(msg)

    try:
        while True:
            key = get_key(settings)

            if key in move_bindings:
                node.update_target(move_bindings[key])
            elif key == ' ':
                node.reset_home()
            elif key == '\x03': # Bấm CTRL-C
                break

    except Exception as e:
        print(f"\nLỗi: {e}")
    finally:
        print("\n\nĐang tắt Teleop...")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    