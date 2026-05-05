#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient

# Import các bản tin Pub/Sub thông thường
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# Import bản tin Action chuẩn cho Tay gắp
from control_msgs.action import GripperCommand

msg = """
=============================================
🤖 BẢNG ĐIỀU KHIỂN TELEOP (CHUẨN ACTION)
=============================================
1. Bánh xe Rocker-Bogie (/cmd_vel):
   w/s : Tiến / Lùi (vx)
   a/d : Xoay tại chỗ (wz)
   q/e : Đi ngang (vy)

2. Cánh tay (/arm_controller/joint_trajectory):
   u/j : Khớp tay 1
   i/k : Khớp tay 2
   o/l : Khớp tay 3

3. Tay gắp (/gripper_controller/gripper_cmd):
   z/c : Kẹp Waveshare (Mở / Đóng - Action Client)

space  : Dừng khẩn cấp (Reset bánh xe)
CTRL-C : Thoát chương trình
=============================================
"""

LIN_VEL_STEP = 0.1  
ANG_VEL_STEP = 0.1
ARM_POS_STEP = 5.0*math.pi/180.0

class TeleopAK60(Node):
    def __init__(self):
        super().__init__('teleop_ak60_keyboard')
        
        # 1. Bánh xe (Pub)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. Cánh tay (Pub Trajectory)
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        
        # 3. Tay gắp (ACTION CLIENT)
        # self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # Trạng thái hiện tại
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        
        self.arm_j1 = 0.0
        self.arm_j2 = 0.0
        self.arm_j3 = 0.0
        self.gripper_pos = 0.0

        # self.get_logger().info('Đang chờ Action Server của Tay gắp khởi động...')
        # Đợi Action Server sẵn sàng (Timeout 2 giây để không bị treo nếu bạn chưa bật controller)
        # server_up = self.gripper_client.wait_for_server(timeout_sec=2.0)
        # if server_up:
        #     self.get_logger().info('✅ Đã kết nối với Gripper Action Server!')
        # else:
        #     self.get_logger().warning('⚠️ Không tìm thấy Gripper Action Server! Phím z/c có thể không hoạt động.')

    def publish_continuous_commands(self):
        # Gửi liên tục lệnh bánh xe
        twist = Twist()
        twist.linear.x = self.target_vx
        twist.linear.y = self.target_vy
        twist.angular.z = self.target_wz

        self.cmd_vel_pub.publish(twist)

        point = Float64MultiArray()
        point.data = [self.arm_j1, self.arm_j2, self.arm_j3, self.gripper_pos]
        
        self.arm_pub.publish(point)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = TeleopAK60()

    print(msg)

    try:
        while True:
            # Cho phép ROS 2 xử lý các luồng ngầm (như nhận kết quả Action)
            rclpy.spin_once(node, timeout_sec=0.01)

            key = get_key(settings)
            
            # Cờ đánh dấu có cần gửi Action tay gắp hay không
            send_gripper = False

            if key == 'w':
                node.target_vx += LIN_VEL_STEP
            elif key == 's':
                node.target_vx -= LIN_VEL_STEP
            elif key == 'a':
                node.target_wz += ANG_VEL_STEP
            elif key == 'd':
                node.target_wz -= ANG_VEL_STEP
            elif key == 'q':
                node.target_vy += LIN_VEL_STEP
            elif key == 'e':
                node.target_vy -= LIN_VEL_STEP
                
            elif key == 'u':
                node.arm_j1 += ARM_POS_STEP
            elif key == 'j':
                node.arm_j1 -= ARM_POS_STEP
            elif key == 'i':
                node.arm_j2  += ARM_POS_STEP
            elif key == 'k':
                node.arm_j2 -= ARM_POS_STEP
            elif key == 'o':
                node.arm_j3 -= ARM_POS_STEP
            elif key == 'l':
                node.arm_j3 += ARM_POS_STEP
                
            # Tay gắp -> Bật cờ gửi Action
            elif key == 'z':
                node.gripper_pos = 1.57
                send_gripper = True
            elif key == 'c':
                node.gripper_pos = 0.0
                send_gripper = True
                
            elif key == ' ':
                node.target_vx = 0.0
                node.target_vy = 0.0
                node.target_wz = 0.0
            elif key == '\x03':
                break

            sys.stdout.write(f"\rVx: {node.target_vx:.1f} | Vy: {node.target_vy:.1f} | Wz: {node.target_wz:.1f} || Arm: [{node.arm_j1:.2f}, {node.arm_j2:.2f}, {node.arm_j3:.2f}] | Grip: {node.gripper_pos:.2f}   ")
            sys.stdout.flush()

            node.target_vx = max(min(node.target_vx, 0.4), -0.4)
            node.target_vy = max(min(node.target_vy, 0.4), -0.4)
            node.target_wz = max(min(node.target_wz, 0.5), -0.5)

            node.arm_j1 =  max(min(node.arm_j1, 19.5), 0.0)
            node.arm_j2 =  max(min(node.arm_j2, 1.7), -5.0*math.pi/180.0)
            node.arm_j3 =  max(min(node.arm_j3, 1.57), -1.57)

            node.gripper_pos =  max(min(node.gripper_pos, 1.57), 0.0)

            # Luôn Publish bánh xe & cánh tay
            node.publish_continuous_commands()
            

    except Exception as e:
        print(f"\nLỗi: {e}")
    finally:
        node.target_vx = 0.0
        node.target_vy = 0.0
        node.target_wz = 0.0
        node.publish_continuous_commands()
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()