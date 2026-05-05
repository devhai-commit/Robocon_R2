#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Lời chào và hướng dẫn hiển thị trên Terminal
msg = """
====================================
🕹️ ĐIỀU KHIỂN AK60 SWERVE DRIVE 🕹️
====================================
W / S : Tiến / Lùi (Trục X)
A / D : Đi ngang Trái / Phải (Trục Y - Crab Walk)
Q / E : Xoay tại chỗ Trái / Phải (Trục Z - Zero Turn)

Phím SPACE : Phanh gấp (Dừng mọi chuyển động)
CTRL-C để thoát
====================================
"""

# Từ điển ánh xạ Phím bấm -> (Tốc độ X, Tốc độ Y, Tốc độ Xoay Z)
move_bindings = {
    'w': ( 0.1,  0.0,  0.0), # Tiến
    's': (-0.1,  0.0,  0.0), # Lùi
    'a': ( 0.0,  0.1,  0.0), # Đi ngang sang trái
    'd': ( 0.0, -0.1,  0.0), # Đi ngang sang phải
    'q': ( 0.0,  0.0,  0.2), # Xoay trái
    'e': ( 0.0,  0.0, -0.2), # Xoay phải
}

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('ak60_keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    # Hàm đọc từng phím bấm trực tiếp từ Linux Terminal
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg)
        twist = Twist()
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key in move_bindings.keys():
                    x, y, z = move_bindings[key]
                    twist.linear.x = x
                    twist.linear.y = y
                    twist.angular.z = z
                    print(f"Lệnh: Tiến={x}, Ngang={y}, Xoay={z} \r")
                elif key == ' ':
                    # Phanh gấp
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.angular.z = 0.0
                    print("PHANH GẤP! \r")
                elif key == '\x03': # Mã hex của Ctrl+C
                    break
                else:
                    # Nếu không bấm gì (hoặc bấm phím lạ) thì không gửi lệnh mới
                    continue

                self.publisher_.publish(twist)

        except Exception as e:
            print(e)
        finally:
            # Gửi lệnh dừng hẳn xe trước khi tắt Node
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
