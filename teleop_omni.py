#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

msg = """
========================================
 ĐIỀU KHIỂN ROCKER-BOGIE 6 BÁNH (OMNI)
========================================
Sử dụng bàn phím để xuất lệnh /cmd_vel:

   Q(Xoay Trái)    W(Tiến)     E(Xoay Phải)
   A(Ngang Trái)   S(Lùi)      D(Ngang Phải)

Phím SPACE hoặc 'K': Dừng lại ngay lập tức (Stop)

CTRL-C để thoát.
========================================
"""

moveBindings = {
    'w': ( 1.0,  0.0,  0.0), # Tiến
    's': (-1.0,  0.0,  0.0), # Lùi
    'a': ( 0.0,  1.0,  0.0), # Đi ngang trái
    'd': ( 0.0, -1.0,  0.0), # Đi ngang phải
    'q': ( 0.0,  0.0,  1.0), # Xoay trái
    'e': ( 0.0,  0.0, -1.0), # Xoay phải
}

speed_lin = 0.3  # Tốc độ tịnh tiến (m/s)
speed_ang = 0.5  # Tốc độ góc (rad/s)

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()

    node = rclpy.create_node('teleop_omni_keyboard')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    x = 0.0
    y = 0.0
    th = 0.0

    print(msg)

    try:
        while True:
            key = getKey(settings)

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
                print(f"\rLệnh xuất: vx={x*speed_lin:.2f}, vy={y*speed_lin:.2f}, wz={th*speed_ang:.2f}     ", end="")
            elif key == ' ' or key == 'k':
                x = 0.0
                y = 0.0
                th = 0.0
                print("\rĐÃ DỪNG (STOP)                                      ", end="")
            elif key == '\x03': # CTRL-C
                break
            else:
                continue

            twist = Twist()
            twist.linear.x = x * speed_lin
            twist.linear.y = y * speed_lin
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * speed_ang
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Gửi lệnh dừng trước khi tắt node
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    