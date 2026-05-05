#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time

class RobotFollowerNode(Node):
    def __init__(self):
        super().__init__('robot_follower_node')

        # 1. Subscriber: Lắng nghe tọa độ từ YOLO Node
        self.subscription = self.create_subscription(
            Point,
            '/target_coordinates',
            self.listener_callback,
            10)

        # 2. Publisher: Gửi lệnh vận tốc tới robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # 3. Thông số khung hình và mục tiêu (Center)
        self.img_width = 640
        self.img_height = 480
        self.center_x = self.img_width / 2  # 320
        self.center_y = self.img_height / 2 # 240

        # 4. Ngưỡng sai số cho phép (Tolerance) - Đơn vị: Pixel
        self.threshold = 25 

        # 5. Hệ số tăng áp (Gains) - Điều chỉnh độ nhạy của robot
        self.kp_linear = 0.002   # Cho vx, vy
        self.kp_angular = 0.005  # Cho angular z

        self.last_time_received = time.time()
        self.get_logger().info("Node Điều Khiển Robot đã sẵn sàng. Đang chờ mục tiêu...")

    def listener_callback(self, msg):
        self.last_time_received = time.time()
        
        # Lấy tọa độ hiện tại của vật thể
        current_x = msg.x
        current_y = msg.y
        
        # Tính toán sai số (Error)
        error_x = current_x - self.center_x
        error_y = self.center_y - current_y # Đảo ngược vì Y trong ảnh tăng từ trên xuống

        twist = Twist()

        # Kiểm tra nếu vật thể đã nằm trong tâm (trong khoảng threshold)
        if abs(error_x) < self.threshold and abs(error_y) < self.threshold:
            self.get_logger().info("MỤC TIÊU ĐÃ VÀO TÂM - DỪNG ROBOT")
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
        else:
            # 6. Tính toán vận tốc dựa trên sai số (P-Control)
            
            # Tiến/Lùi (vx): Phụ thuộc vào sai số trục Y của ảnh
            twist.linear.x = error_y * self.kp_linear
            
            # Di chuyển ngang (vy): Phụ thuộc vào sai số trục X của ảnh
            twist.linear.y = -error_x * self.kp_linear 
            
            # Xoay (angle z): Giúp robot luôn đối mặt với vật thể
            twist.angular.z = -error_x * self.kp_angular

            # Giới hạn tốc độ tối đa (Safety Limit)
            twist.linear.x = max(min(twist.linear.x, 0.2), -0.2)
            twist.linear.y = max(min(twist.linear.y, 0.2), -0.2)
            twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)

        self.publisher_.publish(twist)

    def stop_robot_on_lost(self):
        # Nếu quá 0.5s không thấy vật thể, dừng robot để an toàn
        if time.time() - self.last_time_received > 0.5:
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotFollowerNode()
    
    # Tạo vòng lặp kiểm tra an toàn (Safety check)
    timer_period = 0.1
    node.create_timer(timer_period, node.stop_robot_on_lost)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Dừng robot hẳn khi tắt node
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
