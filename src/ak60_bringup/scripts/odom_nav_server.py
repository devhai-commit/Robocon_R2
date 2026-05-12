#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import threading

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class OdomNavSwerveServer(Node):
    def __init__(self):
        super().__init__('odom_nav_swerve_server')
        
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self.odom_callback, 
            10, 
            callback_group=self.cb_group
        )

        # Khóa chống Race Condition
        self.odom_lock = threading.Lock()
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

    def cancel_callback(self, cancel_request):
        self.get_logger().warn('⚠️ Nhận yêu cầu HỦY (Cancel). Chấp nhận phanh khẩn cấp!')
        return CancelResponse.ACCEPT

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Tối ưu hóa phép chuẩn hóa góc bằng modulo"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def odom_callback(self, msg):
        with self.odom_lock:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            self.odom_received = True

    def stop_robot(self):
        """Hàm phụ trợ để dừng robot"""
        cmd_msg = Twist()
        self.cmd_vel_pub.publish(cmd_msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info('🎯 NHẬN LỆNH DI CHUYỂN SWERVE MỚI!')

        if not self.odom_received:
            self.get_logger().error('Chưa nhận được tín hiệu /odom! Hủy lệnh.')
            goal_handle.abort()
            return NavigateToPose.Result()

        target_x = goal_handle.request.pose.pose.position.x
        target_y = goal_handle.request.pose.pose.position.y
        target_yaw_final = self.quaternion_to_yaw(goal_handle.request.pose.pose.orientation)

        # Lấy trạng thái bắt đầu một cách an toàn
        with self.odom_lock:
            initial_yaw = self.current_yaw 

        # --- Cấu hình tham số ---
        dist_tolerance = 0.05
        yaw_tolerance = 2.0 * math.pi / 180.0  # 2 độ  
        kp_xy = 1.5
        kp_angular = 1.0
        max_vel_xy = 0.6
        max_vel_z = 0.6
        
        # Chạy vòng lặp ở tần số 20Hz (0.05s) chuẩn ROS 2
        rate = self.create_rate(20) 
        feedback_msg = NavigateToPose.Feedback()
        cmd_msg = Twist()

        # ========================================================
        # GIAI ĐOẠN 1: ĐI NGANG HOLONOMIC
        # ========================================================
        self.get_logger().info('Giai đoạn 1: Đi Holonomic đến mục tiêu...')
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('🛑 Đã hủy lệnh Giai đoạn 1.')
                self.stop_robot()
                return NavigateToPose.Result()

            with self.odom_lock:
                cx, cy, cyaw = self.current_x, self.current_y, self.current_yaw

            dx = target_x - cx
            dy = target_y - cy
            distance = math.hypot(dx, dy)

            if distance < dist_tolerance:
                break

            # Tính toán vector vận tốc toàn cục (Global)
            vel_x_global = kp_xy * dx
            vel_y_global = kp_xy * dy
            
            # Giữ nguyên góc hiện tại khi đang tịnh tiến (Tránh bị xoay ngang xe do sai số)
            heading_error = self.normalize_angle(initial_yaw - cyaw)

            # Chuyển hệ tọa độ (Global -> Local)
            vel_x_local = vel_x_global * math.cos(-cyaw) - vel_y_global * math.sin(-cyaw)
            vel_y_local = vel_x_global * math.sin(-cyaw) + vel_y_global * math.cos(-cyaw)
            
            # --- TỐI ƯU CỰC KỲ QUAN TRỌNG: Kẹp giới hạn (Clamp) theo vector, không kẹp từng trục
            vel_mag = math.hypot(vel_x_local, vel_y_local)
            if vel_mag > max_vel_xy:
                scale = max_vel_xy / vel_mag
                vel_x_local *= scale
                vel_y_local *= scale

            cmd_msg.linear.x = vel_x_local
            cmd_msg.linear.y = vel_y_local
            cmd_msg.angular.z = max(-max_vel_z, min(max_vel_z, kp_angular * heading_error))
            self.cmd_vel_pub.publish(cmd_msg)

            feedback_msg.distance_remaining = distance
            goal_handle.publish_feedback(feedback_msg)
            rate.sleep()

        self.stop_robot()

        # ========================================================
        # GIAI ĐOẠN 2: XOAY TẠI CHỖ
        # ========================================================
        
        self.get_logger().info('Giai đoạn 2: Đã tới vị trí, Đang xoay tại chỗ...')
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('🛑 Đã hủy lệnh Giai đoạn 2.')
                self.stop_robot()
                return NavigateToPose.Result()

            with self.odom_lock:
                cyaw = self.current_yaw

            yaw_error = self.normalize_angle(target_yaw_final - cyaw)

            self.get_logger().info(f'Goc lech hien tai: {yaw_error}')
            
            if abs(yaw_error) < yaw_tolerance:
                break

            cmd_msg.linear.x = 0.0
            cmd_msg.linear.y = 0.0
            cmd_msg.angular.z = max(-max_vel_z, min(max_vel_z, kp_angular * yaw_error))
            self.cmd_vel_pub.publish(cmd_msg)
            rate.sleep()

        self.stop_robot()
        goal_handle.succeed()
        self.get_logger().info('✅ LỆNH HOÀN TẤT: Đã đạt tọa độ và góc xoay!')
        return NavigateToPose.Result()

def main(args=None):
    rclpy.init(args=args)
    server = OdomNavSwerveServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    