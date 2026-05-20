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
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class OdomNavSwerveServer(Node):
    def __init__(self):
        super().__init__('odom_nav_swerve_server')
        
        # --- 1. Khai báo ROS 2 Parameters (Dễ dàng tuning) ---
        self.declare_parameter('dist_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.035) # ~2 độ
        self.declare_parameter('kp_xy', 1.5)
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('max_vel_xy', 0.6)
        self.declare_parameter('min_vel_xy', 0.05) # Đủ để thắng ma sát tĩnh
        self.declare_parameter('max_vel_z', 0.6)
        self.declare_parameter('min_vel_z', 0.1)   # Đủ để thắng ma sát tĩnh xoay

        # --- 2. Tối ưu Callback Groups ---
        # Action Server có thể nhận Cancel/Goal mới đồng thời (Reentrant)
        self.action_cb_group = ReentrantCallbackGroup()
        # Odom chỉ cần cập nhật tuần tự, tránh tranh chấp tài nguyên (Mutually Exclusive)
        self.odom_cb_group = MutuallyExclusiveCallbackGroup()

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_cb_group
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self.odom_callback, 
            10, 
            callback_group=self.odom_cb_group
        )

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
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def odom_callback(self, msg):
        with self.odom_lock:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            self.odom_received = True

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def apply_velocity_limits(self, velocity, max_v, min_v):
        """Hàm giới hạn vận tốc an toàn và chống kẹt ma sát tĩnh"""
        if velocity == 0.0: return 0.0
        sign = 1.0 if velocity > 0 else -1.0
        abs_vel = abs(velocity)
        
        if abs_vel > max_v:
            return max_v * sign
        elif abs_vel < min_v:
            return min_v * sign
        return velocity

    def execute_callback(self, goal_handle):
        self.get_logger().info('🎯 NHẬN LỆNH DI CHUYỂN SWERVE MỚI!')

        if not self.odom_received:
            self.get_logger().error('Chưa nhận được tín hiệu /odom! Hủy lệnh.')
            goal_handle.abort()
            return NavigateToPose.Result()

        target_x = goal_handle.request.pose.pose.position.x
        target_y = goal_handle.request.pose.pose.position.y
        target_yaw_final = self.quaternion_to_yaw(goal_handle.request.pose.pose.orientation)

        with self.odom_lock:
            initial_yaw = self.current_yaw 

        # Đọc tham số ROS 2
        p_dist_tol = self.get_parameter('dist_tolerance').value
        p_yaw_tol = self.get_parameter('yaw_tolerance').value
        p_kp_xy = self.get_parameter('kp_xy').value
        p_kp_ang = self.get_parameter('kp_angular').value
        p_max_v_xy = self.get_parameter('max_vel_xy').value
        p_min_v_xy = self.get_parameter('min_vel_xy').value
        p_max_v_z = self.get_parameter('max_vel_z').value
        p_min_v_z = self.get_parameter('min_vel_z').value
        
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
                self.stop_robot()
                return NavigateToPose.Result()

            with self.odom_lock:
                cx, cy, cyaw = self.current_x, self.current_y, self.current_yaw

            dx = target_x - cx
            dy = target_y - cy
            distance = math.hypot(dx, dy)

            if distance < p_dist_tol:
                break

            # Tính vector toàn cục
            vel_x_global = p_kp_xy * dx
            vel_y_global = p_kp_xy * dy
            heading_error = self.normalize_angle(initial_yaw - cyaw)

            cos_y = math.cos(cyaw)
            sin_y = math.sin(cyaw)

            # Chuyển hệ tọa độ (Global -> Local)
            vel_x_local = vel_x_global * cos_y + vel_y_global * sin_y
            vel_y_local = -vel_x_global * sin_y + vel_y_global * cos_y
            
            # Kẹp giới hạn vector tịnh tiến (Clamp & Anti-stall)
            vel_mag = math.hypot(vel_x_local, vel_y_local)
            if vel_mag > 0:
                if vel_mag > p_max_v_xy:
                    scale = p_max_v_xy / vel_mag
                elif vel_mag < p_min_v_xy:
                    scale = p_min_v_xy / vel_mag
                else:
                    scale = 1.0
                    
                vel_x_local *= scale
                vel_y_local *= scale

            cmd_msg.linear.x = vel_x_local
            cmd_msg.linear.y = vel_y_local
            
            # --- FIX LỖI KHÓA YAW TẠI ĐÂY ---
            # Sử dụng Deadband: Chỉ bù góc nếu sai số lớn hơn dung sai cho phép
            if abs(heading_error) < p_yaw_tol:
                cmd_msg.angular.z = 0.0
            else:
                # Dùng p_min_v_z để đảm bảo lực xoay luôn thắng được ma sát tĩnh
                cmd_msg.angular.z = self.apply_velocity_limits(p_kp_ang * heading_error, p_max_v_z, p_min_v_z)
            
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
                self.stop_robot()
                return NavigateToPose.Result()

            with self.odom_lock:
                cyaw = self.current_yaw

            yaw_error = self.normalize_angle(target_yaw_final - cyaw)
            
            if abs(yaw_error) < p_yaw_tol:
                break

            cmd_msg.linear.x = 0.0
            cmd_msg.linear.y = 0.0
            # Giới hạn tốc độ xoay và đảm bảo tốc độ xoay tối thiểu
            cmd_msg.angular.z = self.apply_velocity_limits(p_kp_ang * yaw_error, p_max_v_z, p_min_v_z)
            
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
