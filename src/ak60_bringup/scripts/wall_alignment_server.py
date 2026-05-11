#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf_transformations

import math
import time
import sys
import os

from ak60_bringup.action import WallAlignment

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
from r2_bt.config import WALL_ALIGN_PARAMS

class WallAlignmentServer(Node):
    def __init__(self):
        super().__init__('wall_alignment_server')
        self.cb_group = ReentrantCallbackGroup()

        # 1. Action Server
        self._action_server = ActionServer(
            self, WallAlignment, 'align_and_approach_wall',
            self.execute_callback, callback_group=self.cb_group
        )

        # 2. Subscribers (Lidar & Odom)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback,
            qos_profile=qos_profile_sensor_data, callback_group=self.cb_group
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 
            10, callback_group=self.cb_group
        )
        
        # 3. Publishers (Cmd_vel & Set_pose)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/set_pose', 10)
        
        # Biến trạng thái
        self.latest_scan = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.kp_angular           = WALL_ALIGN_PARAMS['kp_angular']
        self.kp_linear            = WALL_ALIGN_PARAMS['kp_linear']
        self.tolerance_deg        = WALL_ALIGN_PARAMS['tolerance_deg']
        self.tolerance_m          = WALL_ALIGN_PARAMS['tolerance_m']
        self.lidar_yaw_offset_deg = WALL_ALIGN_PARAMS['lidar_yaw_offset_deg']

        self.get_logger().info('Action Server [Align -> Snap Yaw -> Approach] Đã sẵn sàng!')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def get_wall_state(self, scan_msg, window_rad):
        valid_points = []
        offset_rad = math.radians(self.lidar_yaw_offset_deg)

        for i, r in enumerate(scan_msg.ranges):
            raw_angle = scan_msg.angle_min + i * scan_msg.angle_increment
            angle_in_base = math.atan2(math.sin(raw_angle + offset_rad), math.cos(raw_angle + offset_rad))

            if -window_rad/2 <= angle_in_base <= window_rad/2:
                if not math.isinf(r) and not math.isnan(r) and scan_msg.range_min <= r <= scan_msg.range_max:
                    valid_points.append((r, angle_in_base))

        if len(valid_points) < 5:
            return None, None

        xs = [r * math.cos(th) for r, th in valid_points]
        ys = [r * math.sin(th) for r, th in valid_points]
        
        n = len(xs)
        sum_y, sum_x = sum(ys), sum(xs)
        sum_yy = sum(y*y for y in ys)
        sum_yx = sum(y*x for x, y in zip(xs, ys))

        denominator = (n * sum_yy) - (sum_y ** 2)
        m = 0.0 if denominator == 0 else (n * sum_yx - sum_y * sum_x) / denominator

        deviation_rad = math.atan(m)
        c = (sum_x - m * sum_y) / n
        return math.degrees(deviation_rad), c

    def snap_yaw_to_nearest_orthogonal(self):
        self.get_logger().info('Giai đoạn 2: Đang Snap Yaw về lưới tọa độ vuông góc...')
        
        pi_over_2 = math.pi / 2.0
        target_yaw_rad = round(self.current_yaw / pi_over_2) * pi_over_2
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom' 

        msg.pose.pose.position.x = self.current_x
        msg.pose.pose.position.y = self.current_y
        msg.pose.pose.position.z = 0.0

        target_q = tf_transformations.quaternion_from_euler(0.0, 0.0, target_yaw_rad)
        msg.pose.pose.orientation.x = target_q[0]
        msg.pose.pose.orientation.y = target_q[1]
        msg.pose.pose.orientation.z = target_q[2]
        msg.pose.pose.orientation.w = target_q[3]

        msg.pose.covariance[0] = 1e-9   
        msg.pose.covariance[7] = 1e-9   
        msg.pose.covariance[35] = 1e-9  

        self.set_pose_pub.publish(msg)
        self.get_logger().info(f'Đã Snap Yaw từ {math.degrees(self.current_yaw):.1f}° về {math.degrees(target_yaw_rad):.1f}°')

    def execute_callback(self, goal_handle):
        window_rad = math.radians(goal_handle.request.window_degrees)
        goal_dist = goal_handle.request.goal_distance
        feedback_msg = WallAlignment.Feedback()
        cmd = Twist()
        
        # [TỐI ƯU 1]: Sử dụng đối tượng Rate của ROS 2 thay vì time.sleep để không block luồng
        loop_rate = self.create_rate(20) # Chạy ở tần số 20Hz (0.05s)

        # [TỐI ƯU 2]: Thêm rclpy.ok() vào vòng lặp chờ Lidar để tránh bị kẹt cứng nếu Lidar rớt
        while self.latest_scan is None and rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return WallAlignment.Result()
            time.sleep(0.1)
            
        if not rclpy.ok():
            return WallAlignment.Result()

        try:
            # ==========================================
            # PHASE 1: XOAY VUÔNG GÓC (ALIGNING)
            # ==========================================
            self.get_logger().info('PHASE 1: Đang xoay vuông góc với tường...')
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return WallAlignment.Result()

                dev_deg, current_dist = self.get_wall_state(self.latest_scan, window_rad)
                if dev_deg is None: 
                    self.get_logger().warn('Cảnh báo: Không tìm thấy đủ 5 điểm Laser trước mặt!', throttle_duration_sec=1.0)
                    loop_rate.sleep()
                    continue

                feedback_msg.current_phase = "ALIGNING"
                feedback_msg.current_deviation = dev_deg
                goal_handle.publish_feedback(feedback_msg)

                if abs(dev_deg) < self.tolerance_deg:
                    self.stop_robot()
                    time.sleep(0.5) # Chờ dao động cơ khí triệt tiêu
                    break

                cmd.angular.z = -self.kp_angular * math.radians(dev_deg)
                clamp = WALL_ALIGN_PARAMS['ang_clamp']
                cmd.angular.z = max(min(cmd.angular.z, clamp), -clamp)
                self.cmd_pub.publish(cmd)
                
                loop_rate.sleep() # Thay thế time.sleep(0.05)

            # ==========================================
            # PHASE 2: CHỐT GÓC YAW (SNAPPING YAW)
            # ==========================================
            feedback_msg.current_phase = "SNAPPING_YAW"
            goal_handle.publish_feedback(feedback_msg)
            
            self.snap_yaw_to_nearest_orthogonal()
            time.sleep(0.5) # Đợi 0.5s để EKF cập nhật

            # ==========================================
            # PHASE 3: TIẾN ÁP SÁT TƯỜNG (BỎ QUA NẾU = 0)
            # ==========================================
            if goal_dist > 0.001:  
                self.get_logger().info(f'PHASE 3: Đang tiến vào khoảng cách mục tiêu {goal_dist}m...')
                while rclpy.ok():
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return WallAlignment.Result()

                    dev_deg, current_dist = self.get_wall_state(self.latest_scan, window_rad)
                    if current_dist is None: 
                        loop_rate.sleep()
                        continue

                    error_dist = current_dist - goal_dist

                    feedback_msg.current_phase = "APPROACHING"
                    feedback_msg.current_distance = current_dist
                    goal_handle.publish_feedback(feedback_msg)

                    if abs(error_dist) < self.tolerance_m:
                        self.stop_robot()
                        break

                    cmd.angular.z = 0.0 
                    cmd.linear.x = self.kp_linear * error_dist
                    cmd.linear.x = max(min(cmd.linear.x, 0.2), -0.2)
                    self.cmd_pub.publish(cmd)
                    
                    loop_rate.sleep() # Thay thế time.sleep(0.05)
            else:
                self.get_logger().info('PHASE 3: Bỏ qua bước tiến (Goal distance = 0).')

            # Hoàn thành
            goal_handle.succeed()
            result = WallAlignment.Result()
            result.success = True
            result.final_deviation, result.final_distance = self.get_wall_state(self.latest_scan, window_rad)
            
            self.get_logger().info(f'HOÀN THÀNH TOÀN BỘ! Cách tường: {result.final_distance:.3f}m | Lệch: {result.final_deviation:.2f}°')
            return result

        except Exception as e:
            self.get_logger().error(f'Lỗi đột xuất trong quá trình chạy: {e}')
            goal_handle.abort()
            result = WallAlignment.Result()
            result.success = False
            return result
            
        finally:
            # [TỐI ƯU 3]: Bùa hộ mệnh tuyệt đối - Bất kể vòng lặp kết thúc do Cancel, Crash hay Hoàn thành, xe PHẢI PHANH LẠI.
            self.stop_robot()

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = WallAlignmentServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Đang tắt Node, phanh xe an toàn...")
            node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
