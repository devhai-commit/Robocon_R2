#!/usr/bin/env python3
import math
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from ak60_bringup.action import K230Align

try:
    from r2_bt.jetson_read_uart import latest as k230_data, start_reader_thread
    _UART_AVAILABLE = True
except ImportError:
    k230_data = {"alive": False, "ready": False, "centered": False, "offset_x": None}
    _UART_AVAILABLE = False


class K230AlignServer(Node):
    # --- PID / control params (giống tracking_server PHASE_ALIGN_X) ---
    KP_LATERAL      = 0.0005   # pixel error → m/s lateral
    KP_YAW          = 1.0      # rad error   → rad/s angular
    MAX_LATERAL     = 0.2      # m/s
    MAX_ANGULAR     = 0.3      # rad/s
    MIN_LATERAL     = 0.03     # dead-band kick
    TOLERANCE_PX    = 4.0     # pixel — coi là đã căn giữa
    ALIGN_FRAMES    = 10       # số frame liên tiếp để xác nhận SUCCESS
    LOST_FRAMES_MAX = 10       # số frame mất data trước khi ABORT

    def __init__(self):
        super().__init__('k230_align_server')

        self.cb_group = ReentrantCallbackGroup()

        # Yaw lock state
        self.current_yaw   = 0.0
        self.target_yaw    = 0.0
        self.odom_received = False
        self.odom_lock     = threading.Lock()

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10,
            callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._action_server = ActionServer(
            self, K230Align, 'k230_align',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cb_group)

        if _UART_AVAILABLE:
            start_reader_thread()
            self.get_logger().info('K230AlignServer sẵn sàng (UART reader đã khởi động).')
        else:
            self.get_logger().warn('jetson_read_uart không tìm thấy — chạy ở chế độ fallback.')

    # ------------------------------------------------------------------
    # Callbacks phụ
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        with self.odom_lock:
            self.current_yaw = yaw
            self.odom_received = True

    def _goal_cb(self, goal_request):
        self.get_logger().info('Nhận goal K230Align.')
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().warn('Nhận lệnh HỦY K230Align.')
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _yaw_correction(self):
        with self.odom_lock:
            curr = self.current_yaw
        err = math.atan2(math.sin(self.target_yaw - curr),
                         math.cos(self.target_yaw - curr))
        return max(min(err * self.KP_YAW, self.MAX_ANGULAR), -self.MAX_ANGULAR)

    # ------------------------------------------------------------------
    # Execute
    # ------------------------------------------------------------------

    def _execute_cb(self, goal_handle):
        result = K230Align.Result()

        # Chờ đồng bộ Odometry (giống tracking_server)
        waiting = 0.0
        while not self.odom_received and rclpy.ok():
            if waiting == 0.0:
                self.get_logger().warn('Đang chờ Odometry...')
            time.sleep(0.1)
            waiting += 0.1
            if waiting > 3.0:
                self.get_logger().error('Timeout Odometry. Abort.')
                goal_handle.abort()
                result.success = False
                result.message = 'odom_timeout'
                return result

        # Khóa heading tại thời điểm bắt đầu
        with self.odom_lock:
            self.target_yaw = self.current_yaw
        self.get_logger().info(
            f'Khóa heading: {math.degrees(self.target_yaw):.2f}°')

        aligned_frames = 0
        lost_frames    = 0
        loop_rate = self.create_rate(20)

        try:
            while rclpy.ok():
                # --- Cancel check ---
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'canceled'
                    return result

                # --- K230 không alive → không cần căn, coi là xong ---
                if not k230_data.get('alive', False):
                    self._stop()
                    result.success = True
                    result.message = 'k230_not_alive'
                    goal_handle.succeed()
                    return result

                # --- K230 báo đã căn ---
                if k230_data.get('centered', False):
                    self._stop()
                    result.success = True
                    result.message = 'centered'
                    goal_handle.succeed()
                    return result

                offset_x = k230_data.get('offset_x')

                # Feedback
                fb = K230Align.Feedback()
                fb.offset_x = float(offset_x) if offset_x is not None else 0.0
                fb.centered = bool(k230_data.get('centered', False))
                goal_handle.publish_feedback(fb)

                # --- Mất data ---
                if offset_x is None:
                    lost_frames += 1
                    if lost_frames >= self.LOST_FRAMES_MAX:
                        self.get_logger().error('Mất data K230 quá 10 frame. Abort.')
                        self._stop()
                        result.success = False
                        result.message = 'lost'
                        goal_handle.abort()
                        return result
                    loop_rate.sleep()
                    continue

                lost_frames = 0

                # --- Kiểm tra đã vào tolerance chưa ---
                if abs(offset_x) <= self.TOLERANCE_PX:
                    aligned_frames += 1
                    if aligned_frames >= self.ALIGN_FRAMES:
                        self.get_logger().info('Căn giữa xong (N-frame confirm).')
                        self._stop()
                        result.success = True
                        result.message = 'centered'
                        goal_handle.succeed()
                        return result
                else:
                    aligned_frames = 0

                # --- PID lateral + yaw lock ---
                twist = Twist()
                twist.angular.z = self._yaw_correction()

                # offset_x > 0 → lệch phải → strafe phải → linear.x dương
                v_x = float(offset_x) * self.KP_LATERAL
                if 0 < abs(v_x) < self.MIN_LATERAL:
                    v_x = math.copysign(self.MIN_LATERAL, v_x)
                twist.linear.x = max(min(v_x, self.MAX_LATERAL), -self.MAX_LATERAL)

                self.cmd_pub.publish(twist)
                loop_rate.sleep()

        finally:
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = K230AlignServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Dừng K230AlignServer.')
            node._stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
