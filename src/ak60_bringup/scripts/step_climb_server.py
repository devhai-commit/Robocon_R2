#!/usr/bin/env python3
import rclpy
import time  # Thay đổi: Import thư viện time mặc định
import math
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Import Action của bạn
from ak60_bringup.action import ClimbStep 

class StepClimbServer(Node):
    def __init__(self):
        super().__init__('step_climb_server')
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self._action_server = ActionServer(
            self, ClimbStep, 'climb_step', self.execute_callback)
        
        self.current_pitch = 0.0
        self.get_logger().info('Action Server Leo Bậc (Odometry & Math) đã sẵn sàng!')

    def odom_callback(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch_rad = math.copysign(math.pi / 2, sinp) 
        else:
            pitch_rad = math.asin(sinp)
        self.current_pitch = math.degrees(pitch_rad)

    # Thay đổi: Bỏ chữ "async" trước def
    def execute_callback(self, goal_handle):
        self.get_logger().info('Nhận lệnh leo bậc. Bắt đầu di chuyển...')
        
        target_vel = goal_handle.request.velocity
        feedback_msg = ClimbStep.Feedback()
        result = ClimbStep.Result()
        
        stage = 0
        pitch_threshold = 5.0  
        flat_threshold = 1.5   
        move_cmd = Twist()
        move_cmd.linear.x = target_vel

        start_time = self.get_clock().now()
        timeout_sec = 15.0 

        try:
            while rclpy.ok():
                # 1. Kiểm tra Client hủy lệnh
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Client yêu cầu hủy lệnh.')
                    goal_handle.canceled()
                    result.success = False
                    return result

                # 2. Kiểm tra Timeout
                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed_time > timeout_sec:
                    self.get_logger().error('Quá thời gian leo bậc (Timeout)!')
                    goal_handle.abort()
                    result.success = False
                    return result

                # 3. Điều khiển & Feedback
                self._cmd_vel_pub.publish(move_cmd)
                feedback_msg.current_pitch = self.current_pitch
                goal_handle.publish_feedback(feedback_msg)

                # 4. Logic leo bậc
                if stage == 0:
                    if abs(self.current_pitch) > pitch_threshold:
                        stage = 1
                        self.get_logger().info(f'Phát hiện leo bậc: Pitch = {self.current_pitch:.2f}')
                elif stage == 1:
                    if abs(self.current_pitch) < flat_threshold:
                        self.get_logger().info('Đã vượt qua bậc. Robot đã phẳng.')
                        goal_handle.succeed()
                        result.success = True
                        return result

                # Thay đổi: Dùng time.sleep() mặc định thay vì asyncio
                time.sleep(0.05) 

        finally:
            self._stop_robot()

    def _stop_robot(self):
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info('Đã dừng robot.')

def main(args=None):
    rclpy.init(args=args)
    node = StepClimbServer()
    
    # MultiThreadedExecutor giúp các hàm callback chạy song song
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
    