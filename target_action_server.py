#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point, Twist
from ak60_bringup.action import FollowTarget # Thay bằng tên package của bạn
import time

class TargetActionServer(Node):
    def __init__(self):
        super().__init__('target_action_server')
        
        # Sử dụng ReentrantCallbackGroup để có thể vừa chạy Action vừa nhận Topic
        self.callback_group = ReentrantCallbackGroup()

        # 1. Khởi tạo Action Server
        self._action_server = ActionServer(
            self,
            FollowTarget,
            'follow_target',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        # 2. Subscriber & Publisher
        self.subscription = self.create_subscription(
            Point, '/target_coordinates', self.coords_callback, 10,
            callback_group=self.callback_group)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Biến lưu trữ trạng thái
        self.current_coords = None
        self.img_center = (320, 240)
        self.threshold = 20.0
        self.kp = 0.002

        self.get_logger().info("Action Server 'follow_target' đã sẵn sàng...")

    def coords_callback(self, msg):
        self.current_coords = msg

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Nhận yêu cầu bám đuổi Target ID: {goal_request.target_id}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Đã nhận yêu cầu HỦY Action")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Bắt đầu thực hiện điều chỉnh robot...")
        
        feedback_msg = FollowTarget.Feedback()
        result = FollowTarget.Result()
        twist = Twist()

        # Vòng lặp điều khiển
        while rclpy.ok():
            # 1. Kiểm tra yêu cầu hủy
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_robot()
                result.success = False
                return result

            # 2. Nếu không nhận được tọa độ (Target lost)
            if self.current_coords is None:
                self.get_logger().warn("Mất dấu mục tiêu...")
                self.stop_robot()
                time.sleep(0.1)
                continue

            # 3. Tính toán sai số
            error_x = self.current_coords.x - self.img_center[0]
            error_y = self.img_center[1] - self.current_coords.y
            dist = (error_x**2 + error_y**2)**0.5

            # 4. Gửi feedback về client
            feedback_msg.error_distance = float(dist)
            goal_handle.publish_feedback(feedback_msg)

            # 5. Kiểm tra điều kiện dừng (Thành công)
            if dist < self.threshold:
                self.get_logger().info("Mục tiêu ĐÃ VÀO TÂM!")
                self.stop_robot()
                goal_handle.succeed()
                result.success = True
                return result

            # 6. Tính toán P-Control và Publish cmd_vel
            twist.linear.x = error_y * self.kp
            twist.linear.y = -error_x * self.kp
            twist.angular.z = -error_x * 0.005 # Quay để đối diện vật thể
            self.cmd_pub.publish(twist)

            time.sleep(0.05) # Chu kỳ 20Hz

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    # Vì Action Server cần chạy đa luồng để không bị block
    node = TargetActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    