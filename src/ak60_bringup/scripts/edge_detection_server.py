import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
# Thay thế bằng tên package chứa file .action thực tế của bạn
from ak60_bringup.action import MoveToEdge 

import time
import math

class EdgeDetectionActionServer(Node):

    def __init__(self):
        super().__init__('edge_detection_action_server')
        
        self.callback_group = ReentrantCallbackGroup()

        # Action Server
        self._action_server = ActionServer(
            self,
            MoveToEdge,
            'move_to_edge',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        # Publisher điều khiển vận tốc
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber cảm biến laser
        self.distance_sub = self.create_subscription(
            Range,
            'laser_distance',
            self.distance_callback,
            10,
            callback_group=self.callback_group
        )

        # Subscriber Odometry (Filtered) để lấy góc Yaw
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom/filtered',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        # Các biến trạng thái
        self.current_distance = None
        self.last_distance = None
        self.current_yaw = None # Góc quay hiện tại (radian)

        # Tham số bộ điều khiển PID (Chỉ dùng P cho việc giữ hướng)
        self.kp_yaw = 1.5 

        self.get_logger().info('Edge Detection Action Server (với Heading Hold) đã sẵn sàng.')

    def distance_callback(self, msg):
        """ Cập nhật dữ liệu từ cảm biến laser """
        self.last_distance = self.current_distance
        self.current_distance = msg.range

    def odom_callback(self, msg):
        """ Cập nhật và tính toán góc Yaw từ Odometry """
        q = msg.pose.pose.orientation
        # Chuyển đổi Quaternion sang góc Euler (Yaw) quanh trục Z
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, goal_request):
        self.get_logger().info('Nhận được yêu cầu di chuyển tìm mép bậc...')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Nhận được yêu cầu hủy lệnh từ Client.')
        return CancelResponse.ACCEPT

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info('Đã phát lệnh dừng robot.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Đang khởi tạo chuỗi hành vi di chuyển...')
        
        speed = goal_handle.request.speed
        threshold = goal_handle.request.change_threshold
        
        feedback_msg = MoveToEdge.Feedback()
        result = MoveToEdge.Result()
        loop_rate = self.create_rate(20) # 20Hz đồng bộ với laser
        
        # 1. Đợi tín hiệu ổn định từ cả Laser và Odom
        while rclpy.ok() and (self.current_distance is None or self.current_yaw is None):
            self.get_logger().info('Đang đợi tín hiệu từ Laser và Odom/filtered...', throttle_duration_sec=1.0)
            time.sleep(0.1)

        # 2. KHÓA GÓC YAW: Lưu lại góc hiện tại khi bắt đầu chạy
        target_yaw = self.current_yaw
        self.get_logger().info(f'Đã khóa hướng mục tiêu: {math.degrees(target_yaw):.2f} độ')

        move_msg = Twist()

        # Vòng lặp điều khiển
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                result.success = False
                result.final_distance = self.current_distance if self.current_distance else 0.0
                return result

            # Phân tích sự thay đổi khoảng cách đột ngột
            if self.last_distance is not None and self.current_distance is not None:
                delta_d = abs(self.current_distance - self.last_distance)
                
                if delta_d > threshold:
                    self.get_logger().warn(
                        f'PHÁT HIỆN MÉP BẬC! Delta d = {delta_d:.4f}m > Ngưỡng {threshold}m.'
                    )
                    self.stop_robot()
                    break 

            # --- LOGIC ĐIỀU CHỈNH HƯỚNG ---
            # Tính sai số góc
            error_yaw = target_yaw - self.current_yaw
            
            # Chuẩn hóa sai số góc về khoảng [-pi, pi] (quan trọng khi vượt qua các mốc 180/-180 độ)
            error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

            # Phát lệnh di chuyển
            move_msg.linear.x = speed
            # Tốc độ quay bù trừ = P * Sai số góc
            move_msg.angular.z = self.kp_yaw * error_yaw 

            self.cmd_vel_pub.publish(move_msg)

            # Gửi feedback
            feedback_msg.current_distance = self.current_distance
            goal_handle.publish_feedback(feedback_msg)

            loop_rate.sleep()

        goal_handle.succeed()
        result.success = True
        result.final_distance = self.current_distance
        self.get_logger().info('Hành vi hoàn thành. Đã dừng đúng mép mục tiêu.')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    