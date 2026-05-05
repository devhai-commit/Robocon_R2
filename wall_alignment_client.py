#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Nhớ thay thế 'my_robot_interfaces' bằng tên package của bạn
from ak60_bringup.action import WallAlignment

class WallAlignmentClient(Node):
    def __init__(self):
        super().__init__('wall_alignment_client')
        
        # Khởi tạo Action Client kết nối đến Action có tên 'align_and_approach_wall'
        self._action_client = ActionClient(self, WallAlignment, 'align_and_approach_wall')

    def send_goal(self, window_deg, target_dist):
        self.get_logger().info('Đang chờ Action Server kết nối...')
        self._action_client.wait_for_server()

        # Tạo đối tượng Goal
        goal_msg = WallAlignment.Goal()
        goal_msg.window_degrees = float(window_deg)
        goal_msg.goal_distance = float(target_dist)

        self.get_logger().info(f'Đã gửi Lệnh: Quét góc {window_deg}°, dừng cách tường {target_dist}m')

        # Gửi Goal đi (Bất đồng bộ) và khai báo hàm xử lý Feedback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        # Khai báo hàm xử lý khi Server trả lời là có CHẤP NHẬN lệnh hay không
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Hàm này chạy liên tục mỗi khi Server gửi thông tin cập nhật
        feedback = feedback_msg.feedback
        phase = feedback.current_phase
        dist = feedback.current_distance
        dev = feedback.current_deviation
        
        # In ra màn hình để theo dõi robot đang làm gì
        self.get_logger().info(f'[{phase}] Cách tường: {dist:.3f}m | Lệch: {dev:.2f}°')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Lệnh đã bị Server TỪ CHỐI!')
            return

        self.get_logger().info('Server đã CHẤP NHẬN lệnh. Đang thực thi...')

        # Chờ kết quả cuối cùng
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Hàm này chạy khi Action Server báo hoàn thành (goal_handle.succeed())
        result = future.result().result
        status = future.result().status

        if result.success:
            self.get_logger().info('=====================================')
            self.get_logger().info('🚀 NHIỆM VỤ HOÀN THÀNH XUẤT SẮC!')
            self.get_logger().info(f'Khoảng cách chốt: {result.final_distance:.3f} m')
            self.get_logger().info(f'Góc lệch chốt: {result.final_deviation:.2f} độ')
            self.get_logger().info('=====================================')
        else:
            self.get_logger().warn('Nhiệm vụ thất bại hoặc bị hủy.')

        # Tắt Node sau khi xong việc
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    action_client = WallAlignmentClient()
    
    # Kịch bản Test:
    # Cửa sổ quét: 15 độ (để lấy cụm tia laser phía trước)
    # Khoảng cách muốn dừng: 0.4 mét (40 cm)
    action_client.send_goal(window_deg=15.0, target_dist=0.4)
    
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
