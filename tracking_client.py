#!/usr/bin/env python3
import sys
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import đúng package action của bạn
from ak60_bringup.action import FollowTarget

class TrackingClient(Node):
    def __init__(self):
        super().__init__('tracking_standalone_client')
        
        # Khởi tạo Action Client trỏ tới server '/follow_target'
        self._action_client = ActionClient(self, FollowTarget, '/follow_target')

    def send_goal(self, target_id, desired_distance_mm):
        self.get_logger().info('Đang chờ Action Server /follow_target kết nối...')
        self._action_client.wait_for_server()

        # Tạo gói tin Goal
        goal_msg = FollowTarget.Goal()
        goal_msg.target_id = float(target_id)
        goal_msg.desired_distance_mm = float(desired_distance_mm)

        self.get_logger().info(f'🚀 Bắt đầu gửi lệnh: Bám ID {target_id}, Cách {desired_distance_mm}mm...')
        
        # Gửi Goal bất đồng bộ và gắn callback để nhận Feedback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
            
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Lệnh bị Server TỪ CHỐI!')
            rclpy.shutdown()
            return

        self.get_logger().info('✅ Lệnh đã được CHẤP NHẬN, robot đang chạy...')
        # Chờ kết quả cuối cùng
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if result.success:
            self.get_logger().info('🎉 HOÀN THÀNH: Robot đã căn chỉnh tâm và khoảng cách chuẩn xác!')
        else:
            self.get_logger().warn(f'⚠️ THẤT BẠI HOẶC BỊ HỦY (Mã trạng thái: {status})')
            
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # In Feedback ra màn hình liên tục để theo dõi PID
        self.get_logger().info(
            f'⏳ [AI VISION] Lệch ngang (X): {feedback.error_x:6.1f} px | Lệch khoảng cách: {feedback.error_distance_mm:6.1f} mm'
        )

def main(args=None):
    rclpy.init(args=args)
    
    # Thiết lập bộ đọc tham số từ Terminal
    parser = argparse.ArgumentParser(description='Tool test nhanh AI Tracking Client')
    parser.add_argument('-i', '--id', type=float, default=6.0, 
                        help='ID của vật thể cần bám (Mặc định: 0.0 - Người)')
    parser.add_argument('-d', '--dist', type=float, default=360.0, 
                        help='Khoảng cách mong muốn (mm) (Mặc định: 500.0)')
    
    # Tách tham số của tool ra khỏi tham số mặc định của ROS 2
    parsed_args, ros_args = parser.parse_known_args()
    
    action_client = TrackingClient()
    
    # Gửi lệnh đi với tham số truyền vào
    action_client.send_goal(parsed_args.id, parsed_args.dist)
    
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info("🛑 Đã nhận lệnh HỦY bằng Ctrl+C. Đang ngắt kết nối...")
    finally:
        action_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
