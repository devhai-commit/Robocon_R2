import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys
import time


from ak60_bringup.action import ArmSequence 


class ArmTestClient(Node):
    def __init__(self):
        super().__init__('arm_test_client')
        self._action_client = ActionClient(self, ArmSequence, 'arm_sequence_controller')

    def send_goal(self, pose_name, duration=1.5):
        self._action_client.wait_for_server()

        goal_msg = ArmSequence.Goal()
        goal_msg.pose_name = pose_name
        goal_msg.duration = duration

        self.get_logger().info(f'==> Đang gửi lệnh: [{pose_name}] (Thời gian: {duration}s)...')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, self._send_goal_future)

        goal_handle = self._send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Lệnh bị Server từ chối!')
            return False

        self._get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._get_result_future)

        result = self._get_result_future.result().result
        print("\n") 
        if result.success:
            self.get_logger().info(f'[THÀNH CÔNG] Hoàn thành {pose_name}')
            return True
        else:
            self.get_logger().error(f'[THẤT BẠI] Lỗi tại {pose_name}')
            return False

    def feedback_callback(self, feedback_msg):
        percent = feedback_msg.feedback.progress_percent
        sys.stdout.write(f'\r  -> Tiến độ: {percent:.1f}% ')
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    action_client = ArmTestClient()

    # Danh sách thứ tự chuẩn để chạy Auto Macro
    macro_sequence = [
        ("home_pose_right", 1.5),
        ("approach_j4_right", 1.0),
        ("approach_j3_right", 1.5),
        ("grasp_right", 1.0),
        ("move_back_J4_right", 1.5),
        ("move_back_J1_right", 2.0),
        ("release_right", 1.0)
    ]

    try:
        while rclpy.ok():
            print("\n" + "="*50)
            print("🚀 BẢNG ĐIỀU KHIỂN ROBOT (WAYPOINT TESTER)")
            print("="*50)
            print("--- BƯỚC KHỞI ĐỘNG ---")
            print("  1. home_pose    (Gốc)")
            print("--- CHUỖI TIẾN VÀO GẮP ---")
            print("  2. approach_j4  (Hạ J4)")
            print("  3. approach_j3  (Đẩy J3 ra)")
            print("  4. grasp        (Đóng kẹp)")
            print("--- CHUỖI RÚT VỀ ---")
            print("  5. move_back_J4 (Nâng J4)")
            print("  6. move_back_J1 (Xoay J1 về)")
            print("  7. release     (Mở kẹp)")
            print("-" * 50)
            print("  8. 🤖 CHẠY TỰ ĐỘNG TOÀN BỘ CHU TRÌNH TRÊN")
            print("  0. Thoát")
            print("="*50)

            choice = input("Chọn số (0-11) và nhấn Enter: ").strip()

            if choice == '0':
                break
                
            elif choice == '8':
                print("\n[MACRO] Bắt đầu chạy liên hoàn...")
                for pose_name, duration in macro_sequence:
                    success = action_client.send_goal(pose_name, duration)
                    if not success:
                        print(f"\n[MACRO] Chu trình bị dừng khẩn cấp tại: {pose_name}")
                        break
                    time.sleep(0.5) # Nghỉ nửa giây giữa các nhịp cho cơ khí ổn định
                print("\n[MACRO] KẾT THÚC CHU TRÌNH.")

            else:
                # Mapping các lựa chọn đơn lẻ
                options = {
                    '1': 'home_pose_right', 
                    '2': 'approach_j4_right',
                    '3': 'approach_j3_right', 
                    '4': 'grasp_right', 
                    '5': 'move_back_J4_right',
                    '6': 'move_back_J1_right', 
                    '7': 'release_right'
                }
                
                if choice in options:
                    action_client.send_goal(options[choice])
                else:
                    print("⚠️ Lựa chọn không hợp lệ.")
                
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    