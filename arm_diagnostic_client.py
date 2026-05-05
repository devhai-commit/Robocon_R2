import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from ak60_bringup.action import ArmSequence
import time
import sys

class ArmDiagnosticClient(Node):
    def __init__(self):
        super().__init__('arm_diagnostic_client')
        
        # Action Client
        self._action_client = ActionClient(self, ArmSequence, 'arm_sequence_controller')
        
        # Subscriber lắng nghe vị trí liên tục từ Topic
        self.current_feedback_pos = [0.0] * 5
        self.state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.topic_callback, 
            10
        )
        self.get_logger().info("🩺 Diagnostic Client đã sẵn sàng!")

    def topic_callback(self, msg):
        self.current_feedback_pos = list(msg.position)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        sys.stdout.write(f"\r  -> {int(fb.progress_percent)}% | Vị trí hiện tại: {['%.1f' % p for p in fb.current_pos]}")
        sys.stdout.flush()

    def test_single_joint(self, joint_idx, control_mode, target_val, duration=2.0):
        self._action_client.wait_for_server()

        self.get_logger().info("⏳ Đang đồng bộ vị trí thực tế...")
        start_wait = time.time()
        # Đợi nhận data thật từ Robot (Mảng khác [0,0,0,0,0])
        while rclpy.ok() and all(p == 0.0 for p in self.current_feedback_pos):
            if time.time() - start_wait > 3.0:
                self.get_logger().error("❌ Lỗi: Không nhận được dữ liệu từ Topic /joint_states.")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_msg = ArmSequence.Goal()
        goal_msg.pose_name = "custom"
        goal_msg.duration = float(duration)
        
        # Chốt vị trí an toàn cho các khớp còn lại
        current_pos = list(self.current_feedback_pos)
        
        if current_pos[0] == 0.0:
            self.get_logger().warn("⚠️ Cảnh báo: J1 đang là 0.0. Nếu robot đang đứng ở 180, nó sẽ quay về 0!")

        safe_pos = [float(x) for x in (current_pos + [0.0]*5)[:5]]
        safe_eff = [0.0] * 5
        
        if control_mode == 'pos':
            safe_pos[joint_idx] = float(target_val)
        else:
            safe_eff[joint_idx] = float(target_val)
            
        goal_msg.target_pos = safe_pos
        goal_msg.target_eff = safe_eff

        self.get_logger().info(f'==> Gửi lệnh Test J{joint_idx+1} (Dựa trên vị trí J1 hiện tại: {safe_pos[0]}°)')
        
        # ==========================================================
        # CƠ CHẾ ĐỒNG BỘ CHUẨN ROS 2 (THAY THẾ CALLBACK)
        # ==========================================================
        
        # Bước 1: Gửi lệnh và bắt Client đợi Server trả lời (Chấp nhận/Từ chối)
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('\n❌ Lệnh bị Server từ chối!')
            return False

        # Bước 2: Bắt Client đợi Server chạy xong cơ khí và trả kết quả cuối cùng
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        print("\n") # Xuống dòng để không đè lên thanh %
        
        if result.success:
            self.get_logger().info('✅ Hoàn thành nhịp chạy thành công.')
        else:
            self.get_logger().error('❌ Nhịp chạy bị thất bại hoặc hết thời gian chờ.')
            
        return True

def main():
    rclpy.init()
    action_client = ArmDiagnosticClient()

    print("\n" + "="*50)
    print("      CHƯƠNG TRÌNH TEST KHỚP CÁNH TAY (BẢN ỔN ĐỊNH)")
    print("="*50)
    
    try:
        while rclpy.ok():
            print("\nChọn khớp (1-5) hoặc 0 để thoát:")
            try:
                choice = int(input("Khớp: "))
                if choice == 0: break
                if not (1 <= choice <= 5): continue
                
                idx = choice - 1
                if choice in [1, 4, 5]:
                    val = float(input(f"Nhập góc mục tiêu cho J{choice} (Độ): "))
                    dur = float(input("Thời gian chạy (giây): "))
                    action_client.test_single_joint(idx, 'pos', val, dur)
                else:
                    val = float(input(f"Nhập công suất cho J{choice} (-1.0 đến 1.0): "))
                    dur = float(input("Thời gian chạy (giây): "))
                    action_client.test_single_joint(idx, 'eff', val, dur)
                    
            except ValueError:
                print("Vui lòng nhập số hợp lệ!")
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    