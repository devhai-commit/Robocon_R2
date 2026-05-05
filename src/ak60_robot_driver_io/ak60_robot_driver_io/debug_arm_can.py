import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
import sys

class ArmDebugger(Node):
    def __init__(self):
        super().__init__('arm_debugger_node')
        # Gửi thẳng lệnh xuống mạch Waveshare CAN
        self.pub = self.create_publisher(Frame, '/canfd/tx', 10)
        self.get_logger().info("Đã khởi động Node Debug Arm 1 (AK60 V1 - ID 3)")

        # Thay vì chỉ pub.publish(frame) 1 lần, hãy dùng timer để gửi liên tục ở tần số 50Hz (0.02s)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.frame = Frame()

    def send_pos_spd_command(self, pos_deg, spd_erpm=12000.0, acc_erpm=40000.0):
        motor_id = 3
        cmd_id = 6  # CAN_PACKET_SET_POS_SPD

        # ========================================================
        # KHU VỰC CHUẨN ĐOÁN LỖI TỈ LỆ
        # Nếu quay 1 độ -> Sửa MULTIPLIER = 1000000.0
        # Nếu quay 16 độ -> Giữ 10000.0 nhưng sửa GEAR_RATIO = 6.0
        # ========================================================
        MULTIPLIER = 10000.0  # Đã đổi từ 1 vạn thành 1 triệu!
        GEAR_RATIO = 1.0        # Nếu đổi Multiplier mà vẫn thiếu, đổi số này thành 6.0
        
        real_pos_deg = pos_deg * GEAR_RATIO

        # 1. Ép kiểu và Quy đổi theo chuẩn Big-Endian
        pos_int = int(real_pos_deg * MULTIPLIER)
        spd_int = int(spd_erpm / 10.0)  # Tốc độ tăng lên 12000
        acc_int = int(acc_erpm / 10.0)  # Gia tốc tăng lên 40000

        frame = Frame()
        frame.id = motor_id | (cmd_id << 8)
        frame.is_extended = True
        frame.dlc = 8

        # 3. Cắt bit đóng gói
        frame.data = [
            (pos_int >> 24) & 0xFF,
            (pos_int >> 16) & 0xFF,
            (pos_int >> 8) & 0xFF,
            pos_int & 0xFF,
            (spd_int >> 8) & 0xFF,
            spd_int & 0xFF,
            (acc_int >> 8) & 0xFF,
            acc_int & 0xFF
        ]

        self.current_frame = frame

    def timer_callback(self):
        # Đóng gói và gửi frame CAN liên tục chứa góc mục tiêu hiện tại
        self.pub.publish(self.current_frame)


def main(args=None):
    rclpy.init(args=args)
    node = ArmDebugger()

    print("=========================================")
    print(" TOOL DEBUG CÁNH TAY AK60 V1 (ID 3) ")
    print("=========================================")
    print("Nhập một góc (Độ) để cánh tay di chuyển.")
    print("Gõ 'q' để thoát.")
    print("=========================================")

    try:
        while rclpy.ok():
            # user_input = input("\nNhập góc quay (vd: 90, -45, 0): ")
            
            # if user_input.lower() == 'q':
            #     break
            pos_deg = 360    
            try:
                # pos_deg = float(user_input)
                # Bạn có thể đổi tốc độ và gia tốc ở đây nếu muốn test lực
                node.send_pos_spd_command(pos_deg, spd_erpm=15000.0, acc_erpm=15000.0)
                rclpy.spin_once(node, timeout_sec=0.01)
            except ValueError:
                print("Lỗi: Vui lòng nhập một con số hợp lệ!")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    