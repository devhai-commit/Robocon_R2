#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
import time

from .waveshare_controlcanfd import WaveshareControlCANFD

class CANBridge(Node):

    def __init__(self):
        super().__init__('can_bridge')

        self.declare_parameter("so_path", "")
        self.declare_parameter("channel", 0)
        self.declare_parameter("device_index", 0)
        
        channel = self.get_parameter("channel").value
        self.device_index = int(self.get_parameter("device_index").value)

        so_path = self.get_parameter("so_path").value
        if not so_path:
            from ament_index_python.packages import get_package_share_directory
            import os
            share = get_package_share_directory("ak60_robot_driver_io")
            so_path = os.path.join(share, "vendor", "libcontrolcanfd.so")

        self.can = WaveshareControlCANFD(so_path=so_path, device_index=self.device_index)
        self.can.open(channel=channel,
                      abit_baud=1_000_000,
                      dbit_baud=5_000_000,
                      set_all_channels=True)

        self.get_logger().info("CAN opened OK")

        # QoS tối ưu cho lưu lượng cao
        qos_profile = rclpy.qos.QoSProfile(depth=1000)

        self.sub_tx = self.create_subscription(
            Frame,
            "/canfd/tx",
            self.tx_callback,
            qos_profile
        )

        self.pub_rx = self.create_publisher(
            Frame,
            "/canfd/rx",
            qos_profile
        )

        # Biến theo dõi vị trí Arm 1 (Khởi tạo ở một giá trị giả định lớn)
        self.arm_1_current_pos = 999.0

        self.timer = self.create_timer(0.01, self.rx_loop)

    def tx_callback(self, msg):
        self.can.send(msg.id, bytes(msg.data), eff=msg.is_extended)

    def rx_loop(self):
        frames = self.can.recv_all()
        if not frames:
            return

        for rx_id, data, ts, eff in frames:
            # 1. Bóc tách trực tiếp vị trí của Arm 1 (Motor ID = 3)
            # Theo cấu trúc AK60: Byte 0 (Cao) và Byte 1 (Thấp) chứa góc
            if rx_id == 3 and len(data) >= 2:
                pos_int = (data[0] << 8) | data[1]
                # Ép kiểu về số nguyên có dấu 16-bit (signed int16)
                if pos_int > 32767:
                    pos_int -= 65536
                # Độ phân giải của AK60 là 0.1 độ / bit
                self.arm_1_current_pos = pos_int * 0.1

            # 2. Đóng gói và Publish lên ROS 2
            msg = Frame()
            msg.id = rx_id
            msg.is_extended = bool(eff)
            msg.is_rtr = False
            msg.is_error = False
            msg.data = list(data)
            msg.dlc = len(msg.data)

            self.pub_rx.publish(msg)


def main():
    rclpy.init()
    node = CANBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Nhận lệnh Ctrl+C. Hệ thống bắt đầu tiến trình dừng an toàn...")
        
        # Cho phép Node duy trì thêm tối đa 3.5 giây để trung chuyển nốt lệnh từ C++
        # (Hardware Interface C++ mất khoảng 2.5 giây để thu arm về 0)
        start_time = time.time()
        timeout = 3.5 
        
        while time.time() - start_time < timeout:
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except Exception:
                # Nếu ROS 2 đã sập context, thoát vòng lặp ngay lập tức
                break 
            
            if abs(node.arm_1_current_pos) <= 2.0:
                print(f"[INFO] ✅ Arm 1 đã hạ về vị trí an toàn. Đóng cổng CAN.")
                break
        else:
            # Nếu hết 3.5 giây mà motor vẫn chưa về (do kẹt cơ khí, lỗi cáp...)
            print(f"[WARN] Hết thời gian chờ! Vị trí Arm 1 hiện tại: {node.arm_1_current_pos:.1f} độ. Bắt buộc ngắt kết nối.")
    finally:
        # 1. Đóng cổng CAN an toàn
        node.can.close()
        
        # 2. Hủy Node
        node.destroy_node()
        
        # 3. KHIÊN BẢO VỆ: Chỉ shutdown ROS nếu nó chưa bị tắt
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

