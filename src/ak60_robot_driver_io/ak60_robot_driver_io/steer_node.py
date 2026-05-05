#!/usr/bin/env python3
from __future__ import annotations

import rclpy, time, threading, serial
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Int32MultiArray

from .servo_driver import HiwonderBusServo, WaveShareCF35

def _pick_serial_port(preferred: str) -> str:
    import glob, os
    if preferred and os.path.exists(preferred): return preferred
    candidates = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyTHS*"))
    return candidates[0] if candidates else preferred

class SteerNode(Node):
    def __init__(self):
        super().__init__('steer_node')

        self.declare_parameter("port", "/dev/ttyServo")
        self.declare_parameter("servo_ids", [1, 2, 3, 4, 5, 6, 7, 8])
        self.declare_parameter("centers", [500, 500, 500, 500, 500, 500, 375, 500])
        self.declare_parameter("invert", [False]*8)
        self.declare_parameter("move_time_ms", 80)

        self.port = str(self.get_parameter("port").value)
        self.port = _pick_serial_port(self.port)

        self.ids = list(self.get_parameter("servo_ids").value)
        self.centers = list(self.get_parameter("centers").value)
        invert = list(self.get_parameter("invert").value)
        move_time = int(self.get_parameter("move_time_ms").value)

        self.serial_lock = threading.Lock()
        self.ser = None
        
        # Bắt đầu thử kết nối
        self.connect_serial()

        # Khởi tạo Driver phần cứng
        self.servo = HiwonderBusServo(
            serial_port=self.ser, servo_ids=self.ids, centers=self.centers,
            invert=invert, default_move_time_ms=move_time
        )
        self.gripper = WaveShareCF35(self.ser)

        # --- TỐI ƯU HÓA QoS ---
        # Chỉ giữ lại 1 bản tin mới nhất cho lệnh điều khiển (Chống delay/kẹt xe)
        cmd_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Khởi tạo Publisher (Feedback giữ depth=10 là an toàn)
        self.pub_servo_fb = self.create_publisher(Int32MultiArray, "/servo/fb_count", 10)
        self.pub_gripper_fb = self.create_publisher(Int32MultiArray, "/gripper/feedback", 10)

        # Khởi tạo Subscriber (Dùng cmd_qos siêu tốc)
        self.sub_cmd = self.create_subscription(Int32MultiArray, "/servo/cmd_count", self.cmd_callback, cmd_qos)
        self.gripper_sub = self.create_subscription(Int32MultiArray, 'gripper/cmd_pos', self.gripper_callback, cmd_qos)
        
        # Timer đọc phản hồi (Chạy ở 25Hz = 0.04s, khớp hoàn toàn với C++ prescaler)
        self.timer = self.create_timer(0.04, self.read_feedback)

        # --- Khởi tạo và đưa về tâm lúc bật Node ---
        with self.serial_lock:
            # 1. Bật Gripper (1,000,000)
            self.switch_baudrate(1000000)
            self.gripper.EnableTorque(1, True)
            self.gripper.WritePosEx(1, 2648, 60, 50, 500)
            
            # 2. Bật Servo Hiwonder (115,200)
            self.switch_baudrate(115200)
            self.servo.load_all()
            for i, count in enumerate(self.centers):
                if i <= 5:
                    self.servo.move_single(self.ids[i], int(count))
                else:
                    self.servo.move_single(self.ids[i], int(count), move_time=1500)    

    def connect_serial(self):
        while self.ser is None or not self.ser.is_open:
            try:
                self.get_logger().info(f"Đang thử kết nối tới {self.port}...")
                self.ser = serial.Serial(self.port, 115200, timeout=0.05)
                time.sleep(0.5) 
                self.get_logger().info(f"✅ MỞ SERIAL THÀNH CÔNG: {self.port}!")
            except Exception as e:
                self.get_logger().error(f"❌ Không thể mở {self.port}. Thử lại sau 2 giây... ({e})")
                time.sleep(2)

    def switch_baudrate(self, target_baud):
        if self.ser.baudrate != target_baud:
            self.ser.baudrate = target_baud
            time.sleep(0.002) # Chờ IC USB chuyển tần số
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer() # [TỐI ƯU HÓA] Xóa rác chiều gửi để chống nhiễu lệnh

    def cmd_callback(self, msg):
        with self.serial_lock:
            self.switch_baudrate(115200)
            for i, count in enumerate(msg.data):
                if i < len(self.ids):
                    if i <= 5 or i == 6:
                        self.servo.move_single(self.ids[i], int(count))
                    else:
                        self.servo.move_single(self.ids[i], int(count), move_time=500)
                        
    def gripper_callback(self, msg):
        if len(msg.data) > 0:
            with self.serial_lock:
                self.switch_baudrate(1000000)
                self.gripper.WritePosEx(1, msg.data[0], 60, 50, 500)

    def read_feedback(self):
        servo_fb = None
        g_pos, g_cur = None, None

        with self.serial_lock:
            # 1. ĐỌC HIWONDER
            self.switch_baudrate(115200)
            try:
                servo_fb = self.servo.read_position() 
            except Exception as e:
                self.get_logger().error(f"⚠️ Lỗi đọc Hiwonder: {e}")

            # 2. ĐỌC WAVESHARE
            self.switch_baudrate(1000000)
            try:
                g_pos = self.gripper.ReadPosition(1)
                g_cur = self.gripper.ReadCurrent(1)
            except Exception as e:
                self.get_logger().error(f"⚠️ Lỗi đọc Gripper Waveshare: {e}")

        # --- PUBLISH DỮ LIỆU ---
        if rclpy.ok():
            if servo_fb:
                s_msg = Int32MultiArray()
                s_msg.data = [int(x) if x is not None else -1 for x in servo_fb]
                self.pub_servo_fb.publish(s_msg)

            if g_pos is not None or g_cur is not None:
                g_msg = Int32MultiArray()
                g_msg.data = [
                    int(g_pos) if g_pos is not None else -1,
                    int(g_cur) if g_cur is not None else -1
                ]
                self.pub_gripper_fb.publish(g_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SteerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nhận lệnh Ctrl+C. Đang ngắt lực động cơ...")
        try:
            with node.serial_lock:
                node.switch_baudrate(115200)
                node.servo.unload_all()
                time.sleep(0.1) 
                
                node.switch_baudrate(1000000)
                node.gripper.EnableTorque(1, False)
                time.sleep(0.1)
                
                node.ser.close()
                node.get_logger().info("✅ Đã nhả lực 8 Servo và Gripper. Đóng cổng Serial thành công!")
        except Exception as e:
            node.get_logger().error(f"❌ Lỗi tắt phần cứng: {e}")
            
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
    