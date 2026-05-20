import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import time

class KalmanFilter1D:
    def __init__(self, process_noise, measurement_noise):
        """
        process_noise (Q): Phương sai nhiễu hệ thống (mức độ tin tưởng vào mô hình).
        measurement_noise (R): Phương sai nhiễu đo lường (mức độ tin tưởng vào cảm biến).
        """
        self.q = process_noise
        self.r = measurement_noise
        self.x = None  # Trạng thái ước lượng hiện tại (khoảng cách)
        self.p = 1.0   # Sai số ước lượng

    def update(self, measurement):
        # Khởi tạo giá trị bằng ngay phép đo đầu tiên để tránh bị ramp-up từ 0
        if self.x is None:
            self.x = measurement
            return self.x

        # 1. Prediction (Dự đoán)
        p_predict = self.p + self.q

        # 2. Update (Cập nhật)
        k = p_predict / (p_predict + self.r) # Hệ số Kalman Gain
        self.x = self.x + k * (measurement - self.x)
        self.p = (1 - k) * p_predict

        return self.x

class LaserDistancePublisher(Node):
    def __init__(self):
        super().__init__('laser_distance_publisher')
        
        # Khai báo các tham số ROS 2 cho phép tinh chỉnh (tuning) trực tiếp khi node đang chạy
        self.declare_parameter('kalman_q', 0.01)
        self.declare_parameter('kalman_r', 0.1)
        
        q_val = self.get_parameter('kalman_q').value
        r_val = self.get_parameter('kalman_r').value
        
        # Khởi tạo bộ lọc Kalman
        self.kf = KalmanFilter1D(process_noise=q_val, measurement_noise=r_val)
        self.get_logger().info(f'Khởi tạo Kalman Filter (Q={q_val}, R={r_val})')

        # Khởi tạo Publisher
        self.publisher_ = self.create_publisher(Range, 'laser_distance', 10)
        
        # Cấu hình Serial
        self.serial_port = '/dev/ttyLaser'
        self.baud_rate = 38400
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.5)
            self.get_logger().info(f'Đã mở cổng {self.serial_port}')
            
            # Khởi động chế độ đo nhanh
            self.ser.write(b'iHAL\r\n')
            time.sleep(0.2)
            self.ser.flushInput()
            self.ser.write(b'iFACM\r\n')
            
            # Dùng Timer để đọc liên tục
            self.timer = self.create_timer(0.05, self.read_and_publish) # ~20Hz
            
        except serial.SerialException as e:
            self.get_logger().error(f'Lỗi Serial: {e}')

    def read_and_publish(self):
        if hasattr(self, 'ser') and self.ser.in_waiting > 0:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            
            # Parse chuỗi "D=0.578m"
            if line.startswith('D=') and line.endswith('m'):
                try:
                    # Lấy giá trị thô
                    distance_str = line[2:-1] 
                    raw_distance_m = float(distance_str)
                    
                    # Cập nhật giá trị qua bộ lọc Kalman
                    filtered_distance_m = self.kf.update(raw_distance_m)
                    
                    # Đóng gói vào Message Range
                    msg = Range()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "laser_link"
                    msg.radiation_type = Range.INFRARED
                    msg.field_of_view = 0.01 
                    msg.min_range = 0.05
                    msg.max_range = 40.0
                    
                    # Publish giá trị ĐÃ LỌC
                    msg.range = filtered_distance_m
                    
                    self.publisher_.publish(msg)
                    
                except ValueError:
                    self.get_logger().warn(f'Không thể parse giá trị: {line}')

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.write(b'iHAL\r\n') 
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaserDistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    