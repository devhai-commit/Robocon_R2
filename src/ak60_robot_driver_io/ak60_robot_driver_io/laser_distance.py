import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import time

class LaserDistancePublisher(Node):
    def __init__(self):
        super().__init__('laser_distance_publisher')
        # Khởi tạo Publisher
        self.publisher_ = self.create_publisher(Range, 'laser_distance', 10)
        
        # Cấu hình Serial (đúng theo test thực tế của bạn)
        self.serial_port = '/dev/ttyLaser' # Cổng nối với cảm biến laser
        self.baud_rate = 38400
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.5)
            self.get_logger().info(f'Đã mở cổng {self.serial_port}')
            
            # Khởi động chế độ đo nhanh
            self.ser.write(b'iHAL\r\n')
            time.sleep(0.2)
            self.ser.flushInput()
            self.ser.write(b'iFACM\r\n')
            
            # Dùng Timer để đọc liên tục không block main thread
            self.timer = self.create_timer(0.05, self.read_and_publish) # ~20Hz
            
        except serial.SerialException as e:
            self.get_logger().error(f'Lỗi Serial: {e}')

    def read_and_publish(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            
            # Parse chuỗi "D=0.578m"
            if line.startswith('D=') and line.endswith('m'):
                try:
                    # Cắt bỏ "D=" ở đầu và "m" ở cuối để lấy số thực
                    distance_str = line[2:-1] 
                    distance_m = float(distance_str)
                    
                    # Đóng gói vào Message Range
                    msg = Range()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "laser_link"
                    msg.radiation_type = Range.INFRARED # Hoặc tùy chọn phù hợp
                    msg.field_of_view = 0.01 # Góc chiếu laser hẹp
                    msg.min_range = 0.05
                    msg.max_range = 40.0
                    msg.range = distance_m
                    
                    self.publisher_.publish(msg)
                    
                except ValueError:
                    self.get_logger().warn(f'Không thể parse giá trị: {line}')

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.write(b'iHAL\r\n') # Dừng laser khi tắt node
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
