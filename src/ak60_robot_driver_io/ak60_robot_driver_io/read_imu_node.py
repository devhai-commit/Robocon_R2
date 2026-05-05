import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math

class ImuBridgeNode(Node):
    def __init__(self):
        super().__init__('imu_bridge_node')

        self.declare_parameter('port', '/dev/ttyIMU')
        self.port_name = self.get_parameter('port').get_parameter_value().string_value
        self.get_logger().info(f'Đang kết nối tới cổng: {self.port_name}')

        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        try:
            # Sử dụng self.port_name thay vì viết cứng "/dev/ttyUSB0"
            self.ser = serial.Serial(self.port_name, 115200, timeout=1)
        except Exception as e:
            self.get_logger().error(f'Không thể mở cổng {self.port_name}: {e}')
            exit()

        self.get_logger().info('DA KET NOI IMU_bno055!')

        self.timer = self.create_timer(0.004, self.timer_callback) # 250Hz

        # Định nghĩa các giá trị hiệp phương sai (thay đổi dựa trên thực tế)
        # Các giá trị này càng nhỏ nghĩa là cảm biến càng chính xác
        self.ori_cov = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        self.ang_cov = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
        self.acc_cov = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                
                data = [float(x) for x in line.split(',')]
                
                if len(data) == 10:
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'imu_link'

                    # 1. QUATERNIONS (Cột 0 -> 3)
                    w, x, y, z = data[0], data[1], data[2], data[3]
                    msg.orientation.w = w
                    msg.orientation.x = x
                    msg.orientation.y = y
                    msg.orientation.z = z
                    msg.orientation_covariance = self.ori_cov

                    # Angular Velocity (rad/s)
                    msg.angular_velocity.x = data[4] * (math.pi / 180.0)
                    msg.angular_velocity.y = data[5] * (math.pi / 180.0)
                    msg.angular_velocity.z = data[6] * (math.pi / 180.0)
                    msg.angular_velocity_covariance = self.ang_cov

                    # Linear Acceleration (m/s^2)
                    # 3. TÁI TẠO GIA TỐC TUYẾN TÍNH TỪ QUATERNIONS
                    # Biến đổi vector trọng lực thế giới (0, 0, 1) vào hệ tọa độ của IMU
                    # ========================================================
                    g_x = 2.0 * (x * z - w * y)
                    g_y = 2.0 * (w * x + y * z)
                    g_z = (w * w) - (x * x) - (y * y) + (z * z)

                    # Nhân với 9.81 để ra đúng chuẩn m/s^2 của ROS 2
                    msg.linear_acceleration.x = g_x * 9.81
                    msg.linear_acceleration.y = g_y * 9.81
                    msg.linear_acceleration.z = g_z * 9.81
                    msg.linear_acceleration_covariance = self.acc_cov

                    self.publisher_.publish(msg)
            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridgeNode() # Giữ nguyên tên class của bạn
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Khi bạn nhấn Ctrl+C, chương trình sẽ nhảy vào đây thay vì báo lỗi đỏ
        node.get_logger().info("Nhận lệnh Ctrl+C, đang tắt IMU Node...")
    finally:
        # Đảm bảo dọn dẹp sạch sẽ tài nguyên dù tắt bằng cách nào
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
