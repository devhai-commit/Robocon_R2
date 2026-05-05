import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthReader(Node):
    def __init__(self):
        super().__init__('depth_reader_node')
        # Subscribe vào topic depth
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Chuyển đổi dữ liệu thô sang mảng 2D (định dạng 16-bit)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Lấy kích thước ảnh (hiện tại là 320x240)
            height, width = cv_image.shape
            
            # Lấy giá trị khoảng cách ở ngay tâm điểm của ảnh (tính bằng mm)
            center_dist = cv_image[int(height/2), int(width/2)]
            
            # Orbbec thường trả về 0 nếu điểm đó bị nhiễu hoặc không đo được
            if center_dist > 0:
                self.get_logger().info(f'Khoảng cách vật cản ở mũi robot: {center_dist} mm')
            else:
                self.get_logger().info('Vùng mù hoặc ngoài tầm đo!')
                
        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý ảnh: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    