#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import torch
from ultralytics import YOLO

class YoloTensorRTNode(Node):
    def __init__(self):
        super().__init__('yolo_tensorrt_node')
        
        # 1. Nạp vũ khí hạng nặng TensorRT
        self.get_logger().info("Đang nạp lõi YOLOv8 TensorRT...")
        # Lưu ý: Thay đổi đường dẫn nếu file .engine của bạn nằm ở chỗ khác
        self.model = YOLO('/home/robocon/ros_ws/yolo26n_best.engine', task='detect') 
        
        # 2. Tạo kênh phát sóng (Publisher)
        self.publisher_ = self.create_publisher(Point, '/target_coordinates', 10)
        
        # 3. Mở con mắt của robot (Camera)
        self.cap = cv2.VideoCapture(4, cv2.CAP_V4L2) # Force dùng driver V4L2 của Linux
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        if not self.cap.isOpened():
            self.get_logger().error("CẢNH BÁO: Không thể kết nối với Camera!")
            return
            
        # 4. Tạo nhịp đập cho Node (chạy 30 lần 1 giây ~ 30 FPS)
        timer_period = 0.001  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Node YOLO TensorRT ĐÃ SẴN SÀNG! Đang quét mục tiêu...")

    def timer_callback(self):
        # Đọc 1 khung hình từ Camera
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # Đẩy khung hình vào cho GPU xử lý
        results = self.model(frame, verbose=False)
        
        # ========================================================
        # THÊM PHẦN HIỂN THỊ HÌNH ẢNH OPENCV
        # Hàm plot() tự động vẽ bounding box, tên class và độ tin cậy
        annotated_frame = results[0].plot()
        
        # Hiển thị lên cửa sổ có tên "YOLOv8 Detection"
        cv2.imshow("YOLOv8 Detection", annotated_frame)
        
        # Hàm waitKey(1) bắt buộc phải có để OpenCV cập nhật cửa sổ (đợi 1ms)
        cv2.waitKey(1) 
        # ========================================================

        # Phân tích kết quả trả về để gửi lên ROS 2
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Lấy tọa độ 4 góc của khung chứa vật thể
                b = box.xyxy[0].cpu().numpy() 
                
                # Tính TỌA ĐỘ TÂM của vật thể
                center_x = float((b[0] + b[2]) / 2.0)
                center_y = float((b[1] + b[3]) / 2.0)
                
                # Lấy ID của loại vật thể (Ví dụ: 0=Người, 39=Cái chai...)
                class_id = float(box.cls[0].cpu().numpy())
                
                # Đóng gói dữ liệu để gửi đi
                msg = Point()
                msg.x = center_x
                msg.y = center_y
                msg.z = class_id # Mẹo nhỏ: Mượn tạm trục Z để chứa ID vật thể
                
                # Gửi lên mạng lưới ROS 2
                self.publisher_.publish(msg)
                
                # In ra Terminal cho bạn dễ theo dõi
                self.get_logger().info(f"Đã thấy mục tiêu! Tâm X: {center_x:.1f}, Y: {center_y:.1f} | Class ID: {int(class_id)}")
                
                # (Tùy chọn) Nếu có nhiều vật, ở đây chỉ lấy vật đầu tiên tìm thấy rồi bỏ qua các vật khác
                break 

def main(args=None):
    rclpy.init(args=args)
    node = YoloTensorRTNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Nhớ tắt camera cẩn thận trước khi thoát
        node.cap.release()
        
        # Đóng tất cả cửa sổ OpenCV khi tắt Node bằng Ctrl+C
        cv2.destroyAllWindows() 
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    