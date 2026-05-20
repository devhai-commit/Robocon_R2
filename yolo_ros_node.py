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
        self.model = YOLO('/home/robocon/ros_ws/weapon26n_best.engine', task='detect') 
        
        # 2. Tạo kênh phát sóng (Publisher)
        self.publisher_ = self.create_publisher(Point, '/target_coordinates', 10)
        
        # 3. Mở con mắt của robot (Camera)
        self.cap = cv2.VideoCapture(6, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        if not self.cap.isOpened():
            self.get_logger().error("CẢNH BÁO: Không thể kết nối với Camera!")
            return
            
        # 4. Tạo nhịp đập cho Node
        timer_period = 0.001  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Node YOLO TensorRT ĐÃ SẴN SÀNG! Đang quét mục tiêu...")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # ========================================================
        # THÊM BƯỚC 1: CROP KHUNG HÌNH
        # Tùy chỉnh tọa độ này theo vùng làm việc thực tế của camera
        x_min, x_max = 170, 470  # Cắt hai bên
        y_min, y_max = 300, 480   # Cắt trên dưới
        
        # Trích xuất vùng quan tâm (Region of Interest)
        cropped_frame = frame[y_min:y_max, x_min:x_max]
        # ========================================================

        # Chuyển từ BGR sang RGB cho YOLOv8
        rgb_frame = cropped_frame[:, :, ::-1]  
        
        # Đẩy khung hình đã cắt vào GPU xử lý
        results = self.model(rgb_frame, verbose=False)
        
        # Hiển thị hình ảnh OPENCV (Hiển thị trên khung hình đã crop)
        annotated_frame = results[0].plot()
        # Chuyển lại từ RGB về BGR để OpenCV hiển thị đúng màu
        cv2.imshow("YOLOv8 Detection (Cropped)", annotated_frame[:, :, ::-1])
        cv2.waitKey(1) 

        # ========================================================
        # THÊM BƯỚC 2 & 3: TÌM VẬT THỂ LỚN NHẤT
        largest_box = None
        max_area = 0.0

        for r in results:
            for box in r.boxes:
                b = box.xyxy[0].cpu().numpy() 
                
                # Tính Width và Height
                w = b[2] - b[0]
                h = b[3] - b[1]
                area = w * h
                
                # Cập nhật nếu tìm thấy box lớn hơn
                if area > max_area:
                    max_area = area
                    largest_box = box
                    
        # Nếu tìm thấy ít nhất 1 mục tiêu
        if largest_box is not None:
            # Lấy tọa độ của vật thể to nhất
            b = largest_box.xyxy[0].cpu().numpy()
            class_id = float(largest_box.cls[0].cpu().numpy())
            
            # Tính TỌA ĐỘ TÂM trên khung hình đã CROP
            center_x_crop = float((b[0] + b[2]) / 2.0)
            center_y_crop = float((b[1] + b[3]) / 2.0)
            
            # BÙ TRỪ TỌA ĐỘ để suy ra tâm trên khung hình GỐC (640x480)
            # Điều này bắt buộc để tay máy/robot nhắm chính xác
            center_x_original = center_x_crop + x_min
            center_y_original = center_y_crop + y_min
            
            # Đóng gói dữ liệu để gửi đi
            msg = Point()
            msg.x = center_x_original
            msg.y = center_y_original
            msg.z = class_id 
            
            # Gửi lên mạng lưới ROS 2
            self.publisher_.publish(msg)
            self.get_logger().info(f"Mục tiêu to nhất! Tâm gốc X: {center_x_original:.1f}, Y: {center_y_original:.1f} | Class ID: {int(class_id)} | Diện tích: {max_area:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloTensorRTNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()