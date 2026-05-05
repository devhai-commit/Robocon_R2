#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool  # Thư viện Service chuẩn của ROS 2
import cv2
import torch
from ultralytics import YOLO

class YoloHybridNode(Node):
    def __init__(self):
        super().__init__('yolo_hybrid_node')
        
        # 1. Trạng thái hệ thống (Khởi động ở chế độ NGỦ ĐÔNG để tiết kiệm tài nguyên)
        self.is_tracking_active = False
        self.target_class_id = 0.0  # Mặc định tìm Người (ID = 0)
        
        self.get_logger().info("Khởi động YOLO ở chế độ Ngủ Đông (Standby)...")
        self.model = YOLO('/home/robocon/yolov8n.engine', task='detect') 
        
        # 2. Tạo Publisher (Vẫn giữ nguyên để truyền data tốc độ cao)
        self.publisher_ = self.create_publisher(Point, '/target_coordinates', 10)
        
        # 3. TẠO SERVICE SERVER (Công tắc Bật/Tắt từ xa)
        self.srv = self.create_service(SetBool, '/toggle_yolo', self.toggle_callback)
        
        # 4. Mở Camera
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        if not self.cap.isOpened():
            self.get_logger().error("CẢNH BÁO: Không thể kết nối với Camera!")
            return
            
        # 5. Timer chạy liên tục
        timer_period = 0.001  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Hệ thống đã sẵn sàng. Chờ lệnh từ Service /toggle_yolo !")

    def toggle_callback(self, request, response):
        """Hàm này sẽ chạy mỗi khi có ai đó gọi Service /toggle_yolo"""
        if request.data == True:
            self.is_tracking_active = True
            self.get_logger().info("ĐÃ NHẬN LỆNH: BẬT hệ thống AI. Đang quét mục tiêu...")
            response.success = True
            response.message = "YOLO is now ON"
        else:
            self.is_tracking_active = False
            self.get_logger().info("ĐÃ NHẬN LỆNH: TẮT hệ thống AI. Chuyển về ngủ đông...")
            response.success = True
            response.message = "YOLO is now OFF"
            
        return response

    def timer_callback(self):
        # NẾU ĐANG NGỦ ĐÔNG -> Bỏ qua toàn bộ phần xử lý ảnh để giải phóng 100% GPU
        if not self.is_tracking_active:
            # Vẫn đọc frame để xả bộ đệm camera, tránh hình bị trễ (buffer lag) khi bật lại
            self.cap.grab() 
            return
            
        ret, frame = self.cap.read()
        if not ret:
            return
        
        results = self.model(frame, verbose=False)
        
        for r in results:
            boxes = r.boxes
            for box in boxes:
                class_id = float(box.cls[0].cpu().numpy())
                
                # CHỈ LỌC RA VẬT THỂ MÀ HỆ THỐNG ĐANG TÌM KIẾM
                if class_id == self.target_class_id:
                    b = box.xyxy[0].cpu().numpy() 
                    center_x = float((b[0] + b[2]) / 2.0)
                    center_y = float((b[1] + b[3]) / 2.0)
                    
                    msg = Point()
                    msg.x = center_x
                    msg.y = center_y
                    msg.z = class_id 
                    
                    self.publisher_.publish(msg)
                    # Tắt log này đi nếu bạn không muốn Terminal bị trôi quá nhanh
                    # self.get_logger().info(f"Mục tiêu X: {center_x:.1f}, Y: {center_y:.1f}")
                    break 

def main(args=None):
    rclpy.init(args=args)
    node = YoloHybridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    