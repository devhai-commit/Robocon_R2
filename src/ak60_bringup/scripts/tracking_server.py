#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np 
import threading  
import time 
from ultralytics import YOLO

from ak60_bringup.action import FollowTarget

class IntegratedTrackingServer(Node):
    def __init__(self):
        super().__init__('integrated_tracking_server')
        
        self.cb_group = ReentrantCallbackGroup()
        self.enable_gui = False

        self.get_logger().info("Đang nạp lõi YOLOv8 TensorRT...")
        self.model = YOLO('/home/robocon/ros_ws/yolo26n_best.engine', task='detect') 
        
        self.is_tracking = False
        self.target_id = 0
        self.desired_distance_mm = 0.0
        
        self.PHASE_ALIGN_X = 0   
        self.PHASE_APPROACH = 1  
        self.current_phase = self.PHASE_ALIGN_X
        self.aligned_frames = 0  
        self.last_target_y = 240 
        
        self.latest_depth_img = None
        self.br = CvBridge()
        self.depth_lock = threading.Lock() 
        
        self.current_goal_handle = None
        self.success_frames = 0
        self.lost_frames = 0
        self.abort_reason = "" # Lưu lý do thoát server (empty/fake)

        # Kalman Filter
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.05
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5.0 
        self.kf_initialized = False

        # PID Control
        self.img_center_x = 360.0
        self.kp_linear_mm = 0.001 
        self.kp_lateral = 0.0005    
        self.max_linear_speed = 0.1
        self.max_lateral_speed = 0.1
        self.tolerance_x = 20.0
        self.tolerance_distance = 10.0

        self.depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10, callback_group=self.cb_group)
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_cb, 10, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_server = ActionServer(
            self, FollowTarget, 'follow_target', execute_callback=self.execute_callback,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_callback, callback_group=self.cb_group)

        self.get_logger().info("Hệ Thống Tracking (Real/Empty/Fake Status) Đã Sẵn Sàng!")

    def depth_cb(self, msg):
        with self.depth_lock:
            self.latest_depth_img = self.br.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    def color_cb(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")

        if not self.is_tracking:
            if self.enable_gui:
                cv2.putText(frame, "SYSTEM STANDBY", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("Robot Vision", frame)
                cv2.waitKey(1)
            return

        # =========================================================================
        # GIAI ĐOẠN 1: BẬT YOLO AI, TRƯỢT NGANG TÌM TÂM X VÀ PHÂN LOẠI TRẠNG THÁI
        # =========================================================================
        if self.current_phase == self.PHASE_ALIGN_X:
            results = self.model(frame, verbose=False)
            annotated_frame = results[0].plot() if self.enable_gui else frame.copy()
            
            target_found = False
            objects_in_roi = 0 # Biến đếm số vật thể nằm trong vùng quan tâm
            raw_center_x, raw_distance_mm = 0.0, 0.0

            # 1. QUÉT TOÀN BỘ VẬT THỂ ĐỂ PHÂN LOẠI
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    class_id = int(box.cls[0].cpu().numpy()) 
                    b = box.xyxy[0].cpu().numpy()
                    temp_center_x = (b[0] + b[2]) / 2.0
                    center_y = int((b[1] + b[3]) / 2.0)
                    
                    temp_distance_mm = 0.0
                    with self.depth_lock:
                        if self.latest_depth_img is not None:
                            height, width = self.latest_depth_img.shape
                            if 0 <= center_y < height and 0 <= int(temp_center_x) < width:
                                temp_distance_mm = float(self.latest_depth_img[center_y, int(temp_center_x)])
                    
                    error_x_temp = self.img_center_x - temp_center_x
                    
                    # NẾU CÓ VẬT THỂ LỌT VÀO VÙNG NGẮM (Đạt chuẩn không gian)
                    if abs(error_x_temp) <= 200.0 and 0 < temp_distance_mm < 1200.0:
                        objects_in_roi += 1 
                        
                        # Nếu ID trùng khớp -> Đánh dấu REAL
                        if class_id == self.target_id:
                            raw_center_x = temp_center_x
                            raw_distance_mm = temp_distance_mm
                            self.last_target_y = center_y 
                            target_found = True
                            break 
                if target_found: break

            # 2. XÁC ĐỊNH TRẠNG THÁI HIỆN TẠI
            current_state = "empty"
            if target_found:
                current_state = "real"
            elif objects_in_roi > 0:
                current_state = "fake"

            # 3. GỬI FEEDBACK TRẠNG THÁI
            if self.current_goal_handle is not None:
                fb = FollowTarget.Feedback()
                fb.tracking_state = current_state
                # Chỉ xuất sai số khi là REAL
                fb.error_x = float(self.img_center_x - raw_center_x) if target_found else 0.0
                fb.error_distance_mm = float(raw_distance_mm - self.desired_distance_mm) if target_found else 0.0
                self.current_goal_handle.publish_feedback(fb)

            # 4. XỬ LÝ THEO TRẠNG THÁI
            smooth_x, smooth_distance = 0.0, 0.0

            if current_state == "real":
                self.lost_frames = 0
                meas = np.array([[np.float32(raw_center_x)], [np.float32(raw_distance_mm)]])

                if not self.kf_initialized:
                    self.kf.statePost = np.array([[np.float32(raw_center_x)], [np.float32(raw_distance_mm)], [0.0], [0.0]], np.float32)
                    self.kf_initialized = True
                    smooth_x = raw_center_x
                    smooth_distance = raw_distance_mm
                else:
                    self.kf.predict()
                    estimated = self.kf.correct(meas)
                    smooth_x = float(estimated[0][0])
                    smooth_distance = float(estimated[1][0])

                if self.enable_gui:
                    cv2.circle(annotated_frame, (int(raw_center_x), self.last_target_y), 5, (0, 0, 255), -1) 
                    cv2.circle(annotated_frame, (int(smooth_x), self.last_target_y), 8, (0, 255, 0), 2)     
            else:
                # Nếu là EMPTY hoặc FAKE
                self.lost_frames += 1
                if self.lost_frames >= 10: 
                    self.get_logger().error(f"❌ Abort Task! Trạng thái quét: {current_state.upper()}")
                    self.abort_reason = current_state
                    self.stop_robot()
                    self.kf_initialized = False 
                    self.is_tracking = False  # Báo sập cờ để execute_callback thoát ngay
                
                if self.enable_gui:
                    cv2.putText(annotated_frame, f"STATUS: {current_state.upper()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.imshow("Robot Vision", annotated_frame)
                    cv2.waitKey(1)
                return

            error_x = self.img_center_x - smooth_x

            # --- KIỂM TRA CHUYỂN GIAI ĐOẠN ---
            if abs(error_x) <= self.tolerance_x:
                self.aligned_frames += 1
                if self.aligned_frames >= 10:
                    self.get_logger().info("🎯 ĐÃ CĂN TÂM XONG! Tắt AI, chuyển sang tiến/lùi chốt khoảng cách...")
                    self.current_phase = self.PHASE_APPROACH
                    self.lost_frames = 0
                    self.stop_robot()
                    return
            else:
                self.aligned_frames = 0 

            # --- ĐIỀU KHIỂN ĐI NGANG ---
            twist = Twist()
            twist.linear.x = 0.0 
            twist.angular.z = 0.0 
            
            v_y = error_x * self.kp_lateral 
            if abs(v_y) > 0 and abs(v_y) < 0.05:
                v_y = 0.05 if v_y > 0 else -0.05
            twist.linear.y = max(min(v_y, self.max_lateral_speed), -self.max_lateral_speed)
            self.cmd_pub.publish(twist)

            if self.enable_gui:
                cv2.putText(annotated_frame, f"PHASE 1: STRAFING X | Err: {error_x:.1f}px", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imshow("Robot Vision", annotated_frame)
                cv2.waitKey(1)


        # =========================================================================
        # GIAI ĐOẠN 2: TẮT YOLO AI, ĐỌC CẢM BIẾN DEPTH, TIẾN THẲNG
        # =========================================================================
        elif self.current_phase == self.PHASE_APPROACH:
            distance_mm = 0.0
            with self.depth_lock:
                if self.latest_depth_img is not None:
                    h, w = self.latest_depth_img.shape
                    cx = int(self.img_center_x)
                    cy = int(self.last_target_y) 
                    
                    y_min = max(0, cy - 5)
                    y_max = min(h, cy + 5)
                    x_min = max(0, cx - 5)
                    x_max = min(w, cx + 5)
                    
                    patch = self.latest_depth_img[y_min:y_max, x_min:x_max]
                    valid_pixels = patch[patch > 0]
                    if len(valid_pixels) > 0:
                        distance_mm = float(np.median(valid_pixels))

            if distance_mm <= 0:
                self.lost_frames += 1
                if self.lost_frames >= 10: 
                    self.abort_reason = "depth_lost"
                    self.get_logger().error("❌ Mất dữ liệu Depth quá 10 frames! ABORT ACTION...")
                    self.stop_robot()
                    self.is_tracking = False 
                return

            self.lost_frames = 0
            error_dist = distance_mm - self.desired_distance_mm

            if self.current_goal_handle is not None:
                fb = FollowTarget.Feedback()
                fb.tracking_state = "real" # Giai đoạn này mặc định là real do AI đã chốt cứng
                fb.error_x = 0.0 
                fb.error_distance_mm = float(error_dist)
                self.current_goal_handle.publish_feedback(fb)

            if abs(error_dist) < self.tolerance_distance:
                self.success_frames += 1
            else:
                self.success_frames = 0

            if self.success_frames >= 10:
                self.stop_robot()
                return

            twist = Twist()
            twist.angular.z = 0.0
            twist.linear.y = 0.0

            if abs(error_dist) >= self.tolerance_distance:
                v_x = error_dist * self.kp_linear_mm
                if 0 < abs(v_x) < 0.05:
                    v_x = 0.05 if v_x > 0 else -0.05
            else:
                v_x = 0.0

            twist.linear.x = max(min(v_x, self.max_linear_speed), -self.max_linear_speed)
            self.cmd_pub.publish(twist)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Nhận Goal: Bám ID {int(goal_request.target_id)}, Cách {goal_request.desired_distance_mm}mm")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("⚠️ Đã nhận lệnh HỦY! Đang phanh khẩn cấp...")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.current_goal_handle = goal_handle
        self.target_id = int(goal_handle.request.target_id)
        self.desired_distance_mm = goal_handle.request.desired_distance_mm
        
        self.success_frames = 0
        self.lost_frames = 0
        self.aligned_frames = 0
        self.abort_reason = ""
        self.current_phase = self.PHASE_ALIGN_X
        self.kf_initialized = False 
        
        self.is_tracking = True 
        result = FollowTarget.Result()

        while rclpy.ok() and self.is_tracking:
            if goal_handle.is_cancel_requested:
                self.is_tracking = False
                self.stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = "canceled"
                return result

            if self.success_frames >= 10 and self.current_phase == self.PHASE_APPROACH:
                self.get_logger().info("✅ KẾT THÚC: Mục tiêu đã vào vị trí chuẩn!")
                self.is_tracking = False
                self.stop_robot()
                result.success = True
                result.message = "real"
                goal_handle.succeed()
                return result

            time.sleep(0.1) 

        # --- NẾU BỊ ĐÁ VĂNG (EMPTY HOẶC FAKE) ---
        if goal_handle.is_active:
            self.get_logger().error(f"🛑 Đang trả kết quả FAILURE ({self.abort_reason}) về cho Client...")
            result.success = False
            result.message = self.abort_reason
            goal_handle.abort()
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedTrackingServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        if node.enable_gui:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    

    