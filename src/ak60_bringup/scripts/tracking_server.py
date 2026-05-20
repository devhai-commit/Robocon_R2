#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry 
from cv_bridge import CvBridge

import cv2
import math 
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
        self.model = YOLO('/home/robocon/ros_ws/yolo26s_best.engine', task='detect') 

        self.get_logger().info("Đang ép xung TensorRT (Warm-up)... Vui lòng đợi khoảng 5 giây.")
        dummy_img = np.zeros((480, 640, 3), dtype=np.uint8) 
        _ = self.model(dummy_img, verbose=False) 
        self.get_logger().info("✅ TensorRT đã nóng máy!")
        
        self.PHASE_ALIGN_X = 0   
        self.PHASE_APPROACH = 1  
        
        self.latest_depth_img = None
        self.latest_color_img = None
        self.br = CvBridge()
        self.depth_lock = threading.Lock() 
        self.color_lock = threading.Lock() 

        # Quản lý góc Yaw
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.odom_received = False 
        self.odom_lock = threading.Lock()

        # Kalman Filter
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.05
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5.0 

        # PID Control
        self.img_center_x = 320.0
        self.kp_linear_mm = 0.003
        self.kp_lateral = 0.0005
        self.kp_yaw_correction = 1.0 
        
        self.max_linear_speed = 0.2
        self.max_lateral_speed = 0.2
        self.max_angular_speed = 0.3 
        
        self.tolerance_x = 10.0
        self.tolerance_distance = 10.0

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10, callback_group=self.cb_group)
        self.depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10, callback_group=self.cb_group)
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_cb, 10, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_server = ActionServer(
            self, FollowTarget, 'follow_target', execute_callback=self.execute_callback,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_callback, callback_group=self.cb_group)

        self.get_logger().info("Hệ Thống Tracking (Optimized Performance) Đã Sẵn Sàng!")

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        with self.odom_lock:
            self.current_yaw = yaw
            self.odom_received = True 

    def depth_cb(self, msg):
        with self.depth_lock:
            self.latest_depth_img = self.br.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    def color_cb(self, msg):
        with self.color_lock:
            self.latest_color_img = self.br.imgmsg_to_cv2(msg, "bgr8")

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Nhận Goal: Bám ID {goal_request.target_id}, Cách {goal_request.desired_distance_mm}mm")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("⚠️ Đã nhận lệnh HỦY! Đang phanh khẩn cấp...")
        return CancelResponse.ACCEPT

    def get_heading_correction(self):
        with self.odom_lock:
            curr_y = self.current_yaw
            
        error_yaw = self.target_yaw - curr_y
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))
        
        w_z = error_yaw * self.kp_yaw_correction
        return max(min(w_z, self.max_angular_speed), -self.max_angular_speed)

    def execute_callback(self, goal_handle):
        # [TỐI ƯU 2]: Ép kiểu an toàn (Safe Typecasting) để chống lỗi logic
        raw_target_id = str(goal_handle.request.target_id)
        target_label = raw_target_id[:-2] if raw_target_id.endswith('.0') else raw_target_id
        desired_distance_mm = goal_handle.request.desired_distance_mm
        
        result = FollowTarget.Result()

        waiting_time = 0.0
        while not self.odom_received and rclpy.ok():
            if waiting_time == 0.0:
                self.get_logger().warn("⏳ Đang chờ đồng bộ dữ liệu Odometry...")
            time.sleep(0.1)
            waiting_time += 0.1
            if waiting_time > 3.0: 
                self.get_logger().error("❌ Timeout: Không nhận được Odom. Abort!")
                goal_handle.abort()
                result.success = False
                result.message = "odom_timeout"
                return result

        with self.odom_lock:
            self.target_yaw = self.current_yaw
        self.get_logger().info(f"🔒 Đã khóa cứng Heading ở góc: {math.degrees(self.target_yaw):.2f} độ")
        
        success_frames = 0
        lost_frames = 0
        aligned_frames = 0
        current_phase = self.PHASE_ALIGN_X
        kf_initialized = False 
        last_target_y = 240
        
        loop_rate = self.create_rate(20) 

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.stop_robot()
                    goal_handle.canceled()
                    result.success = False
                    result.message = "canceled"
                    return result

                # [TỐI ƯU 1]: Copy sạch bộ nhớ Color VÀ Depth đúng 1 lần ở đầu vòng lặp
                color_frame = None
                depth_frame = None
                
                with self.color_lock:
                    if self.latest_color_img is not None:
                        color_frame = self.latest_color_img.copy()
                
                with self.depth_lock:
                    if self.latest_depth_img is not None:
                        depth_frame = self.latest_depth_img.copy()

                if color_frame is None or depth_frame is None:
                    self.stop_robot()
                    loop_rate.sleep()
                    continue

                annotated_frame = color_frame.copy() if self.enable_gui else None

                # =========================================================================
                # GIAI ĐOẠN 1: BẬT YOLO AI, TRƯỢT NGANG TÌM TÂM X VÀ PHÂN LOẠI TRẠNG THÁI
                # =========================================================================
                if current_phase == self.PHASE_ALIGN_X:
                    results = self.model(color_frame, verbose=False)
                    if self.enable_gui: annotated_frame = results[0].plot()
                    
                    target_found = False
                    objects_in_roi = 0
                    raw_center_x, raw_distance_mm = 0.0, 0.0
                    fake_labels = []

                    for r in results:
                        for box in r.boxes:
                            class_id = int(box.cls[0].cpu().numpy())
                            detected_label = str(self.model.names.get(class_id, class_id))
                            b = box.xyxy[0].cpu().numpy()
                            temp_center_x = (b[0] + b[2]) / 2.0
                            center_y = int((b[1] + b[3]) / 2.0)

                            # Tính Depth dùng bộ nhớ cục bộ (Local Buffer), không bị chặn Lock
                            temp_distance_mm = 0.0
                            h, w = depth_frame.shape
                            if 0 <= center_y < h and 0 <= int(temp_center_x) < w:
                                temp_distance_mm = float(depth_frame[center_y, int(temp_center_x)])

                            error_x_temp = self.img_center_x - temp_center_x
                            if abs(error_x_temp) <= 160.0 and 0 < temp_distance_mm < 1500.0:
                                objects_in_roi += 1
                                if detected_label == target_label:
                                    raw_center_x = temp_center_x
                                    raw_distance_mm = temp_distance_mm
                                    last_target_y = center_y
                                    target_found = True
                                    break
                                else:
                                    fake_labels.append(detected_label)
                        if target_found: break

                    current_state = "empty"
                    if target_found: current_state = "real"
                    elif objects_in_roi > 0: current_state = "fake"

                    fb = FollowTarget.Feedback()
                    fb.tracking_state = current_state
                    fb.error_x = float(self.img_center_x - raw_center_x) if target_found else 0.0
                    fb.error_distance_mm = float(raw_distance_mm - desired_distance_mm) if target_found else 0.0
                    goal_handle.publish_feedback(fb)

                    smooth_x = 0.0
                    if current_state == "real":
                        lost_frames = 0
                        meas = np.array([[np.float32(raw_center_x)], [np.float32(raw_distance_mm)]])

                        if not kf_initialized:
                            self.kf.statePost = np.array([[np.float32(raw_center_x)], [np.float32(raw_distance_mm)], [0.0], [0.0]], np.float32)
                            kf_initialized = True
                            smooth_x = raw_center_x
                        else:
                            self.kf.predict()
                            estimated = self.kf.correct(meas)
                            smooth_x = float(estimated[0][0])

                        if self.enable_gui:
                            cv2.circle(annotated_frame, (int(raw_center_x), last_target_y), 5, (0, 0, 255), -1) 
                            cv2.circle(annotated_frame, (int(smooth_x), last_target_y), 8, (0, 255, 0), 2)     
                    else:
                        lost_frames += 1
                        if lost_frames >= 10: 
                            if current_state == "fake" and fake_labels:
                                seen = []
                                for lbl in fake_labels:
                                    if lbl not in seen:
                                        seen.append(lbl)
                                detected_str = ", ".join(seen)
                                self.get_logger().error(f"❌ Abort Task! Trạng thái quét: FAKE | Cần: '{target_label}' | Đang nhận diện: [{detected_str}]")
                            else:
                                self.get_logger().error(f"❌ Abort Task! Mục tiêu: '{target_label}' | Trạng thái quét: {current_state.upper()}")
                            self.stop_robot()
                            result.success = False
                            result.message = current_state
                            goal_handle.abort()
                            return result
                        loop_rate.sleep()
                        continue

                    error_x = self.img_center_x - smooth_x

                    if abs(error_x) <= self.tolerance_x:
                        aligned_frames += 1
                        if aligned_frames >= 10:
                            self.stop_robot()
                            
                            if abs(desired_distance_mm) < 0.001: 
                                self.get_logger().info("🎯 ĐÃ CĂN TÂM XONG! Khoảng cách yêu cầu là 0.0 -> Bỏ qua tiến thẳng, HOÀN THÀNH NGAY!")
                                result.success = True
                                result.message = "real"
                                goal_handle.succeed()
                                return result
                            else:
                                self.get_logger().info("🎯 ĐÃ CĂN TÂM XONG! Tắt AI, chuyển sang tiến/lùi...")
                                current_phase = self.PHASE_APPROACH
                                lost_frames = 0
                                loop_rate.sleep()
                                continue
                    else:
                        aligned_frames = 0 

                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = self.get_heading_correction()

                    v_y = error_x * self.kp_lateral 
                    if abs(v_y) > 0 and abs(v_y) < 0.03:
                        v_y = 0.03 if v_y > 0 else -0.03
                    twist.linear.y = max(min(v_y, self.max_lateral_speed), -self.max_lateral_speed)
                    self.cmd_pub.publish(twist)

                    if self.enable_gui:
                        cv2.putText(annotated_frame, f"PHASE 1: STRAFING X | Err: {error_x:.1f}px", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


                # =========================================================================
                # GIAI ĐOẠN 2: ĐỌC CẢM BIẾN DEPTH ĐỂ TIẾN CHỐT KHOẢNG CÁCH
                # =========================================================================
                elif current_phase == self.PHASE_APPROACH:
                    distance_mm = 0.0
                    # Tính toán Depth từ Local Buffer
                    h, w = depth_frame.shape
                    cx = int(self.img_center_x)
                    cy = int(last_target_y) 
                    
                    y_min = max(0, cy - 5)
                    y_max = min(h, cy + 5)
                    x_min = max(0, cx - 5)
                    x_max = min(w, cx + 5)
                    
                    patch = depth_frame[y_min:y_max, x_min:x_max]
                    valid_pixels = patch[patch > 0]
                    if len(valid_pixels) > 0:
                        distance_mm = float(np.median(valid_pixels))

                    if distance_mm <= 0:
                        lost_frames += 1
                        if lost_frames >= 10: 
                            self.get_logger().error(f"❌ Mất dữ liệu Depth quá 10 frames! ABORT ACTION... Mục tiêu: '{target_label}'")
                            self.stop_robot()
                            result.success = False
                            result.message = "depth_lost"
                            goal_handle.abort()
                            return result
                        loop_rate.sleep()
                        continue

                    lost_frames = 0
                    error_dist = distance_mm - desired_distance_mm

                    fb = FollowTarget.Feedback()
                    fb.tracking_state = "real" 
                    fb.error_x = 0.0 
                    fb.error_distance_mm = float(error_dist)
                    goal_handle.publish_feedback(fb)

                    if abs(error_dist) < self.tolerance_distance:
                        success_frames += 1
                    else:
                        success_frames = 0

                    if success_frames >= 10:
                        self.get_logger().info("✅ KẾT THÚC: Mục tiêu đã vào vị trí chuẩn!")
                        self.stop_robot()
                        result.success = True
                        result.message = "real"
                        goal_handle.succeed()
                        return result

                    twist = Twist()
                    twist.linear.y = 0.0
                    twist.angular.z = self.get_heading_correction()

                    if abs(error_dist) >= self.tolerance_distance:
                        v_x = error_dist * self.kp_linear_mm
                        if 0 < abs(v_x) < 0.05:
                            v_x = 0.05 if v_x > 0 else -0.05
                    else:
                        v_x = 0.0

                    twist.linear.x = max(min(v_x, self.max_linear_speed), -self.max_linear_speed)
                    self.cmd_pub.publish(twist)

                if self.enable_gui and annotated_frame is not None:
                    cv2.imshow("Robot Vision", annotated_frame)
                    cv2.waitKey(1)

                loop_rate.sleep() 

        finally:
            self.stop_robot()
            if self.enable_gui:
                cv2.destroyAllWindows()
                cv2.waitKey(1) 

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedTrackingServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Đang tắt Node, phanh xe an toàn...")
            node.stop_robot()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
    