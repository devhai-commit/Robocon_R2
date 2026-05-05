#!/usr/bin/env python3
"""
KFS Vision Node — ABU Robocon 2026 "Kung Fu Quest"
Nhận diện R1 KFS, R2 KFS và Fake KFS bằng TensorRT trên Jetson Orin.

Nhận ảnh qua ROS2 topic từ realsense2_camera node (đang chạy sẵn, PID 5938).
KHÔNG mở camera trực tiếp — tránh xung đột với realsense node.

Topics subscribed:
  /camera/color/image_raw  (sensor_msgs/Image) — ảnh màu từ RealSense node

Topics published:
  /kfs/detections  (std_msgs/String)     — JSON list tất cả KFS tìm thấy
  /kfs/r2_target   (geometry_msgs/Point) — Tâm R2 KFS tốt nhất (x, y, conf)
  /kfs/image       (sensor_msgs/Image)   — Frame annotate (bật qua param)

Services:
  /kfs/enable (std_srvs/SetBool) — Bật/tắt inference

Parameters:
  model_path    (str)   '/home/robocon/ros_ws/yolo26n_best.engine'
  color_topic   (str)   '/camera/color/image_raw'
  confidence    (float) 0.5
  publish_image (bool)  True
  class_names   (list)  ['r1_kfs', 'r2_kfs', 'fake_kfs']
"""

import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from ultralytics import YOLO

DEFAULT_CLASS_NAMES = ['r1_kfs', 'r2_kfs', 'fake_kfs']


class KfsVisionNode(Node):
    def __init__(self):
        super().__init__('kfs_vision_node')

        # --- Parameters ---
        self.declare_parameter('model_path',   '/home/robocon/ros_ws/yolo26n_best.engine')
        self.declare_parameter('color_topic',  '/camera/color/image_raw')
        self.declare_parameter('confidence',   0.5)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('class_names',  DEFAULT_CLASS_NAMES)

        model_path         = self.get_parameter('model_path').value
        color_topic        = self.get_parameter('color_topic').value
        self.conf_thr      = self.get_parameter('confidence').value
        self.publish_image = self.get_parameter('publish_image').value
        self.class_names   = self.get_parameter('class_names').value

        # --- Model ---
        self.get_logger().info(f"Nạp model TensorRT: {model_path}")
        self.model = YOLO(model_path, task='detect')
        self.get_logger().info("Model sẵn sàng.")

        self.bridge    = CvBridge()
        self.is_active = True

        # --- Subscriber (nhận ảnh từ realsense2_camera node) ---
        self.create_subscription(Image, color_topic, self._on_image, 10)
        self.get_logger().info(f"Subscribe: {color_topic}")

        # --- Publishers ---
        self.pub_detections = self.create_publisher(String, '/kfs/detections', 10)
        self.pub_r2_target  = self.create_publisher(Point,  '/kfs/r2_target',  10)
        if self.publish_image:
            self.pub_image = self.create_publisher(Image, '/kfs/image', 10)

        # --- Service ---
        self.create_service(SetBool, '/kfs/enable', self._on_enable)

        self.get_logger().info("KFS Vision Node sẵn sàng. Service: /kfs/enable")

    # ------------------------------------------------------------------
    def _on_enable(self, request, response):
        self.is_active = request.data
        self.get_logger().info(f"Inference {'BẬT' if self.is_active else 'TẮT'}.")
        response.success = True
        response.message = f"kfs_vision {'ON' if self.is_active else 'OFF'}"
        return response

    # ------------------------------------------------------------------
    def _on_image(self, msg):
        if not self.is_active:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge input: {e}', throttle_duration_sec=5.0)
            return

        results    = self.model(frame, verbose=False, conf=self.conf_thr)
        detections = self._parse(results)

        self._pub_detections(detections)
        self._pub_r2_target(detections)
        if self.publish_image:
            self._pub_image(results)

    # ------------------------------------------------------------------
    def _parse(self, results):
        out = []
        for r in results:
            for box in r.boxes:
                class_id = int(box.cls[0].cpu().numpy())
                conf     = float(box.conf[0].cpu().numpy())
                b        = box.xyxy[0].cpu().numpy()
                cx = float((b[0] + b[2]) / 2.0)
                cy = float((b[1] + b[3]) / 2.0)

                name = (
                    self.class_names[class_id]
                    if class_id < len(self.class_names)
                    else f'class_{class_id}'
                )

                if 'fake' in name.lower():
                    self.get_logger().warn(
                        f'[FAKE KFS] tại ({cx:.0f},{cy:.0f}) conf={conf:.2f} — BỎ QUA',
                        throttle_duration_sec=2.0,
                    )
                    continue

                out.append({
                    'class_id': class_id,
                    'name':     name,
                    'conf':     round(conf, 3),
                    'cx':       round(cx, 1),
                    'cy':       round(cy, 1),
                    'w':        round(float(b[2] - b[0]), 1),
                    'h':        round(float(b[3] - b[1]), 1),
                })
        return out

    def _pub_detections(self, detections):
        msg      = String()
        msg.data = json.dumps(detections)
        self.pub_detections.publish(msg)

    def _pub_r2_target(self, detections):
        r2_list = [d for d in detections if d['name'] == 'r2_kfs']
        if not r2_list:
            return
        best  = max(r2_list, key=lambda d: d['conf'])
        msg   = Point()
        msg.x = best['cx']
        msg.y = best['cy']
        msg.z = best['conf']
        self.pub_r2_target.publish(msg)
        self.get_logger().info(
            f"R2 KFS → cx={best['cx']:.0f} cy={best['cy']:.0f} conf={best['conf']:.2f}",
            throttle_duration_sec=0.5,
        )

    def _pub_image(self, results):
        annotated = results[0].plot()
        try:
            img_msg                 = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            img_msg.header.stamp    = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera'
            self.pub_image.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'CvBridge output: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = KfsVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
