#!/usr/bin/env python3
"""
KFS Align & Grasp Server — ABU Robocon 2026 "Kung Fu Quest"

Action: /kfs_align_grasp  (ak60_bringup/FollowTarget)
  goal.target_id           = class_id YOLO của KFS cần bám
  goal.desired_distance_mm = khoảng cách dừng trước KFS (mm)
  result.success / message  = "SUCCESS" | "GRASP_FAIL" | "canceled" | "empty" | "depth_lost"

Phase 1  ALIGN_X  : Chạy YOLO, trượt ngang đến khi KFS vào tâm ảnh
Phase 2  APPROACH : Tắt YOLO, dùng depth tiến thẳng đến desired_distance_mm
Phase 3  GRASP    : Gọi arm_sequence_controller (approach_j4 → approach_j3 → grasp)

PID và Kalman kế thừa nguyên từ tracking_server.py (đã tinh chỉnh trên robot).

Subscribe:
  /camera/color/image_raw                  (sensor_msgs/Image)
  /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)

Publish:
  /cmd_vel  (geometry_msgs/Twist)

Parameter:
  model_path  (str)  '/home/robocon/ros_ws/yolo26n_best.engine'
  grasp_side  (str)  'right'  |  'left'
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ultralytics import YOLO

from ak60_bringup.action import ArmSequence, FollowTarget

# ── PID / Kalman constants (from tracking_server.py) ─────────────────────────
IMG_CENTER_X       = 360.0
KP_LATERAL         = 0.0005
KP_LINEAR_MM       = 0.001
MAX_LINEAR_SPEED   = 0.1
MAX_LATERAL_SPEED  = 0.1
TOL_X_PX           = 20.0
TOL_DIST_MM        = 10.0
ALIGNED_FRAMES_REQ = 10
SUCCESS_FRAMES_REQ = 10
LOST_FRAMES_MAX    = 10
MIN_CREEP          = 0.05    # tốc độ tối thiểu để thắng ma sát tĩnh

# Phases
PHASE_ALIGN_X  = 0
PHASE_APPROACH = 1
PHASE_GRASP    = 2
PHASE_DONE     = 3


class KfsAlignGraspServer(Node):
    def __init__(self):
        super().__init__('kfs_align_grasp_server')
        self.cbg = ReentrantCallbackGroup()

        self.declare_parameter('model_path', '/home/robocon/ros_ws/yolo26n_best.engine')
        self.declare_parameter('grasp_side', 'right')

        model_path      = self.get_parameter('model_path').value
        self.grasp_side = self.get_parameter('grasp_side').value

        self.get_logger().info(f"Nạp YOLO TensorRT: {model_path}")
        self.model = YOLO(model_path, task='detect')
        self.get_logger().info("Model sẵn sàng.")

        self.br         = CvBridge()
        self.depth_lock = threading.Lock()

        # ── Tracking state ────────────────────────────────────────────────────
        self.is_tracking      = False
        self.current_phase    = PHASE_DONE
        self.target_id        = 0
        self.desired_dist     = 300.0
        self.latest_depth_img = None
        self.last_target_y    = 240        # fallback: vertical center
        self.aligned_frames   = 0
        self.success_frames   = 0
        self.lost_frames      = 0
        self.abort_reason     = ""
        self.current_gh       = None

        # Kalman 4-state: [cx, dist, vcx, vdist]
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix   = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kf.transitionMatrix    = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        self.kf.processNoiseCov     = np.eye(4, dtype=np.float32) * 0.05
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5.0
        self.kf_initialized         = False

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            self._depth_cb, 10, callback_group=self.cbg)
        self.create_subscription(
            Image, '/camera/color/image_raw',
            self._color_cb, 10, callback_group=self.cbg)

        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_client = ActionClient(
            self, ArmSequence, 'arm_sequence_controller',
            callback_group=self.cbg)

        ActionServer(
            self, FollowTarget, 'kfs_align_grasp',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cbg,
        )

        self.get_logger().info(
            f"KFS Align-Grasp Server sẵn sàng — grasp_side={self.grasp_side}")

    # ── Sensor callbacks ──────────────────────────────────────────────────────
    def _depth_cb(self, msg):
        with self.depth_lock:
            self.latest_depth_img = self.br.imgmsg_to_cv2(
                msg, desired_encoding='16UC1')

    def _color_cb(self, msg):
        if not self.is_tracking:
            return
        frame = self.br.imgmsg_to_cv2(msg, 'bgr8')

        if self.current_phase == PHASE_ALIGN_X:
            self._phase_align(frame)
        elif self.current_phase == PHASE_APPROACH:
            self._phase_approach()

    # ── Phase 1: Strafe until KFS centred in X ────────────────────────────────
    def _phase_align(self, frame):
        results    = self.model(frame, verbose=False)
        raw_cx     = 0.0
        raw_dist   = 0.0
        found      = False

        for r in results:
            for box in r.boxes:
                if int(box.cls[0].cpu().numpy()) != self.target_id:
                    continue
                b  = box.xyxy[0].cpu().numpy()
                cx = float((b[0] + b[2]) / 2)
                cy = int((b[1] + b[3]) / 2)

                with self.depth_lock:
                    dist = 0.0
                    if self.latest_depth_img is not None:
                        h, w = self.latest_depth_img.shape
                        if 0 <= cy < h and 0 <= int(cx) < w:
                            dist = float(self.latest_depth_img[cy, int(cx)])

                # Accept only objects in front-ish region
                if abs(IMG_CENTER_X - cx) <= 200.0 and 0 < dist < 1200.0:
                    raw_cx, raw_dist        = cx, dist
                    self.last_target_y      = cy
                    found                   = True
                    break
            if found:
                break

        if not found:
            self.lost_frames += 1
            if self.lost_frames >= LOST_FRAMES_MAX:
                self.get_logger().error("ALIGN_X: mất target quá lâu → ABORT")
                self.abort_reason = 'empty'
                self._stop()
                self.kf_initialized = False
                self.is_tracking    = False
            return

        self.lost_frames = 0
        meas = np.array([[np.float32(raw_cx)], [np.float32(raw_dist)]])
        if not self.kf_initialized:
            self.kf.statePost   = np.array(
                [[np.float32(raw_cx)], [np.float32(raw_dist)], [0.0], [0.0]],
                np.float32)
            self.kf_initialized = True
            smooth_x            = raw_cx
        else:
            self.kf.predict()
            smooth_x = float(self.kf.correct(meas)[0][0])

        self._pub_fb('real', IMG_CENTER_X - smooth_x, raw_dist - self.desired_dist)

        err_x = IMG_CENTER_X - smooth_x
        if abs(err_x) <= TOL_X_PX:
            self.aligned_frames += 1
            if self.aligned_frames >= ALIGNED_FRAMES_REQ:
                self.get_logger().info(
                    f"✅ ALIGN_X done  err={err_x:+.1f}px — chuyển APPROACH")
                self._stop()
                self.current_phase  = PHASE_APPROACH
                self.success_frames = 0
                self.lost_frames    = 0
        else:
            self.aligned_frames = 0
            v_y = err_x * KP_LATERAL
            if 0 < abs(v_y) < MIN_CREEP:
                v_y = MIN_CREEP if v_y > 0 else -MIN_CREEP
            twist = Twist()
            twist.linear.y = max(min(v_y, MAX_LATERAL_SPEED), -MAX_LATERAL_SPEED)
            self.cmd_pub.publish(twist)

    # ── Phase 2: Advance until desired depth ─────────────────────────────────
    def _phase_approach(self):
        dist_mm = 0.0
        with self.depth_lock:
            if self.latest_depth_img is not None:
                h, w  = self.latest_depth_img.shape
                cx    = int(IMG_CENTER_X)
                cy    = max(0, min(self.last_target_y, h - 1))
                patch = self.latest_depth_img[
                    max(0, cy-5):cy+6, max(0, cx-5):cx+6]
                valid = patch[patch > 0]
                if len(valid) > 0:
                    dist_mm = float(np.median(valid))

        if dist_mm <= 0:
            self.lost_frames += 1
            if self.lost_frames >= LOST_FRAMES_MAX:
                self.get_logger().error("APPROACH: mất depth quá lâu → ABORT")
                self.abort_reason = 'depth_lost'
                self._stop()
                self.is_tracking = False
            return

        self.lost_frames = 0
        err_d = dist_mm - self.desired_dist
        self._pub_fb('real', 0.0, err_d)

        if abs(err_d) < TOL_DIST_MM:
            self.success_frames += 1
        else:
            self.success_frames = 0

        if self.success_frames >= SUCCESS_FRAMES_REQ:
            self.get_logger().info(
                f"✅ APPROACH done  dist={dist_mm:.0f}mm — chuyển GRASP")
            self._stop()
            self.current_phase = PHASE_GRASP
            return

        v_x = err_d * KP_LINEAR_MM
        if 0 < abs(v_x) < MIN_CREEP:
            v_x = MIN_CREEP if v_x > 0 else -MIN_CREEP
        twist = Twist()
        twist.linear.x = max(min(v_x, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED)
        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f"[APPROACH] dist={dist_mm:.0f}mm err={err_d:+.0f}mm vx={twist.linear.x:.3f}",
            throttle_duration_sec=0.3)

    # ── Phase 3: Sequential arm poses ────────────────────────────────────────
    def _arm_send_sync(self, pose_name: str, timeout: float = 10.0) -> bool:
        """Send one ArmSequence goal and block until result. Returns success."""
        goal            = ArmSequence.Goal()
        goal.pose_name  = pose_name
        goal.duration   = 1.5

        done   = threading.Event()
        ok     = [False]

        def _on_result(fut):
            ok[0] = fut.result().result.success
            done.set()

        def _on_goal(fut):
            handle = fut.result()
            if not handle.accepted:
                self.get_logger().warn(f"Pose '{pose_name}' bị từ chối.")
                done.set()
                return
            handle.get_result_async().add_done_callback(_on_result)

        self.arm_client.send_goal_async(goal).add_done_callback(_on_goal)
        done.wait(timeout=timeout)
        return ok[0]

    def _do_grasp(self) -> bool:
        if not self.arm_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("arm_sequence_controller không phản hồi.")
            return False
        side = self.grasp_side
        for pose in (f'approach_j4_{side}', f'approach_j3_{side}', f'grasp_{side}'):
            self.get_logger().info(f"Arm pose: {pose}")
            if not self._arm_send_sync(pose):
                self.get_logger().warn(f"Pose '{pose}' thất bại.")
                return False
        return True

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _pub_fb(self, state: str, err_x: float, err_dist: float):
        if self.current_gh is None:
            return
        fb                   = FollowTarget.Feedback()
        fb.tracking_state    = state
        fb.error_x           = float(err_x)
        fb.error_distance_mm = float(err_dist)
        self.current_gh.publish_feedback(fb)

    # ── Action execute ────────────────────────────────────────────────────────
    def _execute(self, goal_handle):
        self.current_gh     = goal_handle
        self.target_id      = int(goal_handle.request.target_id)
        self.desired_dist   = float(goal_handle.request.desired_distance_mm) or 300.0
        self.current_phase  = PHASE_ALIGN_X
        self.aligned_frames = 0
        self.success_frames = 0
        self.lost_frames    = 0
        self.abort_reason   = ""
        self.kf_initialized = False
        self.is_tracking    = True

        self.get_logger().info(
            f"KFS Align-Grasp start — target_id={self.target_id} "
            f"dist={self.desired_dist:.0f}mm side={self.grasp_side}")

        result = FollowTarget.Result()

        while rclpy.ok() and self.is_tracking:

            if goal_handle.is_cancel_requested:
                self.is_tracking = False
                self._stop()
                goal_handle.canceled()
                result.success = False
                result.message = "canceled"
                return result

            if self.current_phase == PHASE_GRASP:
                ok = self._do_grasp()
                self._stop()
                self.is_tracking  = False
                self.current_phase = PHASE_DONE
                if ok:
                    goal_handle.succeed()
                    result.success = True
                    result.message = "SUCCESS"
                    self.get_logger().info("🎉 Gắp KFS thành công!")
                else:
                    goal_handle.abort()
                    result.success = False
                    result.message = "GRASP_FAIL"
                    self.get_logger().warn("Gripper không xác nhận.")
                return result

            time.sleep(0.05)

        # Aborted from sensor callback (empty/depth_lost)
        if goal_handle.is_active:
            result.success = False
            result.message = self.abort_reason or "LOST"
            goal_handle.abort()
            self.get_logger().error(f"Abort: {result.message}")

        return result


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node     = KfsAlignGraspServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
