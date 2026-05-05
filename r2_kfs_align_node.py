#!/usr/bin/env python3
"""
R2 KFS Alignment Node — ABU Robocon 2026
Visual servoing + depth control để tiếp cận và gắp KFS.

Action Server: /align_and_grasp_kfs  (ak60_bringup/FollowTarget)
  goal.target_id           = 1.0 → R2 KFS  |  0.0 → R1 KFS
  goal.desired_distance_mm = khoảng cách dừng trước KFS (mm)
  result.message           = "SUCCESS" | "TIMEOUT" | "LOST" | "GRASP_FAIL"

Subscribe:
  /kfs/detections                  (std_msgs/String)  — JSON từ kfs_vision_node
  /camera/depth/image_rect_raw     (sensor_msgs/Image) — depth từ RealSense ROS2 node

Publish:
  /cmd_vel  (geometry_msgs/Twist)  — omni robot motion

State Machine:
  IDLE → SEARCH → CENTERING → APPROACHING → ALIGNED → GRASPING → DONE

PID vòng 1 (pixel X error → angular.z + linear.y):
  Căn KFS vào giữa khung hình → trượt ngang / xoay robot

PID vòng 2 (depth Z error → linear.x):
  Tiến/lùi cho đến khi đúng khoảng cách gắp
"""

import json
import time
import math
import numpy as np
import rclpy
import rclpy.task
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from action_msgs.msg import GoalStatus
from ak60_bringup.action import FollowTarget, ArmSequence


# ── Hằng số camera & điều khiển ──────────────────────────────────────────────
IMG_CX = 320.0          # tâm ảnh (640×480)
IMG_CY = 240.0

CENTER_PIX_THR        = 25     # px — coi là "đã căn giữa"
GRASP_DIST_DEFAULT_MM = 250.0  # mm — khoảng cách gắp mặc định
CLOSE_ENOUGH_MM       = 40.0   # ±40 mm quanh desired_dist → aligned

TASK_TIMEOUT_SEC = 15.0
LOST_TIMEOUT_SEC = 1.5         # giây mất target → dừng robot

# PID gains — kế thừa từ robot_control_node.py, tinh chỉnh cho depth
KP_LATERAL  = 0.0018   # pixel X → linear.y  (trượt ngang)
KP_ANGULAR  = 0.004    # pixel X → angular.z (xoay)
KP_FORWARD  = 0.0005   # depth err (mm) → linear.x (tiến/lùi)

MAX_VX = 0.18   # m/s
MAX_VY = 0.18
MAX_WZ = 0.8    # rad/s
MIN_CREEP = 0.03  # tốc độ tối thiểu để thắng ma sát tĩnh


# ─────────────────────────────────────────────────────────────────────────────
class State:
    IDLE        = 'IDLE'
    SEARCH      = 'SEARCH'
    CENTERING   = 'CENTERING'
    APPROACHING = 'APPROACHING'
    ALIGNED     = 'ALIGNED'
    GRASPING    = 'GRASPING'
    DONE        = 'DONE'


# ─────────────────────────────────────────────────────────────────────────────
class KfsAlignNode(Node):
    def __init__(self):
        super().__init__('kfs_align_node')
        self.cbg = ReentrantCallbackGroup()

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            String, '/kfs/detections', self._on_kfs, 10,
            callback_group=self.cbg)

        self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self._on_depth, 10,
            callback_group=self.cbg)

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Action clients ───────────────────────────────────────────────────
        self.arm_client = ActionClient(self, ArmSequence, 'arm_sequence_controller')

        # ── Action server ────────────────────────────────────────────────────
        ActionServer(
            self, FollowTarget, 'align_and_grasp_kfs',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cbg,
        )

        # ── Internal state ───────────────────────────────────────────────────
        self.bridge          = CvBridge()
        self.state           = State.IDLE
        self.target_class    = 'r2_kfs'
        self.desired_dist    = GRASP_DIST_DEFAULT_MM
        self._detections     = []
        self._depth_img      = None          # numpy uint16 (mm)
        self._last_kfs_t     = 0.0
        self._last_depth_t   = 0.0

        # Safety: dừng robot nếu mất target khi đang di chuyển
        self.create_timer(0.1, self._safety_stop, callback_group=self.cbg)

        self.get_logger().info("KFS Align Node sẵn sàng — /align_and_grasp_kfs")

    # ── Sensor callbacks ─────────────────────────────────────────────────────
    def _on_kfs(self, msg):
        try:
            self._detections = json.loads(msg.data)
            self._last_kfs_t = time.time()
        except Exception:
            self._detections = []

    def _on_depth(self, msg):
        try:
            self._depth_img    = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self._last_depth_t = time.time()
        except Exception:
            self._depth_img = None

    def _safety_stop(self):
        if self.state in (State.IDLE, State.DONE, State.GRASPING):
            return
        if time.time() - self._last_kfs_t > LOST_TIMEOUT_SEC:
            self._stop()

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _stop(self):
        self.pub_cmd.publish(Twist())

    def _best_target(self):
        """KFS của target_class có confidence cao nhất, hoặc None."""
        pool = [d for d in self._detections if d['name'] == self.target_class]
        return max(pool, key=lambda d: d['conf']) if pool else None

    def _depth_mm(self, cx, cy, half=5):
        """Median depth (mm) trong vùng 11×11px quanh (cx, cy)."""
        if self._depth_img is None:
            return None
        h, w = self._depth_img.shape[:2]
        x0 = max(0, int(cx) - half);  x1 = min(w - 1, int(cx) + half)
        y0 = max(0, int(cy) - half);  y1 = min(h - 1, int(cy) + half)
        patch = self._depth_img[y0:y1+1, x0:x1+1].astype(np.float32)
        valid = patch[(patch > 80) & (patch < 4000)]
        return float(np.median(valid)) if len(valid) >= 4 else None

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _with_creep(v, dead=0.01):
        """Nếu |v| > dead → thêm min creep để robot thực sự di chuyển."""
        if abs(v) < dead:
            return 0.0
        return math.copysign(abs(v) + MIN_CREEP, v)

    # ── PID ──────────────────────────────────────────────────────────────────
    def _compute_twist(self, kfs, depth):
        """
        Trả về (Twist, pixel_aligned, depth_aligned).

        Vòng 1 — pixel X:  trượt ngang + xoay nhẹ
        Vòng 2 — depth Z:  tiến/lùi
        """
        err_x = kfs['cx'] - IMG_CX   # + = KFS lệch phải

        vy_raw = -err_x * KP_LATERAL
        az_raw = -err_x * KP_ANGULAR

        twist = Twist()
        twist.linear.y  = self._clamp(vy_raw, -MAX_VY, MAX_VY)
        twist.angular.z = self._clamp(az_raw, -MAX_WZ, MAX_WZ)

        px_aligned = abs(err_x) < CENTER_PIX_THR

        depth_aligned = False
        if depth is not None:
            err_d  = depth - self.desired_dist   # + = còn xa
            vx_raw = self._with_creep(err_d * KP_FORWARD)
            twist.linear.x = self._clamp(vx_raw, -MAX_VX, MAX_VX)
            depth_aligned  = abs(err_d) < CLOSE_ENOUGH_MM
        # else: linear.x = 0.0 (mặc định của Twist)

        return twist, px_aligned, depth_aligned

    # ── Gripper ──────────────────────────────────────────────────────────────
    async def _grasp(self, pose='grasp', duration=1.5):
        if not self.arm_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("arm_sequence_controller không phản hồi.")
            return False
        goal          = ArmSequence.Goal()
        goal.pose_name = pose
        goal.duration  = float(duration)
        fut = self.arm_client.send_goal_async(goal)
        while not fut.done():
            await rclpy.task.sleep(0.05)
        handle = fut.result()
        if not handle.accepted:
            return False
        res_fut = handle.get_result_async()
        while not res_fut.done():
            await rclpy.task.sleep(0.05)
        return res_fut.result().result.success

    # ── Action Execute ────────────────────────────────────────────────────────
    async def _execute(self, goal_handle):
        req = goal_handle.request
        self.target_class = 'r2_kfs' if req.target_id == 1.0 else 'r1_kfs'
        self.desired_dist = req.desired_distance_mm or GRASP_DIST_DEFAULT_MM
        self.state        = State.SEARCH
        t_start           = time.time()

        self.get_logger().info(
            f"Bắt đầu căn chỉnh  target={self.target_class}  "
            f"dist={self.desired_dist:.0f}mm")

        fb  = FollowTarget.Feedback()
        res = FollowTarget.Result()

        while rclpy.ok():

            # Cancel ──────────────────────────────────────────────────────────
            if goal_handle.is_cancel_requested:
                self._stop()
                self.state = State.IDLE
                goal_handle.canceled()
                res.message = "CANCELED"
                return res

            # Timeout ─────────────────────────────────────────────────────────
            if time.time() - t_start > TASK_TIMEOUT_SEC:
                self._stop()
                self.state = State.DONE
                goal_handle.abort()
                res.message = "TIMEOUT"
                self.get_logger().warn("Timeout — không căn chỉnh được KFS.")
                return res

            kfs = self._best_target()

            # SEARCH ──────────────────────────────────────────────────────────
            if self.state == State.SEARCH:
                if kfs is None:
                    self._stop()
                    await rclpy.task.sleep(0.1)
                    continue
                self.state = State.CENTERING
                self.get_logger().info("KFS phát hiện — bắt đầu căn chỉnh.")

            # CENTERING / APPROACHING ─────────────────────────────────────────
            if self.state in (State.CENTERING, State.APPROACHING):
                if kfs is None:
                    self._stop()
                    await rclpy.task.sleep(0.05)
                    continue

                depth  = self._depth_mm(kfs['cx'], kfs['cy'])
                twist, px_ok, dist_ok = self._compute_twist(kfs, depth)
                self.pub_cmd.publish(twist)

                # Chuyển state
                if px_ok and depth is not None:
                    self.state = State.APPROACHING
                if px_ok and dist_ok:
                    self._stop()
                    self.state = State.ALIGNED
                    self.get_logger().info(
                        f"ALIGNED  depth={depth:.0f}mm  "
                        f"err_px={kfs['cx']-IMG_CX:+.0f}")

                # Feedback
                fb.distance_to_target = float(depth or -1.0)
                goal_handle.publish_feedback(fb)
                self.get_logger().info(
                    f"[{self.state}] px_err={kfs['cx']-IMG_CX:+.0f}  "
                    f"depth={depth:.0f}mm" if depth else
                    f"[{self.state}] px_err={kfs['cx']-IMG_CX:+.0f}  depth=N/A",
                    throttle_duration_sec=0.3)

                await rclpy.task.sleep(0.04)
                continue

            # ALIGNED → GRASPING ──────────────────────────────────────────────
            if self.state == State.ALIGNED:
                self.state = State.GRASPING
                ok = await self._grasp(pose='grasp', duration=1.5)
                self._stop()
                self.state = State.DONE

                if ok:
                    goal_handle.succeed()
                    res.message = "SUCCESS"
                    self.get_logger().info("Gắp KFS thành công!")
                else:
                    goal_handle.abort()
                    res.message = "GRASP_FAIL"
                    self.get_logger().warn("Gripper không xác nhận.")
                return res

            await rclpy.task.sleep(0.04)

        self._stop()
        self.state = State.IDLE
        res.message = "SHUTDOWN"
        goal_handle.abort()
        return res


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = KfsAlignNode()
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
