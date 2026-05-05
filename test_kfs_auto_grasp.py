#!/usr/bin/env python3
"""
Test KFS Auto Grasp — Tự động align + gắp KFS (không cần action server)

Phases:
  ALIGN_X   → trượt ngang đến khi KFS vào tâm ảnh
  APPROACH  → tiến đến khoảng cách gắp
  PRE_GRASP → hạ cánh tay xuống vị trí chuẩn bị
  EXTEND    → duỗi arm_joint_1 về phía KFS
  CLOSE     → đóng gripper
  LIFT      → nâng cánh tay lên
  RETRACT   → thu cánh tay về
  DONE      → hoàn thành

Interface arm: /arm_controller/commands (Float64MultiArray)
  [arm1_m, arm2_rad, arm3_rad, gripper_rad]   (khớp teleop_test_arm.py)

⚠  Tune GRASP_POSES trước khi chạy thật (dùng teleop_test_arm.py để tìm vị trí).
⚠  Nhấn SPACE để bật — mặc định TẮT.
⚠  Nhấn r để reset, q/ESC để thoát (robot về HOME).

Chạy:
    source /opt/ros/humble/setup.bash && source ~/ros_ws/install/setup.bash
    python3 test_kfs_auto_grasp.py
    python3 test_kfs_auto_grasp.py --target-id 5 --dist 250
    python3 test_kfs_auto_grasp.py --model yolo26n_best.pt --conf 0.3
"""

import argparse
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from control_msgs.msg import DynamicJointState
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from ultralytics import YOLO

# ── PID constants — khớp tracking_server.py ──────────────────────────────────
IMG_CENTER_X       = 360.0
KP_LATERAL         = 0.0005
KP_LINEAR_MM       = 0.001
MAX_LINEAR_SPEED   = 0.1
MAX_LATERAL_SPEED  = 0.1
TOL_X_PX           = 20.0
TOL_DIST_MM        = 15.0
ALIGNED_FRAMES_REQ = 10
SUCCESS_FRAMES_REQ = 10
LOST_FRAMES_MAX    = 10
MIN_CREEP          = 0.05

# ── Grasp poses — TUNE sau khi thử bằng teleop_test_arm.py ───────────────────
# Định dạng: (arm1_mm, arm2_deg, arm3_deg, gripper_deg, wait_sec)
GRASP_POSES = {
    'home':      (  0.0,  -5.0,   0.0,  45.0,  1.0),
    'pre_grasp': (  300.0,  55.0, 0.0,  70.0,  1.5),   # vào vị trí gắp, gripper mở
    'extend':    (  300.0,  50.0,  0.0,  70.0,  1.0),   # giữ nguyên vị trí (arm1=0)
    'close':     (  300,  70.0,  -0.5,  -28.0,  1.0),   # đóng gripper — vị trí thực tế
    'lift':      (  240.0,   5.0,   56.0,  -28.0,  1.5),   # nâng lên giữ vật
    'retract':   (  240.0,   50.0,   56.0,  70.0,  1.5),   # thu về
}
GRASP_SEQUENCE = ['pre_grasp', 'extend', 'close', 'lift', 'retract']

# Giới hạn phần cứng — khớp teleop_test_arm.py
ARM_LIMITS = [
    (0.0,    780.0),   # arm1 mm
    (-5.0,    90.0),   # arm2 deg
    (-90.0,   90.0),   # arm3 deg
    (-30.0,   80.0),   # gripper deg
]

# ── Phase labels ──────────────────────────────────────────────────────────────
PHASE_IDLE      = 'IDLE'
PHASE_ALIGN_X   = 'ALIGN_X'
PHASE_APPROACH  = 'APPROACH'
PHASE_PRE_GRASP = 'PRE_GRASP'
PHASE_EXTEND    = 'EXTEND'
PHASE_CLOSE     = 'CLOSE'
PHASE_LIFT      = 'LIFT'
PHASE_RETRACT   = 'RETRACT'
PHASE_DONE      = 'DONE'

PHASE_COLOR = {
    PHASE_IDLE:      (120, 120, 120),
    PHASE_ALIGN_X:   (0,   200, 255),
    PHASE_APPROACH:  (0,   255, 120),
    PHASE_PRE_GRASP: (255, 180,   0),
    PHASE_EXTEND:    (255, 140,   0),
    PHASE_CLOSE:     (255,  80,   0),
    PHASE_LIFT:      (200,   0, 255),
    PHASE_RETRACT:   (160,   0, 255),
    PHASE_DONE:      (0,   255,   0),
}


# ─────────────────────────────────────────────────────────────────────────────
class AutoGraspNode(Node):
    def __init__(self, target_id: int, desired_dist: float):
        super().__init__('kfs_auto_grasp')
        self.br    = CvBridge()
        self._lock = threading.Lock()

        self._color = None
        self._depth = None

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/color/image_raw', self._color_cb, 10)
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self._depth_cb, 10)
        self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self._joint_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10)

        # ── Arm state ─────────────────────────────────────────────────────────
        self._joint_names  = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'gripper_joint']
        self._arm_synced   = False
        self._arm_pos_user = list(GRASP_POSES['home'][:4])   # [mm, °, °, °]

        # ── Vision / motion state ─────────────────────────────────────────────
        self.target_id    = target_id
        self.desired_dist = desired_dist
        self._reset_state()

        self.get_logger().info(
            f"AutoGrasp — target_id={target_id}  dist={desired_dist:.0f}mm")

    # ── Camera callbacks ──────────────────────────────────────────────────────
    def _color_cb(self, msg):
        try:
            with self._lock:
                self._color = self.br.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _depth_cb(self, msg):
        try:
            with self._lock:
                self._depth = self.br.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception:
            pass

    def get_frames(self):
        with self._lock:
            c = self._color.copy() if self._color is not None else None
            d = self._depth.copy() if self._depth is not None else None
        return c, d

    # ── Arm joint state sync ──────────────────────────────────────────────────
    def _joint_cb(self, msg):
        if self._arm_synced:
            return
        temp = [None] * 4
        for i, name in enumerate(msg.joint_names):
            if name not in self._joint_names:
                continue
            idx = self._joint_names.index(name)
            iv  = msg.interface_values[i]
            if 'position' not in iv.interface_names:
                continue
            val      = iv.values[iv.interface_names.index('position')]
            temp[idx] = val * 1000.0 if idx == 0 else math.degrees(val)

        if all(p is not None for p in temp):
            self._arm_pos_user = temp
            self._arm_synced   = True
            self.get_logger().info(
                f"Arm synced: [{temp[0]:.0f}mm {temp[1]:.1f}° {temp[2]:.1f}° {temp[3]:.1f}°]")

    # ── Arm control ───────────────────────────────────────────────────────────
    def arm_go(self, pose_name: str):
        pose   = GRASP_POSES[pose_name]
        target = [
            max(ARM_LIMITS[i][0], min(ARM_LIMITS[i][1], pose[i]))
            for i in range(4)
        ]
        self._arm_pos_user = list(target)
        cmd      = Float64MultiArray()
        cmd.data = [
            target[0] / 1000.0,
            math.radians(target[1]),
            math.radians(target[2]),
            math.radians(target[3]),
        ]
        self.arm_pub.publish(cmd)
        self.get_logger().info(
            f"Arm → {pose_name}: [{target[0]:.0f}mm "
            f"{target[1]:.1f}° {target[2]:.1f}° {target[3]:.1f}°]")

    def arm_home(self):
        self.arm_go('home')

    # ── Motion helpers ────────────────────────────────────────────────────────
    def stop(self):
        self.cmd_pub.publish(Twist())

    def _reset_state(self):
        self.phase          = PHASE_IDLE
        self.aligned_frames = 0
        self.success_frames = 0
        self.lost_frames    = 0
        self.kf_initialized = False
        self.last_target_y  = 240
        kf = cv2.KalmanFilter(4, 2)
        kf.measurementMatrix   = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        kf.transitionMatrix    = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        kf.processNoiseCov     = np.eye(4, dtype=np.float32) * 0.05
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5.0
        self.kf = kf

    def reset(self):
        self.stop()
        self._reset_state()

    # ── Vision control step ───────────────────────────────────────────────────
    def step_vision(self, yolo_results, depth_img, move_enabled: bool) -> dict:
        info = {
            'phase': self.phase, 'found': False,
            'raw_cx': 0.0, 'raw_dist': 0.0,
            'err_x': 0.0, 'err_dist': 0.0,
            'vx': 0.0, 'vy': 0.0,
            'aligned_frames': self.aligned_frames,
            'success_frames': self.success_frames,
            'lost_frames':    self.lost_frames,
            'box': None, 'target_y': self.last_target_y,
        }

        if self.phase == PHASE_ALIGN_X:
            raw_cx, raw_dist, found, box = self._find_target(yolo_results, depth_img)
            info.update({'found': found, 'raw_cx': raw_cx,
                         'raw_dist': raw_dist, 'box': box})

            if not found:
                self.lost_frames += 1
                if move_enabled and self.lost_frames >= LOST_FRAMES_MAX:
                    self.stop()
                return info

            self.lost_frames = 0
            smooth_cx = self._kalman_update(raw_cx, raw_dist)
            err_x     = IMG_CENTER_X - smooth_cx
            info['err_x'] = err_x

            if abs(err_x) <= TOL_X_PX:
                self.aligned_frames += 1
                if self.aligned_frames >= ALIGNED_FRAMES_REQ:
                    self.get_logger().info("✅ ALIGN_X xong — chuyển APPROACH")
                    self.phase = PHASE_APPROACH
                    self.success_frames = self.lost_frames = 0
                    if move_enabled:
                        self.stop()
            else:
                self.aligned_frames = 0
                v_y = err_x * KP_LATERAL
                if 0 < abs(v_y) < MIN_CREEP:
                    v_y = MIN_CREEP if v_y > 0 else -MIN_CREEP
                v_y = max(min(v_y, MAX_LATERAL_SPEED), -MAX_LATERAL_SPEED)
                info['vy'] = v_y
                if move_enabled:
                    twist = Twist()
                    twist.linear.y = v_y
                    self.cmd_pub.publish(twist)

            info['phase'] = self.phase
            return info

        if self.phase == PHASE_APPROACH:
            dist_mm = self._median_depth(depth_img, int(IMG_CENTER_X), self.last_target_y)
            info['raw_dist'] = dist_mm

            if dist_mm <= 0:
                self.lost_frames += 1
                if move_enabled and self.lost_frames >= LOST_FRAMES_MAX:
                    self.stop()
                return info

            self.lost_frames = 0
            err_d            = dist_mm - self.desired_dist
            info['err_dist'] = err_d

            if abs(err_d) < TOL_DIST_MM:
                self.success_frames += 1
                if self.success_frames >= SUCCESS_FRAMES_REQ:
                    self.get_logger().info(
                        f"✅ APPROACH xong dist={dist_mm:.0f}mm — bắt đầu GRASP")
                    self.stop()
                    self.phase = PHASE_PRE_GRASP   # signal cho main loop
            else:
                self.success_frames = 0
                v_x = err_d * KP_LINEAR_MM
                if 0 < abs(v_x) < MIN_CREEP:
                    v_x = MIN_CREEP if v_x > 0 else -MIN_CREEP
                v_x = max(min(v_x, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED)
                info['vx'] = v_x
                if move_enabled:
                    twist = Twist()
                    twist.linear.x = v_x
                    self.cmd_pub.publish(twist)

            info['phase'] = self.phase
            return info

        return info

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _find_target(self, results, depth_img):
        for r in results:
            for box in r.boxes:
                if int(box.cls[0].cpu().numpy()) != self.target_id:
                    continue
                b    = box.xyxy[0].cpu().numpy()
                cx   = float((b[0] + b[2]) / 2)
                cy   = int((b[1] + b[3]) / 2)
                dist = self._median_depth(depth_img, int(cx), cy)
                in_d = (0 < dist < 4000.0) if dist > 0 else True
                if abs(IMG_CENTER_X - cx) <= 300.0 and in_d:
                    self.last_target_y = cy
                    return cx, dist, True, b
        return 0.0, 0.0, False, None

    def _median_depth(self, depth_img, cx, cy, half=5):
        if depth_img is None:
            return 0.0
        h, w   = depth_img.shape
        y0, y1 = max(0, cy - half), min(h - 1, cy + half)
        x0, x1 = max(0, cx - half), min(w - 1, cx + half)
        patch  = depth_img[y0:y1+1, x0:x1+1]
        valid  = patch[patch > 0]
        return float(np.median(valid)) if len(valid) >= 1 else 0.0

    def _kalman_update(self, raw_cx, raw_dist):
        meas = np.array([[np.float32(raw_cx)], [np.float32(raw_dist)]])
        if not self.kf_initialized:
            self.kf.statePost   = np.array(
                [[np.float32(raw_cx)], [np.float32(raw_dist)], [0.0], [0.0]],
                np.float32)
            self.kf_initialized = True
            return raw_cx
        self.kf.predict()
        return float(self.kf.correct(meas)[0][0])


# ─────────────────────────────────────────────────────────────────────────────
def run_grasp_sequence(node: AutoGraspNode):
    """Chạy chuỗi arm poses trong thread riêng."""
    for pose_name in GRASP_SEQUENCE:
        if node.phase == PHASE_IDLE:
            return
        node.phase = pose_name.upper()
        node.arm_go(pose_name)
        time.sleep(GRASP_POSES[pose_name][4])
    node.phase = PHASE_DONE
    node.get_logger().info("🎉 Hoàn thành grasp sequence!")


# ─────────────────────────────────────────────────────────────────────────────
def draw_overlay(frame, info, node, fps, move_enabled):
    h, w   = frame.shape[:2]
    phase  = node.phase
    color  = PHASE_COLOR.get(phase, (128, 128, 128))
    cx_int = int(IMG_CENTER_X)

    mv_text  = f">>> {phase} <<<" if move_enabled else f"--- PAUSED | {phase} (SPACE) ---"
    mv_color = color if move_enabled else (80, 80, 80)
    cv2.rectangle(frame, (0, 0), (w, 30), (0, 0, 0), -1)
    (tw, _), _ = cv2.getTextSize(mv_text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
    cv2.putText(frame, mv_text, ((w - tw) // 2, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, mv_color, 2)

    cv2.line(frame, (cx_int, 30), (cx_int, h), (60, 60, 200), 1)
    tol = int(TOL_X_PX)
    cv2.rectangle(frame, (cx_int - tol, h // 2 - 6),
                  (cx_int + tol, h // 2 + 6), (80, 80, 80), 1)

    if info.get('found') and info.get('box') is not None:
        b  = info['box'].astype(int)
        bx = int(info['raw_cx'])
        by = info['target_y']
        cv2.rectangle(frame, (b[0], b[1]), (b[2], b[3]), color, 2)
        cv2.drawMarker(frame, (bx, by), color, cv2.MARKER_CROSS, 16, 2)
        cv2.line(frame, (bx, by), (cx_int, by), (200, 200, 0), 1)
        cv2.putText(frame, f"err_x={info['err_x']:+.0f}px",
                    (min(bx, cx_int) + 4, by - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

    arm_sync = "OK" if node._arm_synced else "WAIT..."
    lines = [
        (f"FPS {fps:.1f}   Arm: {arm_sync}",                           (0, 255, 0)),
        (f"Phase : {phase}",                                             color),
        (f"Target id={node.target_id}  goal={node.desired_dist:.0f}mm", (210, 210, 210)),
        (f"Depth {info['raw_dist']:.0f}mm  err={info['err_dist']:+.0f}mm",
                                                                         (210, 210, 210)),
        (f"Align {info['aligned_frames']:2d}/{ALIGNED_FRAMES_REQ}"
         f"  Succ {info['success_frames']:2d}/{SUCCESS_FRAMES_REQ}"
         f"  Lost {info['lost_frames']}",                                (210, 210, 210)),
        (f"cmd_vel  vx={info['vx']:+.4f}  vy={info['vy']:+.4f}",        (255, 200,  50)),
        (f"Arm [{node._arm_pos_user[0]:.0f}mm"
         f" {node._arm_pos_user[1]:.1f}°"
         f" {node._arm_pos_user[2]:.1f}°"
         f" {node._arm_pos_user[3]:.1f}°]",                             (180, 180, 255)),
    ]
    for i, (text, col) in enumerate(lines):
        cv2.putText(frame, text, (10, 48 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

    if phase == PHASE_DONE:
        text = "=== GRASP COMPLETE! ==="
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
        tx, ty = (w - tw) // 2, h // 2
        cv2.rectangle(frame, (tx - 10, ty - th - 8), (tx + tw + 10, ty + 10), (0, 0, 0), -1)
        cv2.putText(frame, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

    # Depth bar
    bar_y  = h - 20
    center = w // 2
    bar_w  = min(int(abs(info['err_x']) / (w / 2) * (w // 2)), w // 2)
    bar_c  = (0, 200, 80) if move_enabled else (60, 60, 180)
    if info['err_x'] > 0:
        cv2.rectangle(frame, (center, bar_y), (center + bar_w, bar_y + 14), bar_c, -1)
    else:
        cv2.rectangle(frame, (center - bar_w, bar_y), (center, bar_y + 14), bar_c, -1)
    cv2.line(frame, (center, bar_y - 4), (center, bar_y + 18), (255, 255, 255), 1)


# ─────────────────────────────────────────────────────────────────────────────
def run(args):
    rclpy.init()
    node = AutoGraspNode(args.target_id, args.dist)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"Nạp model: {args.model}")
    model = YOLO(args.model, task='detect')
    print("Class names (10 đầu):", {k: v for k, v in list(model.names.items())[:10]})

    print("Đợi arm sync từ /dynamic_joint_states (tối đa 5s)...")
    t0 = time.time()
    while not node._arm_synced and time.time() - t0 < 5.0:
        time.sleep(0.1)
    if not node._arm_synced:
        print("[WARN] Arm chưa sync — lệnh arm sẽ dùng giá trị mặc định.")

    move_enabled  = False
    grasp_thread  = None
    fps           = 0.0
    frame_cnt     = 0
    fps_t         = time.time()
    save_cnt      = 0
    show_dep      = False

    print(f"\nTarget_id={args.target_id}  dist={args.dist}mm")
    print("SPACE=bắt đầu/pause | r=reset→HOME | q/ESC=thoát | s=lưu | d=depth\n")

    try:
        while True:
            color, depth = node.get_frames()
            if color is None:
                time.sleep(0.05)
                continue

            display = color.copy()

            # YOLO chỉ khi đang ở phase vision
            if node.phase in (PHASE_ALIGN_X, PHASE_APPROACH):
                results = model(color, verbose=False, conf=args.conf)
                info    = node.step_vision(results, depth, move_enabled)

                # Khi APPROACH xong, phase chuyển sang PRE_GRASP — khởi động grasp
                if node.phase == PHASE_PRE_GRASP:
                    if grasp_thread is None or not grasp_thread.is_alive():
                        grasp_thread = threading.Thread(
                            target=run_grasp_sequence, args=(node,), daemon=True)
                        grasp_thread.start()
            else:
                results = None
                info    = {
                    'phase': node.phase, 'found': False,
                    'raw_cx': 0.0, 'raw_dist': 0.0,
                    'err_x': 0.0, 'err_dist': 0.0,
                    'vx': 0.0, 'vy': 0.0,
                    'aligned_frames': 0, 'success_frames': 0, 'lost_frames': 0,
                    'box': None, 'target_y': node.last_target_y,
                }

            frame_cnt += 1
            if frame_cnt % 30 == 0:
                fps   = 30.0 / (time.time() - fps_t)
                fps_t = time.time()
                n_det = sum(len(r.boxes) for r in results) if results else 0
                n_tgt = (sum(1 for r in results for b in r.boxes
                             if int(b.cls[0].cpu().numpy()) == node.target_id)
                         if results else 0)
                print(f"\r[F{frame_cnt:5d}] det={n_det} id={node.target_id}(ok={n_tgt})"
                      f" depth={'OK' if depth is not None else 'NONE'}"
                      f" phase={node.phase} move={'ON' if move_enabled else 'OFF'}",
                      end='', flush=True)

            draw_overlay(display, info, node, fps, move_enabled)

            if show_dep and depth is not None:
                dc   = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
                view = np.hstack([display, dc])
            else:
                view = display

            cv2.imshow(
                f"KFS Auto Grasp | id={node.target_id} | {node.phase}"
                " | SPACE/r/q", view)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord('q'), 27):
                break

            elif key == ord(' '):
                if node.phase == PHASE_IDLE:
                    node.phase   = PHASE_ALIGN_X
                    grasp_thread = None
                    print("\n→ Bắt đầu ALIGN_X")
                move_enabled = not move_enabled
                if not move_enabled:
                    node.stop()
                print(f"\nDi chuyển: {'BẬT ✓' if move_enabled else 'TẮT'}")

            elif key == ord('r'):
                node.stop()
                node.arm_home()
                node.reset()
                grasp_thread = None
                move_enabled = False
                print("\n→ Reset → IDLE, arm về HOME")

            elif key == ord('s'):
                fname = f"auto_grasp_{save_cnt:03d}.jpg"
                cv2.imwrite(fname, display)
                print(f"\n[LƯU] {fname}")
                save_cnt += 1

            elif key == ord('d'):
                show_dep = not show_dep

    finally:
        node.stop()
        node.arm_home()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        print("\nRobot dừng. Arm về HOME. Thoát.")


# ─────────────────────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(description='KFS Auto Grasp — align + gắp tự động')
    p.add_argument('--model',     default='/home/robocon/ros_ws/yolo26n_best.engine')
    p.add_argument('--target-id', type=int,   default=5,     dest='target_id',
                   help='Class ID YOLO của KFS (mặc định 5)')
    p.add_argument('--dist',      type=float, default=250.0,
                   help='Khoảng cách dừng trước khi gắp mm (mặc định 250)')
    p.add_argument('--conf',      type=float, default=0.5)
    run(p.parse_args())


if __name__ == '__main__':
    main()
