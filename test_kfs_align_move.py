#!/usr/bin/env python3
"""
Test KFS Align + Move — Robot thực sự di chuyển (không cần action server)

Subscribe ROS topic, chạy YOLO, publish /cmd_vel để robot tự căn chỉnh KFS.
Phase 1 ALIGN_X  : trượt ngang đến khi KFS vào tâm ảnh
Phase 2 APPROACH : tiến/lùi đến khoảng cách mong muốn
Phase 3 DONE     : dừng, hiển thị "ALIGNED"

⚠  Nhấn SPACE để bật/tắt di chuyển — mặc định TẮT khi khởi động!
⚠  Nhấn q hoặc Ctrl+C để thoát (robot dừng ngay).

Chạy:
    source /opt/ros/humble/setup.bash && source ~/ros_ws/install/setup.bash
    python3 test_kfs_align_move.py
    python3 test_kfs_align_move.py --target-id 1 --dist 300
    python3 test_kfs_align_move.py --model yolo26n_best.pt --conf 0.3

Phím (click vào cửa sổ ảnh trước):
    SPACE  — bật/tắt di chuyển robot (mặc định TẮT)
    r      — reset về Phase 1 (dừng robot, tắt di chuyển)
    q      — thoát (dừng robot)
    s      — lưu ảnh
    d      — bật/tắt depth colormap
    +/-    — tăng/giảm target_id (reset tự động)
"""

import argparse
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

# ── Constants — KHỚP với kfs_align_grasp_server.py ───────────────────────────
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
MIN_CREEP          = 0.05

PHASE_ALIGN_X  = 0
PHASE_APPROACH = 1
PHASE_DONE     = 2

PHASE_LABEL = {
    PHASE_ALIGN_X:  ("ALIGN X",  (0, 200, 255)),
    PHASE_APPROACH: ("APPROACH", (0, 255, 120)),
    PHASE_DONE:     ("ALIGNED!", (0, 255, 0)),
}


# ─────────────────────────────────────────────────────────────────────────────
class KfsAlignMoveNode(Node):
    def __init__(self, target_id: int, desired_dist: float):
        super().__init__('kfs_align_move')
        self.br    = CvBridge()
        self._lock = threading.Lock()
        self._color = None
        self._depth = None

        self.create_subscription(
            Image, '/camera/color/image_raw', self._color_cb, 10)
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self._depth_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_id    = target_id
        self.desired_dist = desired_dist
        self._reset_state()

        self.get_logger().info(
            f"KFS Align Move — target_id={target_id}  dist={desired_dist:.0f}mm")
        self.get_logger().info("Nhấn SPACE trong cửa sổ ảnh để bật di chuyển!")

    # ── ROS callbacks ─────────────────────────────────────────────────────────
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

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ── State ─────────────────────────────────────────────────────────────────
    def _reset_state(self):
        self.phase          = PHASE_ALIGN_X
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

    # ── Control step ──────────────────────────────────────────────────────────
    def step(self, yolo_results, depth_img, move_enabled: bool) -> dict:
        info = {
            'phase': self.phase, 'found': False,
            'raw_cx': 0.0, 'smooth_cx': 0.0,
            'raw_dist': 0.0, 'err_x': 0.0, 'err_dist': 0.0,
            'vx': 0.0, 'vy': 0.0,
            'aligned_frames': self.aligned_frames,
            'success_frames': self.success_frames,
            'lost_frames': self.lost_frames,
            'box': None, 'target_y': self.last_target_y,
        }

        if self.phase == PHASE_DONE:
            return info

        if self.phase == PHASE_ALIGN_X:
            raw_cx, raw_dist, found, box = self._find_target(yolo_results, depth_img)
            info.update({'found': found, 'raw_cx': raw_cx, 'raw_dist': raw_dist, 'box': box})

            if not found:
                self.lost_frames += 1
                # Chỉ dừng sau LOST_FRAMES_MAX frame liên tiếp (theo tracking_server.py)
                if move_enabled and self.lost_frames >= LOST_FRAMES_MAX:
                    self.stop()
                return info

            self.lost_frames = 0
            smooth_cx = self._kalman_update(raw_cx, raw_dist)
            err_x     = IMG_CENTER_X - smooth_cx
            info.update({'smooth_cx': smooth_cx, 'err_x': err_x})

            if abs(err_x) <= TOL_X_PX:
                self.aligned_frames += 1
                if self.aligned_frames >= ALIGNED_FRAMES_REQ:
                    self.get_logger().info("✅ ALIGN_X xong — chuyển APPROACH")
                    self.phase          = PHASE_APPROACH
                    self.success_frames = 0
                    self.lost_frames    = 0
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
                if move_enabled:
                    self.stop()
                return info

            self.lost_frames = 0
            err_d = dist_mm - self.desired_dist
            info['err_dist'] = err_d

            if abs(err_d) < TOL_DIST_MM:
                self.success_frames += 1
                if self.success_frames >= SUCCESS_FRAMES_REQ:
                    self.get_logger().info(
                        f"✅ APPROACH xong — dist={dist_mm:.0f}mm")
                    self.phase = PHASE_DONE
                    if move_enabled:
                        self.stop()
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
                # Tăng giới hạn depth lên 4000mm (D435 range), bỏ filter nếu depth không có
                in_depth = (0 < dist < 4000.0) if dist > 0 else True
                if abs(IMG_CENTER_X - cx) <= 300.0 and in_depth:
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
        # Giảm yêu cầu xuống 1 pixel (theo tracking_server.py dùng single pixel)
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
def draw_overlay(frame, info, node, fps, move_enabled):
    h, w         = frame.shape[:2]
    phase        = info['phase']
    label, color = PHASE_LABEL.get(phase, ("???", (128, 128, 128)))
    cx_int       = int(IMG_CENTER_X)

    # Banner trạng thái di chuyển
    mv_text  = ">>> MOVING <<<" if move_enabled else "--- PAUSED (SPACE to move) ---"
    mv_color = (0, 255, 80)     if move_enabled else (0, 80, 220)
    cv2.rectangle(frame, (0, 0), (w, 30), (0, 0, 0), -1)
    (tw, _), _ = cv2.getTextSize(mv_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    cv2.putText(frame, mv_text, ((w - tw) // 2, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, mv_color, 2)

    # Đường tâm + vùng tolerance
    cv2.line(frame, (cx_int, 30), (cx_int, h), (60, 60, 200), 1)
    tol = int(TOL_X_PX)
    cv2.rectangle(frame,
                  (cx_int - tol, h // 2 - 6),
                  (cx_int + tol, h // 2 + 6), (80, 80, 80), 1)

    # Bounding box + lệch
    if info['found'] and info['box'] is not None:
        b  = info['box'].astype(int)
        bx = int(info['raw_cx'])
        by = info['target_y']
        cv2.rectangle(frame, (b[0], b[1]), (b[2], b[3]), color, 2)
        cv2.drawMarker(frame, (bx, by), color, cv2.MARKER_CROSS, 16, 2)
        cv2.line(frame, (bx, by), (cx_int, by), (200, 200, 0), 1)
        cv2.putText(frame, f"err_x={info['err_x']:+.0f}px",
                    (min(bx, cx_int) + 4, by - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

    # HUD
    lines = [
        (f"FPS: {fps:.1f}",                                         (0, 255, 0)),
        (f"Phase : {label}",                                         color),
        (f"Target: id={node.target_id}  goal={node.desired_dist:.0f}mm",
                                                                     (210, 210, 210)),
        (f"Depth : {info['raw_dist']:.0f}mm  err={info['err_dist']:+.0f}mm",
                                                                     (210, 210, 210)),
        (f"Align {info['aligned_frames']:2d}/{ALIGNED_FRAMES_REQ}"
         f"  Succ {info['success_frames']:2d}/{SUCCESS_FRAMES_REQ}"
         f"  Lost {info['lost_frames']}",                            (210, 210, 210)),
        (f"cmd_vel  vx={info['vx']:+.4f}  vy={info['vy']:+.4f}",    (255, 200, 50)),
    ]
    for i, (text, col) in enumerate(lines):
        cv2.putText(frame, text, (10, 50 + i * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

    # Thanh lệch ngang ở đáy
    bar_y  = h - 20
    center = w // 2
    bar_w  = min(int(abs(info['err_x']) / (w / 2) * (w // 2)), w // 2)
    bar_c  = (0, 200, 80) if move_enabled else (60, 60, 180)
    if info['err_x'] > 0:
        cv2.rectangle(frame, (center, bar_y), (center + bar_w, bar_y + 14), bar_c, -1)
    else:
        cv2.rectangle(frame, (center - bar_w, bar_y), (center, bar_y + 14), bar_c, -1)
    cv2.line(frame, (center, bar_y - 4), (center, bar_y + 18), (255, 255, 255), 1)

    # Banner DONE
    if phase == PHASE_DONE:
        text = "=== ALIGNED! ==="
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.1, 2)
        tx = (w - tw) // 2
        ty = h // 2
        cv2.rectangle(frame, (tx - 10, ty - th - 8), (tx + tw + 10, ty + 10), (0, 0, 0), -1)
        cv2.putText(frame, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 2)


# ─────────────────────────────────────────────────────────────────────────────
def run(args):
    rclpy.init()
    node = KfsAlignMoveNode(args.target_id, args.dist)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"Nạp model: {args.model}")
    model = YOLO(args.model, task='detect')
    print("Class names (10 đầu):", {k: v for k, v in list(model.names.items())[:10]})

    move_enabled = False
    fps          = 0.0
    frame_cnt    = 0
    fps_t        = time.time()
    save_cnt     = 0
    show_dep     = False

    print(f"\nTarget_id={args.target_id}  dist={args.dist}mm")
    print("Nhấn SPACE để bật di chuyển, q để thoát\n")

    try:
        while True:
            color, depth = node.get_frames()
            if color is None:
                time.sleep(0.05)
                continue

            display = color.copy()
            results = model(color, verbose=False, conf=args.conf)
            info    = node.step(results, depth, move_enabled)

            frame_cnt += 1
            if frame_cnt % 30 == 0:
                fps   = 30.0 / (time.time() - fps_t)
                fps_t = time.time()
                # Diagnostic: in trạng thái mỗi 30 frame (~1s)
                n_det = sum(len(r.boxes) for r in results)
                n_target = sum(
                    1 for r in results for b in r.boxes
                    if int(b.cls[0].cpu().numpy()) == node.target_id)
                dep_ok = depth is not None
                print(f"\r[F{frame_cnt:5d}] det={n_det} target_id={node.target_id}"
                      f"(found={n_target}) depth={'OK' if dep_ok else 'NONE'}"
                      f" phase={PHASE_LABEL[info['phase']][0]}"
                      f" lost={info['lost_frames']}"
                      f" move={'ON' if move_enabled else 'OFF'}"
                      f" vy={info['vy']:+.3f} vx={info['vx']:+.3f}",
                      end='', flush=True)

            draw_overlay(display, info, node, fps, move_enabled)

            if show_dep and depth is not None:
                dc   = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
                view = np.hstack([display, dc])
            else:
                view = display

            cv2.imshow(
                f"KFS Align MOVE | id={node.target_id}"
                f" | {'ON' if move_enabled else 'OFF'} | SPACE/r/q/+/-",
                view)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord(' '):
                move_enabled = not move_enabled
                if not move_enabled:
                    node.stop()
                print(f"Di chuyển: {'BẬT ✓' if move_enabled else 'TẮT'}")
            elif key == ord('r'):
                node.reset()
                move_enabled = False
                print("→ Reset → ALIGN_X  (di chuyển TẮT)")
            elif key == ord('s'):
                fname = f"align_move_{save_cnt:03d}.jpg"
                cv2.imwrite(fname, display)
                print(f"[LƯU] {fname}")
                save_cnt += 1
            elif key == ord('d'):
                show_dep = not show_dep
            elif key in (ord('+'), ord('=')):
                node.target_id += 1
                node.reset()
                move_enabled = False
                print(f"→ target_id = {node.target_id}")
            elif key == ord('-'):
                node.target_id = max(0, node.target_id - 1)
                node.reset()
                move_enabled = False
                print(f"→ target_id = {node.target_id}")

    finally:
        node.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        print("Robot đã dừng. Thoát.")


# ─────────────────────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(description='KFS Align + Move — robot thực sự di chuyển')
    p.add_argument('--model',     default='/home/robocon/ros_ws/yolo26n_best.engine')
    p.add_argument('--target-id', type=int,   default=1,     dest='target_id',
                   help='Class ID YOLO cần bám (mặc định 1)')
    p.add_argument('--dist',      type=float, default=300.0,
                   help='Khoảng cách dừng mm (mặc định 300)')
    p.add_argument('--conf',      type=float, default=0.5)
    run(p.parse_args())


if __name__ == '__main__':
    main()
