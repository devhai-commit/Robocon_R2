#!/usr/bin/env python3
"""
Test KFS Align — Visual Simulator (không cần ROS, không cần robot)

Dùng pyrealsense2 lấy color + depth trực tiếp, chạy YOLO và mô phỏng
toàn bộ logic căn chỉnh của kfs_align_grasp_server.py.
Hiển thị cmd_vel SẼ ĐƯỢC GỬI mà không thực sự di chuyển robot.

⚠  Phải dừng realsense ROS node trước khi chạy:
       ros2 lifecycle set /camera shutdown   (hoặc Ctrl+C node đó)

Chạy:
    python3 test_kfs_align_visual.py
    python3 test_kfs_align_visual.py --target-id 1 --dist 300
    python3 test_kfs_align_visual.py --model yolo26n_best.pt --conf 0.3

Phím:
    q  — thoát
    r  — reset về Phase 1 (ALIGN_X)
    s  — lưu ảnh hiện tại
    d  — bật/tắt depth colormap
    +/- — tăng/giảm target_id (để thử class khác)
"""

import argparse
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# ── Constants — PHẢI KHỚP với kfs_align_grasp_server.py ─────────────────────
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
class AlignSimulator:
    def __init__(self, target_id: int, desired_dist: float):
        self.target_id    = target_id
        self.desired_dist = desired_dist
        self.reset()

    def reset(self):
        self.phase          = PHASE_ALIGN_X
        self.aligned_frames = 0
        self.success_frames = 0
        self.lost_frames    = 0
        self.kf_initialized = False
        self.last_target_y  = 240
        self.last_vx        = 0.0
        self.last_vy        = 0.0

        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix   = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kf.transitionMatrix    = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        self.kf.processNoiseCov     = np.eye(4, dtype=np.float32) * 0.05
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5.0

    def update(self, yolo_results, depth_img):
        """Tính toán trạng thái một frame. Trả về dict để vẽ HUD."""
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
                self.last_vy = 0.0
                return info

            self.lost_frames = 0
            smooth_cx = self._kalman_update(raw_cx, raw_dist)
            err_x     = IMG_CENTER_X - smooth_cx
            info.update({'smooth_cx': smooth_cx, 'err_x': err_x})

            if abs(err_x) <= TOL_X_PX:
                self.aligned_frames += 1
                if self.aligned_frames >= ALIGNED_FRAMES_REQ:
                    self.phase          = PHASE_APPROACH
                    self.success_frames = 0
                    self.lost_frames    = 0
                    self.last_vy        = 0.0
            else:
                self.aligned_frames = 0
                v_y = err_x * KP_LATERAL
                if 0 < abs(v_y) < MIN_CREEP:
                    v_y = MIN_CREEP if v_y > 0 else -MIN_CREEP
                self.last_vy = max(min(v_y, MAX_LATERAL_SPEED), -MAX_LATERAL_SPEED)
                info['vy'] = self.last_vy

            info['phase'] = self.phase
            return info

        if self.phase == PHASE_APPROACH:
            dist_mm = self._median_depth(depth_img, int(IMG_CENTER_X), self.last_target_y)
            info['raw_dist'] = dist_mm

            if dist_mm <= 0:
                self.lost_frames += 1
                self.last_vx = 0.0
                return info

            self.lost_frames = 0
            err_d = dist_mm - self.desired_dist
            info['err_dist'] = err_d

            if abs(err_d) < TOL_DIST_MM:
                self.success_frames += 1
                if self.success_frames >= SUCCESS_FRAMES_REQ:
                    self.phase   = PHASE_DONE
                    self.last_vx = 0.0
            else:
                self.success_frames = 0
                v_x = err_d * KP_LINEAR_MM
                if 0 < abs(v_x) < MIN_CREEP:
                    v_x = MIN_CREEP if v_x > 0 else -MIN_CREEP
                self.last_vx = max(min(v_x, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED)
                info['vx'] = self.last_vx

            info['phase'] = self.phase
            return info

        return info

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _find_target(self, results, depth_img):
        for r in results:
            for box in r.boxes:
                if int(box.cls[0].cpu().numpy()) != self.target_id:
                    continue
                b  = box.xyxy[0].cpu().numpy()
                cx = float((b[0] + b[2]) / 2)
                cy = int((b[1] + b[3]) / 2)
                dist = self._median_depth(depth_img, int(cx), cy)
                if abs(IMG_CENTER_X - cx) <= 200.0 and 0 < dist < 1200.0:
                    self.last_target_y = cy
                    return cx, dist, True, b
        return 0.0, 0.0, False, None

    def _median_depth(self, depth_img, cx, cy, half=5):
        if depth_img is None:
            return 0.0
        h, w  = depth_img.shape
        y0, y1 = max(0, cy - half), min(h - 1, cy + half)
        x0, x1 = max(0, cx - half), min(w - 1, cx + half)
        patch  = depth_img[y0:y1+1, x0:x1+1]
        valid  = patch[patch > 0]
        return float(np.median(valid)) if len(valid) >= 4 else 0.0

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
def draw_overlay(frame, info: dict, target_id: int, desired_dist: float, fps: float):
    h, w         = frame.shape[:2]
    phase        = info['phase']
    label, color = PHASE_LABEL.get(phase, ("???", (128, 128, 128)))
    cx_int       = int(IMG_CENTER_X)

    # Đường tâm ảnh
    cv2.line(frame, (cx_int, 0), (cx_int, h), (60, 60, 200), 1)
    cv2.putText(frame, f"cx={IMG_CENTER_X:.0f}", (cx_int + 4, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 200), 1)

    # Vùng tolerance ±TOL_X_PX
    tol = int(TOL_X_PX)
    cv2.rectangle(frame,
                  (cx_int - tol, h // 2 - 6),
                  (cx_int + tol, h // 2 + 6),
                  (80, 80, 80), 1)

    # Bounding box + tâm + đường lệch
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

    # HUD góc trái trên
    lines = [
        (f"FPS: {fps:.1f}",                                        (0, 255, 0)),
        (f"Phase : {label}",                                        color),
        (f"Target: id={target_id}  goal={desired_dist:.0f}mm",     (210, 210, 210)),
        (f"Depth : {info['raw_dist']:.0f}mm  err={info['err_dist']:+.0f}mm",
                                                                    (210, 210, 210)),
        (f"Align {info['aligned_frames']:2d}/{ALIGNED_FRAMES_REQ}"
         f"  Succ {info['success_frames']:2d}/{SUCCESS_FRAMES_REQ}"
         f"  Lost {info['lost_frames']}",                           (210, 210, 210)),
        (f"cmd_vel  vx={info['vx']:+.3f}  vy={info['vy']:+.3f}",   (255, 200, 50)),
    ]
    for i, (text, col) in enumerate(lines):
        cv2.putText(frame, text, (10, 22 + i * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

    # Thanh lệch ngang ở đáy
    bar_y  = h - 20
    center = w // 2
    bar_w  = min(int(abs(info['err_x']) / (w / 2) * (w // 2)), w // 2)
    if info['err_x'] > 0:
        cv2.rectangle(frame, (center, bar_y), (center + bar_w, bar_y + 14), (0, 140, 255), -1)
    else:
        cv2.rectangle(frame, (center - bar_w, bar_y), (center, bar_y + 14), (0, 140, 255), -1)
    cv2.line(frame, (center, bar_y - 4), (center, bar_y + 18), (255, 255, 255), 1)

    # Banner ALIGNED
    if phase == PHASE_DONE:
        text = "=== ALIGNED! READY TO GRASP ==="
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
        tx = (w - tw) // 2
        ty = h // 2
        cv2.rectangle(frame, (tx - 8, ty - th - 6), (tx + tw + 8, ty + 8), (0, 0, 0), -1)
        cv2.putText(frame, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)


# ─────────────────────────────────────────────────────────────────────────────
def run(args):
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
    print("Khởi động RealSense...")
    try:
        pipe.start(cfg)
    except Exception as e:
        print(f"[LỖI] Không mở được camera: {e}")
        print("  → Kiểm tra xem realsense ROS node có đang chạy không.")
        print("  → Nếu có, dừng nó trước rồi chạy lại script này.")
        return

    align = rs.align(rs.stream.color)

    print(f"Nạp model: {args.model}")
    model = YOLO(args.model, task='detect')
    print("Class names (10 đầu):", {k: v for k, v in list(model.names.items())[:10]})

    sim       = AlignSimulator(args.target_id, args.dist)
    fps       = 0.0
    frame_cnt = 0
    fps_t     = time.time()
    save_cnt  = 0
    show_dep  = False

    print(f"\nSimulating align: target_id={args.target_id}  dist={args.dist}mm")
    print("Phím: q=thoát  r=reset  s=lưu  d=depth  +/-=đổi target_id\n")

    try:
        while True:
            frames      = pipe.wait_for_frames()
            aligned     = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            img       = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(depth_frame.get_data()).astype(np.uint16)
            display   = img.copy()

            results = model(img, verbose=False, conf=args.conf)
            info    = sim.update(results, depth_img)

            frame_cnt += 1
            if frame_cnt % 30 == 0:
                fps   = 30.0 / (time.time() - fps_t)
                fps_t = time.time()

            draw_overlay(display, info, sim.target_id, sim.desired_dist, fps)

            if show_dep:
                dc   = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET)
                view = np.hstack([display, dc])
            else:
                view = display

            cv2.imshow(
                f"KFS Align Sim | phase={PHASE_LABEL[info['phase']][0]}"
                f" | id={sim.target_id} | q/r/s/d/+/-",
                view)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('r'):
                sim.reset()
                print("→ Reset → ALIGN_X")
            elif key == ord('s'):
                fname = f"align_test_{save_cnt:03d}.jpg"
                cv2.imwrite(fname, display)
                print(f"[LƯU] {fname}")
                save_cnt += 1
            elif key == ord('d'):
                show_dep = not show_dep
            elif key in (ord('+'), ord('=')):
                sim.target_id += 1
                sim.reset()
                print(f"→ target_id = {sim.target_id}")
            elif key == ord('-'):
                sim.target_id = max(0, sim.target_id - 1)
                sim.reset()
                print(f"→ target_id = {sim.target_id}")

    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        print("Đã thoát.")


# ─────────────────────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(description='KFS Align Visual Simulator')
    p.add_argument('--model',     default='/home/robocon/ros_ws/yolo26n_best.engine')
    p.add_argument('--target-id', type=int,   default=1,     dest='target_id',
                   help='Class ID YOLO cần bám (mặc định 1)')
    p.add_argument('--dist',      type=float, default=300.0,
                   help='Khoảng cách dừng mong muốn mm (mặc định 300)')
    p.add_argument('--conf',      type=float, default=0.5)
    run(p.parse_args())


if __name__ == '__main__':
    main()
