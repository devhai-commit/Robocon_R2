#!/usr/bin/env python3
"""
Demo: KFS 3D Position Detection
RealSense D-series + YOLO → tọa độ 3D (X, Y, Z) của KFS tính bằng mét.

Cài đặt nếu chưa có:
    pip install pyrealsense2

Chạy:
    python3 demo_kfs_3d.py
    python3 demo_kfs_3d.py --model yolo26n_best.pt   # dùng .pt nếu .engine lỗi
    python3 demo_kfs_3d.py --conf 0.4

Phím:
    q — thoát
    s — lưu ảnh hiện tại
    d — bật/tắt depth colormap cạnh ảnh màu
"""

import argparse
import time
import numpy as np
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO

# ---------- Class mapping (chỉnh nếu model train thứ tự khác) ----------
CLASS_NAMES = {0: 'r1_kfs', 1: 'r2_kfs', 2: 'fake_kfs'}
CLASS_COLORS = {
    'r1_kfs':   (0,   210,   0),
    'r2_kfs':   (0,   120, 255),
    'fake_kfs': (0,     0, 220),
    'unknown':  (160, 160, 160),
}

DEPTH_MIN_M = 0.15   # gần hơn thì không tin cậy
DEPTH_MAX_M = 4.00


# -----------------------------------------------------------------------
def median_depth(depth_frame, cx, cy, half=5):
    """
    Lấy median depth trong vùng (2*half+1)² quanh tâm bounding box.
    Bỏ các điểm = 0 (không đo được). Trả về mét, hoặc 0.0 nếu không đủ data.
    """
    w = depth_frame.get_width()
    h = depth_frame.get_height()
    x0, x1 = max(0, cx - half), min(w - 1, cx + half)
    y0, y1 = max(0, cy - half), min(h - 1, cy + half)

    samples = [
        depth_frame.get_distance(x, y)
        for y in range(y0, y1 + 1)
        for x in range(x0, x1 + 1)
        if DEPTH_MIN_M < depth_frame.get_distance(x, y) < DEPTH_MAX_M
    ]
    return float(np.median(samples)) if len(samples) >= 4 else 0.0


def to_3d(intrinsics, cx, cy, depth_m):
    """Deproject pixel → 3D point [X, Y, Z] trong camera frame (mét)."""
    return rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth_m)


# -----------------------------------------------------------------------
def draw_detection(frame, box, name, color, depth_m, xyz):
    b    = box.xyxy[0].cpu().numpy().astype(int)
    conf = float(box.conf[0].cpu().numpy())
    cx   = int((b[0] + b[2]) / 2)
    cy   = int((b[1] + b[3]) / 2)

    cv2.rectangle(frame, (b[0], b[1]), (b[2], b[3]), color, 2)
    cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 14, 2)

    line1 = f'{name}  {conf:.2f}'
    line2 = (f'Z={depth_m:.2f}m  X={xyz[0]:+.2f}  Y={xyz[1]:+.2f}'
             if depth_m > 0 else 'depth: N/A')

    font = cv2.FONT_HERSHEY_SIMPLEX
    for i, line in enumerate([line2, line1]):   # line1 ở trên cùng
        (tw, th), _ = cv2.getTextSize(line, font, 0.55, 2)
        ty = b[1] - 8 - i * (th + 8)
        cv2.rectangle(frame, (b[0], ty - th - 2), (b[0] + tw + 4, ty + 4), color, -1)
        cv2.putText(frame, line, (b[0] + 2, ty), font, 0.55, (255, 255, 255), 2)


def draw_hud(frame, fps, counts, show_depth):
    lines = [
        f'FPS : {fps:.1f}',
        f'R1  : {counts.get("r1_kfs", 0)}',
        f'R2  : {counts.get("r2_kfs", 0)}',
        f'FAKE: {counts.get("fake_kfs", 0)}',
        f'[d] depth: {"ON" if show_depth else "OFF"}',
    ]
    for i, line in enumerate(lines):
        cv2.putText(frame, line, (10, 25 + i * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if i == 0 else (210, 210, 210), 2)


# -----------------------------------------------------------------------
def run(model_path, conf_thr):
    # --- RealSense ---
    pipeline = rs.pipeline()
    cfg      = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)

    print("Khởi động RealSense pipeline...")
    profile    = pipeline.start(cfg)
    intrinsics = (profile.get_stream(rs.stream.color)
                         .as_video_stream_profile()
                         .get_intrinsics())
    print(f"  Intrinsics: fx={intrinsics.fx:.1f} fy={intrinsics.fy:.1f} "
          f"ppx={intrinsics.ppx:.1f} ppy={intrinsics.ppy:.1f}")

    align = rs.align(rs.stream.color)   # căn depth về color frame

    # --- YOLO ---
    print(f"Nạp model: {model_path}")
    model = YOLO(model_path, task='detect')
    print("Sẵn sàng!  q=thoát  s=lưu  d=depth map\n")

    fps_t      = time.time()
    fps        = 0.0
    frame_cnt  = 0
    save_cnt   = 0
    show_depth = False

    try:
        while True:
            frames      = pipeline.wait_for_frames()
            aligned     = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            img     = np.asanyarray(color_frame.get_data())   # BGR 640×480
            display = img.copy()
            results = model(img, verbose=False, conf=conf_thr)
            counts  = {'r1_kfs': 0, 'r2_kfs': 0, 'fake_kfs': 0}

            for box in results[0].boxes:
                class_id = int(box.cls[0].cpu().numpy())
                name     = CLASS_NAMES.get(class_id, f'class_{class_id}')
                color    = CLASS_COLORS.get(name, CLASS_COLORS['unknown'])

                b       = box.xyxy[0].cpu().numpy().astype(int)
                cx      = int((b[0] + b[2]) / 2)
                cy      = int((b[1] + b[3]) / 2)
                depth_m = median_depth(depth_frame, cx, cy)
                xyz     = to_3d(intrinsics, cx, cy, depth_m) if depth_m > 0 else [0, 0, 0]

                draw_detection(display, box, name, color, depth_m, xyz)

                if 'fake' in name:
                    print(f'\r⚠  FAKE KFS ({cx},{cy}) Z={depth_m:.2f}m — BỎ QUA       ',
                          end='', flush=True)
                else:
                    counts[name] = counts.get(name, 0) + 1
                    if depth_m > 0:
                        print(f'\r[{name}] Z={depth_m:.2f}m  '
                              f'X={xyz[0]:+.3f}m  Y={xyz[1]:+.3f}m       ',
                              end='', flush=True)

            # FPS
            frame_cnt += 1
            if frame_cnt % 30 == 0:
                fps   = 30.0 / (time.time() - fps_t)
                fps_t = time.time()

            # Hiển thị
            if show_depth:
                depth_img   = np.asanyarray(depth_frame.get_data())
                depth_color = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET)
                view = np.hstack([display, depth_color])
                draw_hud(view, fps, counts, show_depth)
            else:
                view = display
                draw_hud(view, fps, counts, show_depth)

            cv2.imshow('KFS 3D Detection  (q=thoat d=depth s=luu)', view)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s'):
                fname = f'depth_kfs_{save_cnt:03d}.jpg'
                cv2.imwrite(fname, display)
                print(f'\n[LƯU] {fname}')
                save_cnt += 1
            elif key == ord('d'):
                show_depth = not show_depth

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print('\nCamera đã dừng.')


# -----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Demo KFS 3D — RealSense + YOLO')
    parser.add_argument('--model', default='/home/robocon/ros_ws/yolo26n_best.engine')
    parser.add_argument('--conf',  type=float, default=0.5)
    args = parser.parse_args()
    run(args.model, args.conf)


if __name__ == '__main__':
    main()
