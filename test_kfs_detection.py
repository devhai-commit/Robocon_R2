#!/usr/bin/env python3
"""
Test KFS Detection — không cần ROS, chạy thẳng trên Jetson Orin.

Dùng để:
  - Kiểm tra model nhận diện đúng class chưa
  - Xác nhận thứ tự class_id (0=r1_kfs? 1=r2_kfs? ...)
  - Đo FPS thực tế của TensorRT engine
  - Test với ảnh tĩnh hoặc camera live

Chạy:
  python3 test_kfs_detection.py                         # camera id=4 (mặc định)
  python3 test_kfs_detection.py --source 0              # camera id=0
  python3 test_kfs_detection.py --source photo.jpg      # ảnh tĩnh
  python3 test_kfs_detection.py --model yolo26n_best.pt # dùng .pt thay .engine
  python3 test_kfs_detection.py --conf 0.3              # hạ threshold
"""

import argparse
import time
import cv2
from ultralytics import YOLO

# Chỉnh lại nếu model train với thứ tự class khác
CLASS_NAMES = {
    0: 'r1_kfs',
    1: 'r2_kfs',
    2: 'fake_kfs',
}

CLASS_COLORS = {
    'r1_kfs':   (0,   200,  0),   # xanh lá
    'r2_kfs':   (0,   120, 255),  # cam
    'fake_kfs': (0,     0, 220),  # đỏ
    'unknown':  (180, 180, 180),  # xám
}


# ------------------------------------------------------------------
def draw_boxes(frame, boxes, conf_thr):
    counts = {'r1_kfs': 0, 'r2_kfs': 0, 'fake_kfs': 0}
    for box in boxes:
        conf     = float(box.conf[0].cpu().numpy())
        if conf < conf_thr:
            continue
        class_id = int(box.cls[0].cpu().numpy())
        b        = box.xyxy[0].cpu().numpy().astype(int)
        name     = CLASS_NAMES.get(class_id, f'class_{class_id}')
        color    = CLASS_COLORS.get(name, CLASS_COLORS['unknown'])

        cv2.rectangle(frame, (b[0], b[1]), (b[2], b[3]), color, 2)

        label = f'{name} {conf:.2f}'
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frame, (b[0], b[1] - th - 8), (b[0] + tw + 4, b[1]), color, -1)
        cv2.putText(frame, label, (b[0] + 2, b[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cx = (b[0] + b[2]) // 2
        cy = (b[1] + b[3]) // 2
        cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 12, 2)

        if name in counts:
            counts[name] += 1
    return counts


def draw_stats(frame, fps, counts):
    lines = [
        f'FPS: {fps:.1f}',
        f'R1 KFS : {counts["r1_kfs"]}',
        f'R2 KFS : {counts["r2_kfs"]}',
        f'FAKE   : {counts["fake_kfs"]}',
    ]
    for i, line in enumerate(lines):
        color = (0, 255, 0) if i == 0 else (200, 200, 200)
        cv2.putText(frame, line, (10, 25 + i * 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)


# ------------------------------------------------------------------
def run_image(model, path, conf_thr):
    frame = cv2.imread(path)
    if frame is None:
        print(f'[LỖI] Không đọc được ảnh: {path}')
        return

    results = model(frame, verbose=False, conf=conf_thr)
    counts  = draw_boxes(frame, results[0].boxes, conf_thr)
    draw_stats(frame, 0, counts)

    print('\n=== Kết quả nhận diện ===')
    for box in results[0].boxes:
        class_id = int(box.cls[0].cpu().numpy())
        conf     = float(box.conf[0].cpu().numpy())
        b        = box.xyxy[0].cpu().numpy()
        cx, cy   = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
        name     = CLASS_NAMES.get(class_id, f'class_{class_id}')
        flag     = '  ⚠ FAKE' if name == 'fake_kfs' else ''
        print(f'  [{class_id}] {name:10s}  conf={conf:.3f}  tâm=({cx:.0f},{cy:.0f}){flag}')

    cv2.imshow('KFS Detection — nhấn bất kỳ để thoát', frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_camera(model, camera_id, conf_thr):
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print(f'[LỖI] Không mở được camera ID={camera_id}')
        return

    print(f"Camera ID={camera_id} mở thành công. conf={conf_thr}")
    print("Phím: 'q' thoát | 's' lưu ảnh")

    fps_t     = time.time()
    fps       = 0.0
    frame_cnt = 0
    save_cnt  = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        results = model(frame, verbose=False, conf=conf_thr)
        counts  = draw_boxes(frame, results[0].boxes, conf_thr)

        frame_cnt += 1
        if frame_cnt % 30 == 0:
            fps   = 30.0 / (time.time() - fps_t)
            fps_t = time.time()

        draw_stats(frame, fps, counts)

        if sum(counts.values()) > 0:
            summary = '  '.join(f'{k}={v}' for k, v in counts.items() if v > 0)
            print(f'\rFPS={fps:.1f}  {summary}   ', end='', flush=True)

        cv2.imshow('KFS Detection (q=thoat, s=luu)', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            fname = f'kfs_capture_{save_cnt:03d}.jpg'
            cv2.imwrite(fname, frame)
            print(f'\n[LƯU] {fname}')
            save_cnt += 1

    cap.release()
    cv2.destroyAllWindows()
    print('\nĐã thoát.')


# ------------------------------------------------------------------
def print_model_info(model):
    """In class names từ model để xác nhận thứ tự class_id."""
    print('\n=== Class names trong model ===')
    try:
        for cid, cname in sorted(model.names.items()):
            local = CLASS_NAMES.get(cid, '???')
            ok    = '✓' if local == cname else f'→ map về "{local}"'
            print(f'  [{cid}] model="{cname}"  {ok}')
    except Exception as e:
        print(f'Không lấy được names: {e}')
    print('================================\n')


# ------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Test KFS detection — Robocon 2026')
    parser.add_argument('--model',  default='/home/robocon/ros_ws/yolo26n_best.engine')
    parser.add_argument('--source', default='4', help='Camera ID hoặc đường dẫn ảnh')
    parser.add_argument('--conf',   type=float, default=0.5)
    args = parser.parse_args()

    print(f'Nạp model: {args.model}')
    model = YOLO(args.model, task='detect')
    print_model_info(model)

    try:
        run_camera(model, int(args.source), args.conf)
    except ValueError:
        run_image(model, args.source, args.conf)


if __name__ == '__main__':
    main()
