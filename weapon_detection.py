# -*- coding: utf-8 -*-
# ==========================================
# K230 - Load best.kmodel, object detect 1 class, gui offset_x sang Jetson
#
# K230 (chay file nay) -> UART -> Jetson
#
# Moi dong gui di la 1 JSON ket thuc bang '\n', vi du:
#   {"status":"detect","cx":322.0,"cy":180.5,"offset_x":2.0,
#    "direction":"GIUA","centered":true,"area":12345,"score":0.87,"n_obj":1}
#   {"status":"empty"}
#   {"status":"alive","uptime_frame":120}
#   {"system":"booting"|"warming_up"|"ready"|"stopped_manually"}
#   {"error":"..."}
#
# !!! CHU Y CHAN UART !!!
# UART1: TX=pin3, RX=pin4  (mac dinh duoi day)
# UART2: TX=pin11, RX=pin12
# Sua UART_NUM va UART_TX_PIN/UART_RX_PIN cho phu hop board cua ban.
# ==========================================
from libs.PipeLine import PipeLine, ScopedTiming
from libs.YOLO import YOLO11
from libs.Utils import *
from machine import UART, FPIOA
import os, sys, gc, json, time

gc.threshold(80000)

# ==========================================
# CAU HINH UART
# ==========================================
UART_NUM      = 1
UART_BAUDRATE = 115200
UART_TX_PIN   = 9
UART_RX_PIN   = 10
DEBUG_PRINT   = True

uart = None
try:
    fpioa = FPIOA()
    if UART_NUM == 1:
        fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
        fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
    elif UART_NUM == 2:
        fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
        fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
    uart = UART(UART_NUM, baudrate=UART_BAUDRATE)
    if DEBUG_PRINT:
        print("[HE THONG] Da mo UART" + str(UART_NUM) + " @ " + str(UART_BAUDRATE) + " bps")
except Exception as e:
    print("[LOI] Khong khoi tao duoc UART: " + str(e))
    uart = None


def send_uart(data_dict):
    """Dong goi JSON, them xuong dong, gui qua UART."""
    try:
        data_str = json.dumps(data_dict)
    except Exception:
        data_str = '{"error":"serialize_fail"}'
    if DEBUG_PRINT:
        print("UART SEND:", data_str)
    if uart:
        try:
            uart.write((data_str + "\n").encode('utf-8'))
        except Exception as e:
            if DEBUG_PRINT:
                print("[LOI UART WRITE]", str(e))


# ==========================================
# DO PHAN GIAI VA VUNG TAM
# ==========================================
# QUAN TRONG:
#   libs/YOLO.YOLO11 scale boxes ve khong gian DISPLAY (DISP_W x DISP_H),
#   KHONG phai khong gian PROC. Vi vay cx, cy, w, h tu yolo.run() da o DISP.
#   => Tinh tam khung va ve OSD truc tiep trong khong gian DISP, KHONG scale lai.
PROC_W = 640
PROC_H = 360
DISP_W = 640
DISP_H = 480

# Tam khung tinh theo DISP vi boxes da o DISP space
CAM_CENTER_X = float(DISP_W) / 2.0
CAM_CENTER_Y = float(DISP_H) / 2.0

CENTER_TOLERANCE = 30.0   # pixel - khoang dung sai coi nhu "o giua"

# ==========================================
# VUNG NHAN DIEN (ROI) - tinh theo khong gian DISP
# Chi chap nhan detection co TAM nam trong vung nay.
# Giam ty le de thu hep vung nhan dien.
# ==========================================
ROI_W_RATIO = 0.5    # 50% chieu ngang (lay giua)
ROI_H_RATIO = 0.5    # 60% chieu doc   (lay giua)

_roi_half_w = DISP_W * ROI_W_RATIO / 2.0
_roi_half_h = DISP_H * ROI_H_RATIO / 2.0
ROI_X1 = CAM_CENTER_X - _roi_half_w
ROI_Y1 = CAM_CENTER_Y - _roi_half_h
ROI_X2 = CAM_CENTER_X + _roi_half_w
ROI_Y2 = CAM_CENTER_Y + _roi_half_h


def in_roi(cx, cy):
    return (ROI_X1 <= cx <= ROI_X2) and (ROI_Y1 <= cy <= ROI_Y2)

# ==========================================
# CAU HINH YOLO
# ==========================================
LABELS = ["Target"]                          # Chi 1 class
KMODEL_PATH      = "/sdcard/kmodel/yolo11m.kmodel"
MODEL_INPUT_SIZE = [320, 320]
DISPLAY_MODE     = "lcd"

CONF_THRESHOLD = 0.4
NMS_THRESHOLD  = 0.2
MAX_BOXES      = 1

# ==========================================
# CAU HINH TRUYEN DU LIEU
# ==========================================
PERIODIC_SEND_EVERY = 5
HEARTBEAT_SEC       = 2.0
GC_EVERY_N_FRAME    = 100

# Debounce: chi gui "detect" khi cx on dinh >= N frame lien tiep
STABLE_MIN_FRAMES     = 3
STABLE_CX_TOLERANCE   = 20.0   # pixel — cx dao dong trong vung nay van coi la on dinh
# Debounce: chi gui "empty" khi mat object >= N frame lien tiep
EMPTY_DEBOUNCE_FRAMES = 5


def get_detections(res):
    dets = []
    if res is None or len(res) < 3:
        return dets

    boxes   = res[0]
    cls_ids = res[1]
    scores  = res[2]

    n = len(boxes)
    if n == 0:
        return dets

    for i in range(n):
        try:
            score  = float(scores[i])
            cls_id = int(cls_ids[i])
            if score < CONF_THRESHOLD:
                continue
            box = boxes[i]
            x = float(box[0])
            y = float(box[1])
            w = float(box[2])
            h = float(box[3])
            if w <= 0.0 or h <= 0.0:
                continue
            cx   = x + w / 2.0
            cy   = y + h / 2.0
            if not in_roi(cx, cy):
                continue
            area = w * h
            if cls_id < len(LABELS):
                name = LABELS[cls_id].upper()
            else:
                name = "CLS" + str(cls_id)
            dets.append({
                "name"  : name,
                "cx"    : cx,
                "cy"    : cy,
                "x"     : x,
                "y"     : y,
                "w"     : w,
                "h"     : h,
                "area"  : area,
                "score" : score,
            })
        except Exception as e:
            send_uart({"error": "Loi det " + str(i) + ": " + str(e)})
            continue
    return dets


def find_nearest(dets):
    """Tra ve detect co cx gan tam khung hinh nhat."""
    if not dets:
        return None
    best = dets[0]
    for d in dets[1:]:
        if abs(d["cx"] - CAM_CENTER_X) < abs(best["cx"] - CAM_CENTER_X):
            best = d
    return best


def centering_status(cx):
    """Tra ve (offset_x, centered, direction)."""
    offset_x = float(cx) - CAM_CENTER_X
    centered = abs(offset_x) <= CENTER_TOLERANCE
    if centered:
        direction = "GIUA"
    elif offset_x < 0.0:
        direction = "TRAI"
    else:
        direction = "PHAI"
    return offset_x, centered, direction


def build_detect_payload(nearest, offset_x, centered, direction, n_obj):
    return {
        "status"   : "detect",
        "target"   : nearest["name"],
        "cx"       : round(nearest["cx"], 1),
        "cy"       : round(nearest["cy"], 1),
        "offset_x" : round(offset_x, 1),
        "direction": direction,
        "centered" : centered,
        "area"     : int(nearest["area"]),
        "score"    : round(nearest["score"], 2),
        "n_obj"    : n_obj,
    }


if __name__ == "__main__":
    send_uart({"system": "booting"})
    gc.collect()

    try:
        os.stat(KMODEL_PATH)
    except OSError:
        send_uart({"error": "Khong tim thay model: " + KMODEL_PATH})
        sys.exit()

    pl   = None
    yolo = None

    try:
        yolo = YOLO11(
            task_type="detect", mode="video", kmodel_path=KMODEL_PATH,
            labels=LABELS,
            rgb888p_size=[PROC_W, PROC_H],
            model_input_size=MODEL_INPUT_SIZE,
            display_size=[DISP_W, DISP_H],
            conf_thresh=CONF_THRESHOLD,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES,
            debug_mode=0,
        )
        yolo.config_preprocess()

        pl = PipeLine(rgb888p_size=[PROC_W, PROC_H], display_size=[DISP_W, DISP_H], display_mode=DISPLAY_MODE)
        pl.create()

        last_printed_state = None
        frame_idx          = 0
        last_heartbeat     = time.ticks_ms()
        stable_cx          = None   # cx cua target dang theo doi
        stable_count       = 0      # so frame cx on dinh lien tiep
        empty_count        = 0      # so frame lien tiep khong co object

        while True:
            with ScopedTiming("total", 0):
                img = pl.get_frame()
                frame_idx += 1

                res = yolo.run(img)
                # Xoa OSD truoc khi tu ve (tranh ve chong len frame cu)
                try:
                    pl.osd_img.clear()
                except Exception:
                    pass

                # ----- Warm-up 60 frame dau -----
                if frame_idx < 60:
                    if frame_idx == 1:
                        send_uart({"system": "warming_up"})
                    pl.show_image()
                    del img, res
                    continue
                elif frame_idx == 60:
                    send_uart({
                        "system"    : "ready",
                        "proc_space": [PROC_W, PROC_H],
                        "cam_center": [CAM_CENTER_X, CAM_CENTER_Y],
                        "tolerance" : CENTER_TOLERANCE,
                    })

                if frame_idx % GC_EVERY_N_FRAME == 0:
                    gc.collect()

                dets = get_detections(res)
                nearest = find_nearest(dets)

                # ----- Debounce: kiem tra on dinh cx -----
                if nearest:
                    empty_count = 0
                    cx_now = nearest["cx"]
                    if stable_cx is None or abs(cx_now - stable_cx) > STABLE_CX_TOLERANCE:
                        # cx nhay qua object khac hoac lan dau detect -> reset dem
                        stable_cx    = cx_now
                        stable_count = 1
                    else:
                        stable_count += 1
                        stable_cx = cx_now  # cap nhat dan
                else:
                    stable_cx    = None
                    stable_count = 0
                    empty_count += 1

                # Chi duoc gui "detect" khi cx da on dinh du N frame
                detect_ready = (nearest is not None and stable_count >= STABLE_MIN_FRAMES)
                # Chi duoc gui "empty" khi object mat du N frame
                empty_ready  = (nearest is None and empty_count >= EMPTY_DEBOUNCE_FRAMES)

                if detect_ready:
                    offset_x, centered, direction = centering_status(nearest["cx"])
                    state_key = (centered, direction)
                    state_changed = (state_key != last_printed_state)
                    periodic_send = (frame_idx % PERIODIC_SEND_EVERY == 0)
                    if state_changed or periodic_send:
                        last_printed_state = state_key
                        send_uart(build_detect_payload(
                            nearest, offset_x, centered, direction, len(dets)
                        ))
                elif empty_ready:
                    if last_printed_state != ("TRONG",):
                        last_printed_state = ("TRONG",)
                        send_uart({"status": "empty"})
                else:
                    # Dang trong thoi gian debounce — khong gui gi
                    nearest   = None
                    offset_x  = 0.0
                    centered  = False
                    direction = "NONE"

                # ----- Heartbeat khi rong -----
                now_ms = time.ticks_ms()
                if not dets and time.ticks_diff(now_ms, last_heartbeat) > int(HEARTBEAT_SEC * 1000):
                    send_uart({"status": "alive", "uptime_frame": frame_idx})
                    last_heartbeat = now_ms
                elif dets:
                    last_heartbeat = now_ms

                # ==========================================
                # VE LEN MAN HINH (toa do trong khong gian DISP)
                # cx, cy tu yolo.run() da o khong gian DISP nen KHONG scale lai
                # ==========================================
                left_bound  = int(CAM_CENTER_X - CENTER_TOLERANCE)
                right_bound = int(CAM_CENTER_X + CENTER_TOLERANCE)
                cam_cx_int  = int(CAM_CENTER_X)
                disp_h_int  = int(DISP_H)

                pl.osd_img.draw_line(left_bound,  0, left_bound,  disp_h_int - 1, color=(0, 255, 0), thickness=1)
                pl.osd_img.draw_line(right_bound, 0, right_bound, disp_h_int - 1, color=(0, 255, 0), thickness=1)
                pl.osd_img.draw_line(cam_cx_int,  0, cam_cx_int,  disp_h_int - 1, color=(255, 255, 0), thickness=2)

                # Khung ROI (vung nhan dien)
                roi_x1 = int(ROI_X1); roi_y1 = int(ROI_Y1)
                roi_x2 = int(ROI_X2); roi_y2 = int(ROI_Y2)
                pl.osd_img.draw_rectangle(
                    roi_x1, roi_y1, roi_x2 - roi_x1, roi_y2 - roi_y1,
                    color=(0, 200, 255), thickness=2
                )

                for d in dets:
                    off, cent, _ = centering_status(d["cx"])
                    box_color = (0, 255, 0) if cent else (255, 80, 0)
                    bx = int(d["x"]); by = int(d["y"])
                    bw = int(d["w"]); bh = int(d["h"])
                    pl.osd_img.draw_rectangle(bx, by, bw, bh, color=box_color, thickness=2)
                    label_txt = d["name"] + " " + ("%.2f" % d["score"])
                    pl.osd_img.draw_string_advanced(bx, max(0, by - 20), 18, label_txt, color=box_color)
                    pl.osd_img.draw_circle(int(d["cx"]), int(d["cy"]), 5, color=box_color, thickness=-1)

                if dets:
                    n_obj = find_nearest(dets)
                    off, cent, direction = centering_status(n_obj["cx"])
                    sign3 = "+" if off >= 0.0 else ""
                    if cent:
                        status_str = "GIUA (lech " + sign3 + ("%.2f" % off) + "px)"
                        bar_color  = (0, 255, 0)
                    else:
                        dist_to_zone = abs(off) - CENTER_TOLERANCE if off < 0.0 else off - CENTER_TOLERANCE
                        side_txt   = "PHAI" if off < 0.0 else "TRAI"
                        status_str = "Can " + side_txt + " " + ("%.2f" % dist_to_zone) + "px"
                        bar_color  = (255, 80, 0)
                    pl.osd_img.draw_string_advanced(
                        4, disp_h_int - 30, 24,
                        n_obj["name"] + " | " + status_str,
                        color=bar_color
                    )

                pl.show_image()
                del img, res

    except KeyboardInterrupt:
        send_uart({"system": "stopped_manually"})
    except Exception as e:
        send_uart({"error": "Exception: " + str(e)})
    finally:
        if yolo:
            yolo.deinit()
        if pl:
            pl.destroy()
        if uart:
            try:
                uart.deinit()
            except Exception:
                pass
        gc.collect()
