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
PROC_W = 640
PROC_H = 360
DISP_W = 640
DISP_H = 480

CAM_CENTER_X = float(PROC_W) / 2.0
CAM_CENTER_Y = float(PROC_H) / 2.0

CENTER_TOLERANCE = 30.0   # pixel - khoang dung sai coi nhu "o giua"

# ==========================================
# CAU HINH YOLO
# ==========================================
LABELS = ["Target"]                          # Chi 1 class
KMODEL_PATH      = "/sdcard/kmodel/best.kmodel"
MODEL_INPUT_SIZE = [320, 320]
DISPLAY_MODE     = "lcd"

CONF_THRESHOLD = 0.45
NMS_THRESHOLD  = 0.2
MAX_BOXES      = 10

# ==========================================
# CAU HINH TRUYEN DU LIEU
# ==========================================
PERIODIC_SEND_EVERY = 5
HEARTBEAT_SEC       = 2.0
GC_EVERY_N_FRAME    = 100


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
            area = w * h
            if cls_id < len(LABELS):
                name = LABELS[cls_id].upper()
            else:
                name = "CLS" + str(cls_id)
            dets.append({
                "name"  : name,
                "cx"    : cx,
                "cy"    : cy,
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
            display_size=[PROC_W, PROC_H],
            conf_thresh=CONF_THRESHOLD,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES,
            debug_mode=0,
        )
        yolo.config_preprocess()

        pl = PipeLine(rgb888p_size=[PROC_W, PROC_H], display_mode=DISPLAY_MODE)
        pl.create()

        last_printed_state = None
        stable_count       = 0
        prev_layout        = []
        frame_idx          = 0
        last_heartbeat     = time.ticks_ms()

        while True:
            with ScopedTiming("total", 0):
                img = pl.get_frame()
                frame_idx += 1

                res = yolo.run(img)
                yolo.draw_result(res, pl.osd_img)

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

                # ----- Kiem tra layout on dinh -----
                layout_names = [d["name"] for d in sorted(dets, key=lambda d: d["cx"])]
                if layout_names == prev_layout:
                    stable_count += 1
                else:
                    prev_layout  = layout_names
                    stable_count = 1

                # ----- Quyet dinh gui du lieu -----
                if stable_count >= 3:
                    nearest = find_nearest(dets)

                    if nearest:
                        offset_x, centered, direction = centering_status(nearest["cx"])
                        state_key = (tuple(layout_names), centered, direction)
                    else:
                        state_key = ("TRONG",)
                        offset_x  = 0.0
                        centered  = False
                        direction = "NONE"

                    state_changed = (state_key != last_printed_state)
                    periodic_send = (nearest is not None and frame_idx % PERIODIC_SEND_EVERY == 0)

                    if state_changed or periodic_send:
                        last_printed_state = state_key
                        if not dets:
                            send_uart({"status": "empty"})
                        else:
                            send_uart(build_detect_payload(
                                nearest, offset_x, centered, direction, len(dets)
                            ))

                # ----- Heartbeat khi rong -----
                now_ms = time.ticks_ms()
                if not dets and time.ticks_diff(now_ms, last_heartbeat) > int(HEARTBEAT_SEC * 1000):
                    send_uart({"status": "alive", "uptime_frame": frame_idx})
                    last_heartbeat = now_ms
                elif dets:
                    last_heartbeat = now_ms

                # ==========================================
                # VE LEN MAN HINH
                # ==========================================
                left_bound  = int(CAM_CENTER_X - CENTER_TOLERANCE)
                right_bound = int(CAM_CENTER_X + CENTER_TOLERANCE)
                cam_cx_int  = int(CAM_CENTER_X)
                proc_h_int  = int(PROC_H)

                pl.osd_img.draw_line(left_bound,  0, left_bound,  proc_h_int - 1, color=(0, 255, 0), thickness=1)
                pl.osd_img.draw_line(right_bound, 0, right_bound, proc_h_int - 1, color=(0, 255, 0), thickness=1)
                pl.osd_img.draw_line(cam_cx_int,  0, cam_cx_int,  proc_h_int - 1, color=(255, 255, 0), thickness=2)

                for d in dets:
                    off, cent, _ = centering_status(d["cx"])
                    dot_color = (0, 255, 0) if cent else (255, 80, 0)
                    pl.osd_img.draw_circle(int(d["cx"]), int(d["cy"]), 5, color=dot_color, thickness=-1)

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
                        4, proc_h_int - 30, 24,
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
