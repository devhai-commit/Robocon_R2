# -*- coding: utf-8 -*-
# ==========================================
# JETSON SIDE - Doc du lieu tu K230 (19.best_kmodel_uart.py)
#
# K230 gui JSON moi dong, vi du:
#   {"status":"detect","target":"TARGET","cx":322.0,"cy":180.5,
#    "offset_x":2.0,"direction":"GIUA","centered":true,
#    "area":12345,"score":0.87,"n_obj":1}
#   {"status":"empty"}
#   {"status":"alive","uptime_frame":120}
#   {"system":"booting"|"warming_up"|"ready"|"stopped_manually"}
#   {"error":"..."}
#
# offset_x > 0 -> object lech sang PHAI khung hinh -> robot can quay PHAI
# offset_x < 0 -> object lech sang TRAI khung hinh -> robot can quay TRAI
# centered    -> object trong vung tam ( |offset_x| <= tolerance )
# ==========================================
import serial
import json
import time
import threading

PORT     = "/dev/ttyYaboom"
BAUDRATE = 115200

# Trang thai moi nhat tu K230 (dung cho vong dieu khien robot)
latest = {
    "ready"      : False,
    "target"     : None,
    "cx"         : None,
    "cy"         : None,
    "offset_x"   : None,
    "direction"  : None,   # "GIUA" / "TRAI" / "PHAI" / "NONE"
    "centered"   : False,
    "area"       : None,
    "score"      : None,
    "n_obj"      : 0,
    "last_seen"  : 0.0,
    "alive"      : False,
    "last_msg_t" : 0.0,
}

WATCHDOG_SEC = 5.0


def handle_message(msg):
    """Xu ly 1 dict JSON da parse tu K230. Cap nhat 'latest' va in log."""
    now = time.time()
    latest["last_msg_t"] = now
    latest["alive"]      = True

    if "system" in msg:
        sys_state = msg["system"]
        print(f"[K230 SYSTEM] {sys_state}")
        if sys_state == "ready":
            latest["ready"] = True
            if "proc_space" in msg:
                print(f"  proc_space = {msg['proc_space']}, "
                      f"cam_center = {msg['cam_center']}, "
                      f"tolerance = {msg['tolerance']}")
        return

    if "error" in msg:
        print(f"[K230 ERROR] {msg['error']}")
        return

    status = msg.get("status")

    if status == "detect":
        latest["target"]    = msg.get("target")
        latest["cx"]        = msg.get("cx")
        latest["cy"]        = msg.get("cy")
        latest["offset_x"]  = msg.get("offset_x")
        latest["direction"] = msg.get("direction")
        latest["centered"]  = msg.get("centered", False)
        latest["area"]      = msg.get("area")
        latest["score"]     = msg.get("score")
        latest["n_obj"]     = msg.get("n_obj", 0)
        latest["last_seen"] = now

        flag = "[GIUA]" if latest["centered"] else f"[{latest['direction']}]"
        print(f"DETECT {flag} target={latest['target']:>8s}  "
              f"cx={latest['cx']:>6.1f}  off={latest['offset_x']:>+6.1f}  "
              f"area={latest['area']:>6d}  score={latest['score']:.2f}  "
              f"n={latest['n_obj']}")

    elif status == "empty":
        latest["target"]    = None
        latest["direction"] = "NONE"
        latest["centered"]  = False
        latest["n_obj"]     = 0
        print("EMPTY  (khong co object trong khung hinh)")

    elif status == "alive":
        print(f"HEARTBEAT (uptime_frame={msg.get('uptime_frame')})")

    else:
        print(f"[K230 UNKNOWN] {msg}")


def check_watchdog():
    """Phat hien K230 mat ket noi."""
    if latest["alive"] and (time.time() - latest["last_msg_t"]) > WATCHDOG_SEC:
        latest["alive"] = False
        print(f"[WATCHDOG] K230 khong gui message qua {WATCHDOG_SEC}s -> coi nhu mat ket noi")


def main():
    print(f"[JETSON] Mo cong {PORT} @ {BAUDRATE} bps...")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    except serial.SerialException as e:
        print(f"[LOI] Khong mo duoc cong serial: {e}")
        return

    print("[JETSON] San sang doc du lieu tu K230. Ctrl+C de thoat.\n")

    buf = b""
    try:
        while True:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)

                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    if not (line.startswith(b"{") and line.endswith(b"}")):
                        continue
                    try:
                        msg = json.loads(line.decode("utf-8", errors="ignore"))
                    except (json.JSONDecodeError, UnicodeDecodeError) as e:
                        print(f"[PARSE FAIL] {line!r}  ({e})")
                        continue
                    handle_message(msg)
            else:
                check_watchdog()
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[JETSON] Dung boi nguoi dung.")
    finally:
        ser.close()
        print("[JETSON] Da dong cong serial.")


# ==========================================
# VI DU DUNG 'latest' DE DIEU KHIEN ROBOT
# ==========================================
# if latest["ready"] and latest["alive"]:
#     if latest["target"] is None:
#         scan()                              # khong co object -> quet
#     elif latest["centered"]:
#         stop()                              # o giua -> dung lai (hoac kep gap)
#     elif latest["direction"] == "TRAI":
#         turn_left(abs(latest["offset_x"]))  # lech trai -> quay trai
#     elif latest["direction"] == "PHAI":
#         turn_right(latest["offset_x"])      # lech phai -> quay phai
# ==========================================

def start_reader_thread():
    """Khởi động vòng đọc UART K230 trong daemon thread.
    Gọi 1 lần trước khi dùng biến `latest` từ module khác.
    """
    t = threading.Thread(target=main, daemon=True, name="k230_uart_reader")
    t.start()
    return t


if __name__ == "__main__":
    main()
