import subprocess
import time
import os

# Cấu hình: ID của Astra Pro Plus (060f hoặc 050f)
VENDOR_ID = "2bc5"
PRODUCT_IDS = ["060f", "050f"]

def get_usb_path():
    """Tìm đường dẫn bus/device của camera"""
    try:
        result = subprocess.check_output("lsusb", shell=True).decode()
        for line in result.split('\n'):
            if any(pid in line for pid in PRODUCT_IDS):
                parts = line.split()
                bus = parts[1]
                dev = parts[3].replace(':', '')
                return f"/dev/bus/usb/{bus}/{dev}"
    except:
        return None
    return None

def reset_usb():
    path = get_usb_path()
    if path:
        print(f"[INFO] Đang thực hiện reset cứng tại: {path}")
        # Giải phóng các node đang chiếm giữ camera trước khi reset
        subprocess.run("sudo fuser -k /dev/video*", shell=True)
        # Lệnh reset cổng USB
        subprocess.run(f"sudo usbreset {path}", shell=True)
        time.sleep(2)
        print("[SUCCESS] Đã reset xong!")
    else:
        print("[ERROR] Không tìm thấy camera để reset.")

def monitor_dmesg():
    print("[RUNNING] Đang theo dõi lỗi -71 trên hệ thống...")
    # Lệnh theo dõi log dmesg theo thời gian thực
    cmd = "sudo dmesg -w"
    process = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    while True:
        line = process.stdout.readline().decode('utf-8')
        if not line:
            break
        
        # Phát hiện mã lỗi -71 (Protocol error)
        if "error -71" in line or "No streaming interface found" in line:
            print(f"\n[ALERT] Phát hiện lỗi camera: {line.strip()}")
            reset_usb()
            # Sau khi reset, bạn có thể thêm lệnh khởi chạy lại node ROS2 ở đây
            # os.system("ros2 launch orbbec_camera astra.launch.py &")

if __name__ == "__main__":
    monitor_dmesg()