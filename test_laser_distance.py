import serial
import time

SERIAL_PORT = '/dev/ttyLaser'  # Chỉnh lại theo cổng thực tế của bạn
BAUD_RATE = 38400             # Thử 9600 nếu 38400 không có phản hồi

def main():
    try:
        # Khởi tạo cổng Serial
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.5)
        print(f"Đã mở cổng {SERIAL_PORT} ở baudrate {BAUD_RATE}")
        
        # Đảm bảo cảm biến đang ở trạng thái dừng trước khi gửi lệnh mới
        ser.write(b'iHAL\r\n')
        time.sleep(0.2)
        ser.flushInput()

        # Gửi lệnh đo liên tục tốc độ cao
        print("Gửi lệnh bắt đầu đo nhanh (iFACM)...")
        ser.write(b'iFACM\r\n')

        while True:
            if ser.in_waiting > 0:
                # Đọc từng dòng dữ liệu cảm biến trả về
                response = ser.readline().decode('ascii', errors='ignore').strip()
                
                if response:
                    # Dữ liệu trả về thường có định dạng là một con số (đơn vị mét hoặc milimet)
                    # Ví dụ: "1.234" hoặc "1234mm"
                    print(f"Giá trị đo được: {response}")

    except serial.SerialException as e:
        print(f"Lỗi kết nối Serial: {e}")
    except KeyboardInterrupt:
        # Dừng an toàn khi bấm Ctrl+C
        print("\nĐang gửi lệnh dừng (iHAL)...")
        if 'ser' in locals() and ser.is_open:
            ser.write(b'iHAL\r\n')
            time.sleep(0.1)
            ser.close()
            print("Đã đóng kết nối.")

if __name__ == "__main__":
    main()
