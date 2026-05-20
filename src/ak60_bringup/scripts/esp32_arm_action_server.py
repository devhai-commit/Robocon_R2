#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
import serial
import struct
import threading
import time
import sys

# --- IMPORT ACTION TỪ WORKSPACE CỦA BẠN ---
from ak60_bringup.action import ArmSequence 

# ================= CONFIG =================
PORT = '/dev/ttyEsp32' 
BAUD_RATE = 115200
NUM_JOINTS = 5
HEADER1 = 0xAA
HEADER2 = 0x55
KEEP_ALIVE_TIMEOUT = 4.0

PREDEFINED_POSES = {
    "home_pose_right":    ( [0.0, 0.0, 0.0, 45.0, 90.0],    [0.0,  0.0,  -1.0, 0.0, 0.0] ),
    "approach_j4_right":  ( [0.0, 0.0, 0.0, 105.0, 90.0],    [0.0,  0.0, 0.0, 0.0, 0.0] ), 
    "approach_j3_right":  ( [0.0, 0.0, 0.0, 105.0, 90.0],    [0.0,  0.0, 1.0, 0.0, 0.0] ), 
    "grasp_right":        ( [0.0, 0.0, 0.0, 105.0, 0.0],    [0.0,  0.0,  0.0, 0.0, 0.0] ),
    "move_back_J4_right": ( [0.0, 0.0, 0.0, 45.0, 0.0],  [0.0,  0.0,  0.0, 0.0, 0.0] ),
    "move_back_J1_right": ( [180.0, 0.0, 0.0, 45.0, 0.0],    [0.0,  0.0,  0.0, 0.0, 0.0] ),
    "release_right":      ( [180.0, 0.0, 0.0, 45.0, 90.0],    [0.0,  0.0,  0.0, 0.0, 0.0] ),

    "home_pose_left":    ( [180.0, 0.0, 0.0, 45.0, 90.0],    [0.0,  0.0,  -1.0, 0.0, 0.0] ),
    "approach_j4_left":  ( [180.0, 0.0, 0.0, 105.0,  90.0],    [0.0,  0.0, 0.0, 0.0, 0.0] ), 
    "approach_j3_left":  ( [180.0, 0.0, 0.0, 105.0,  90.0],    [0.0,  0.0, 1.0, 0.0, 0.0] ), 
    "grasp_left":        ( [180.0, 0.0, 0.0, 105.0,  0.0],    [0.0,  0.0,  0.0, 0.0, 0.0] ),
    "move_back_J4_left": ( [180.0, 0.0, 0.0, 45.0, 0.0],  [0.0,  0.0,  0.0, 0.0, 0.0] ),
    "move_back_J1_left": ( [0.0, 0.0, 0.0, 45.0,   0.0],    [0.0,  0.0,  0.0, 0.0, 0.0] ),
    "release_left":      ( [0.0, 0.0, 0.0, 45.0,   90.0],    [0.0,  0.0,  0.0, 0.0, 0.0] ),

    # Alias phía phải — tên dùng trong BT
    "approach_j4":  ( [0.0, 0.0, 0.0,  40.0, 30.0],   [0.0,  0.0,  0.0, 0.0, 0.0] ),
    "approach_j3":  ( [0.0, 0.0, 0.0,  40.0, 30.0],   [0.0,  0.0, -1.0, 0.0, 0.0] ),
    "grasp":        ( [0.0, 0.0, 0.0,  40.0, 90.0],   [0.0,  0.0,  0.0, 0.0, 0.0] ),
    "move_back_J4": ( [0.0, 0.0, 0.0, 100.0, 90.0],   [0.0,  0.0,  0.0, 0.0, 0.0] ),
    "approach_j1":  ( [180.0, 0.0, 0.0, 100.0, 90.0], [0.0,  0.0,  0.0, 0.0, 0.0] ),
}

class Esp32ArmServer(Node):
    def __init__(self):
        super().__init__('esp32_arm_action_server')
        self.state_pos = [0.0] * NUM_JOINTS
        self.target_pos = [0.0] * NUM_JOINTS
        self.target_eff = [0.0] * NUM_JOINTS 
        
        self.ser = None
        self.active_control = False 
        self.goal_running = False   
        self.state_received = False 
        self.last_activity_time = time.time()
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.data_lock = threading.Lock()

        # ================= SỬA LỖI: CHỐNG AUTO-RESET ESP32 =================
        try:
            self.ser = serial.Serial()
            self.ser.port = PORT
            self.ser.baudrate = BAUD_RATE
            self.ser.timeout = 0.1
            self.ser.setDTR(False) # <--- QUAN TRỌNG: Ngăn ESP32 bị reset/đơ
            self.ser.setRTS(False) # <--- QUAN TRỌNG: Ngăn ESP32 bị reset/đơ
            self.ser.open()
            
            threading.Thread(target=self.rx_task, daemon=True).start()
            self.get_logger().info(f"Đã mở kết nối UART tại {PORT} (Đã bật chống Reset).")
        except Exception as e:
            self.get_logger().error(f"Lỗi mở cổng: {e}. Vui lòng cắm cáp và chạy lại!")
            sys.exit(1)

        self.tx_timer = self.create_timer(0.05, self.tx_task) 
        self.keep_alive_timer = self.create_timer(1.0, self.check_keep_alive) 
        self._action_server = ActionServer(self, ArmSequence, 'arm_sequence_controller', self.execute_callback)
        self.get_logger().info("Action Server đã sẵn sàng! Chờ Client...")
        

    def tx_task(self):
        if self.active_control and self.ser and self.ser.is_open:
            self.send_uart_frame()

    def rx_task(self):
        state = 0; expected_len = 0; cmd_type = 0; payload = bytearray(); checksum = 0
        while rclpy.ok() and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    b = self.ser.read(1)[0]
                    if state == 0:
                        if b == HEADER1: state = 1
                    elif state == 1:
                        if b == HEADER2: state = 2
                        else: state = 0
                    elif state == 2:
                        expected_len = b; checksum = b; state = 3
                    elif state == 3:
                        cmd_type = b; checksum ^= b; payload = bytearray(); state = 4
                    elif state == 4:
                        payload.append(b); checksum ^= b
                        if len(payload) >= (expected_len - 1): state = 5
                    elif state == 5:
                        if b == checksum and cmd_type == 0x02 and len(payload) == 40:
                            try:
                                data = struct.unpack('<10f', payload)
                                with self.data_lock:
                                    # Lấy nguyên bản [ĐỘ] từ ESP32, KHÔNG ĐỔI RADIAN NỮA
                                    self.state_pos = [data[i*2] for i in range(NUM_JOINTS)]
                                    self.state_received = True 

                                    js_msg = JointState()
                                    js_msg.header.stamp = self.get_clock().now().to_msg()
                                    js_msg.name = ['j1_can', 'j2_dc', 'j3_dc', 'j4_arm', 'j5_gripper']
                                    js_msg.position = self.state_pos # Đã là ĐỘ
                                    self.state_pub.publish(js_msg)
                                    
                            except Exception as parse_err:
                                self.get_logger().error(f"Lỗi đọc data struct: {parse_err}")
                        state = 0
                else:
                    time.sleep(0.01) # 100 Hz
            except serial.SerialException:
                self.get_logger().error("Cáp UART bị rút đột ngột!")
                break

    def send_uart_frame(self):
        with self.data_lock:
            # Gửi nguyên bản [ĐỘ] xuống ESP32, KHÔNG ĐỔI RADIAN NỮA
            combined_data = self.target_pos + self.target_eff
            data_bytes = struct.pack('<10f', *combined_data)
            
        cmd_type = 0x01
        length = len(data_bytes) + 1
        payload = bytes([length, cmd_type]) + data_bytes
        cs = 0
        for byte in payload: cs ^= byte
        frame = bytes([HEADER1, HEADER2]) + payload + bytes([cs])
        try:
            self.ser.write(frame)
        except serial.SerialException:
            pass

    def check_keep_alive(self):
        if self.active_control and not self.goal_running:
            if (time.time() - self.last_activity_time) > KEEP_ALIVE_TIMEOUT:
                self.enter_sleep_mode()

    def enter_sleep_mode(self):
        self.active_control = False 
        with self.data_lock:
            self.target_eff = [0.0] * NUM_JOINTS 
        self.send_uart_frame() 
        self.get_logger().info("<== Server chuyển về trạng thái NGỦ ĐÔNG (Khóa cơ khí).")

    def get_state(self):
        with self.data_lock:
            return list(self.state_pos)

    def execute_callback(self, goal_handle):
        if self.goal_running:
            goal_handle.abort(); return ArmSequence.Result(success=False)

        self.goal_running = True
        goal = goal_handle.request
        
        if not self.active_control:
            self.get_logger().info(f"==> Đánh thức Robot! Yêu cầu: '{goal.pose_name}'")
            self.active_control = True
        else:
            self.get_logger().info(f"==> Kế thừa kết nối! Yêu cầu: '{goal.pose_name}'")

        # Tăng thời gian chờ đồng bộ lên 3.0s cho an toàn
        wait_time = 0.0
        while not self.state_received and wait_time < 3.0:
            time.sleep(0.05); wait_time += 0.05
            
        if not self.state_received:
            self.get_logger().error("Lỗi: Không đọc được trạng thái từ mạch. Kiểm tra dây tín hiệu.")
            self.enter_sleep_mode(); self.goal_running = False
            goal_handle.abort(); return ArmSequence.Result(success=False)

        with self.data_lock:
            self.target_pos = list(self.state_pos)
            self.target_eff = [0.0] * NUM_JOINTS
        result = ArmSequence.Result()

        # ================= KỊCH BẢN 1: LẮP RÁP =================
        if goal.pose_name == "assemble_sequence":
            timeout_max = 60.0
            
            # BƯỚC 1: Đẩy J3 xuống (Phải cấp công suất > 0 để cơ cấu chạy)
            # with self.data_lock: self.target_eff[2] = -0.3
            # start_wait = time.time()
            # while rclpy.ok():
            #     if self.get_state()[2] < -0.5: break 
            #     time.sleep(0.05)
            #     if goal_handle.is_cancel_requested or (time.time() - start_wait) > timeout_max: 
            #         with self.data_lock: self.target_eff[2] = 0.0
            #         self.goal_running = False; self.last_activity_time = time.time()
            #         goal_handle.abort(); return ArmSequence.Result(success=False)

            # BƯỚC 2: Chay max power J3, chờ cơ cấu lò xo đẩy về điểm âm
            # with self.data_lock: self.target_eff[2] = 1.0
            start_wait = time.time()
            while rclpy.ok():
                if self.get_state()[2] < 0.5: 
                    break 
                time.sleep(0.05)
                if goal_handle.is_cancel_requested or (time.time() - start_wait) > timeout_max: 
                    self.goal_running = False; self.last_activity_time = time.time()
                    goal_handle.abort(); return ArmSequence.Result(success=False)

            # BƯỚC 3: Mở kẹp J5 (Dùng vòng lặp thay vì time.sleep để Node không bị đơ)
            wait_timer = time.time()
            while rclpy.ok() and (time.time() - wait_timer) < 5.0:
                time.sleep(0.05)
                
            with self.data_lock: self.target_pos[4] = 90.0 
            
            wait_timer = time.time()
            while rclpy.ok() and (time.time() - wait_timer) < 10.0:
                time.sleep(0.05)

            self.goal_running = False; 
            self.last_activity_time = time.time()
            goal_handle.succeed(); 
            result.success = True
            return result

        # ================= KỊCH BẢN 2: HOMING =================
        elif goal.pose_name == "home":
            with self.data_lock:
                self.target_pos[0] = 0.0; self.target_pos[3] = 100.0; self.target_pos[4] = 30.0
                # SỬA LỖI: J2 và J3 đều phải được kéo về (âm 0.3)
                self.target_eff[1] = -0.3; self.target_eff[2] = -0.3 
            timeout = 10.0; start_time = time.time(); success = False
            while rclpy.ok() and self.active_control:
                curr = self.get_state()
                with self.data_lock:
                    if curr[1] < -0.5: self.target_eff[1] = 0.0
                    if curr[2] < -0.5: self.target_eff[2] = 0.0
                if (curr[1] < -0.5) and (curr[2] < -0.5):
                    success = True; break
                if (time.time() - start_time) > timeout or goal_handle.is_cancel_requested: break
                time.sleep(0.05) 
            with self.data_lock: self.target_eff = [0.0] * NUM_JOINTS 
            self.goal_running = False; self.last_activity_time = time.time()
            if success: goal_handle.succeed(); result.success = True
            else: goal_handle.abort(); result.success = False
            return result
        
        # ================= KỊCH BẢN 3: WAYPOINTS =================
        else:
            if goal.pose_name in PREDEFINED_POSES:
                goal_pos, goal_eff = PREDEFINED_POSES[goal.pose_name]
            elif goal.pose_name == "custom":
                goal_pos = list(goal.target_pos); goal_eff = list(goal.target_eff)
            else:
                self.goal_running = False; self.last_activity_time = time.time()
                goal_handle.abort(); return ArmSequence.Result(success=False)

            with self.data_lock:
                start_pos = list(self.target_pos); start_eff = list(self.target_eff)
            
            duration = goal.duration if goal.duration > 0 else 2.5
            steps = int(duration * 20.0)
            feedback_msg = ArmSequence.Feedback()

            for step in range(1, steps + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    with self.data_lock: self.target_eff = [0.0] * NUM_JOINTS 
                    self.goal_running = False; self.last_activity_time = time.time()
                    return ArmSequence.Result(success=False)

                alpha = step / float(steps)
                with self.data_lock:
                    for i in range(NUM_JOINTS):
                        self.target_pos[i] = start_pos[i] + (goal_pos[i] - start_pos[i]) * alpha
                        self.target_eff[i] = start_eff[i] + (goal_eff[i] - start_eff[i]) * alpha
                
                feedback_msg.current_pos = self.get_state()
                feedback_msg.progress_percent = alpha * 100.0
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1.0 / 20.0)
                
            timeout_wait = 5.0 
            start_wait = time.time()
            reached = False

            while rclpy.ok():
                curr = self.get_state()
                # Kiểm tra sai số J1, J4, J5
                err0 = abs(curr[0]-goal_pos[0])
                err3 = abs(curr[3]-goal_pos[3])
                err4 = abs(curr[4]-goal_pos[4])
                
                if err0 < 5.0 and err3 < 5.0 and err4 < 5.0: # Nới J1 lên 2.5 cho dễ đạt
                    reached = True
                    break
                
                if (time.time() - start_wait) > timeout_wait or goal_handle.is_cancel_requested:
                    break
                time.sleep(0.05)

            self.goal_running = False
            self.last_activity_time = time.time()

            if reached:
                self.get_logger().info('🏁 Đã tới đích! Gửi kết quả: THÀNH CÔNG') # Thêm log này
                goal_handle.succeed()
                result.success = True
            else:
                self.get_logger().warn('⚠️ Hết thời gian chờ hoặc bị hủy. Gửi kết quả: THẤT BẠI')
                goal_handle.abort()
                result.success = False
            
            return result
        
def destroy_node(self):
        self.get_logger().info("Đang tắt Server, đưa tay máy về Sleep...")
        # Đưa robot về an toàn và nhả cổng COM khi bấm Ctrl+C
        self.enter_sleep_mode()
        time.sleep(0.1)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    action_server = Esp32ArmServer()
    try:
        from rclpy.executors import MultiThreadedExecutor
        rclpy.spin(action_server, executor=MultiThreadedExecutor())
    except KeyboardInterrupt: pass
    finally:
        action_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

