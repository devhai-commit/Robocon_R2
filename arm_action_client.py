#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from ak60_bringup.action import MoveArm

import time
import sys

class PickPlaceClient(Node):

    def __init__(self):
        super().__init__('pick_place_client_node')

        self.client = ActionClient(self, MoveArm, 'move_arm')

        # Dữ liệu vị trí: [Arm1 (mm), Arm2 (Độ), Arm3 (Độ), Gripper (Độ)]
        # Action Server sẽ tự động lo việc chuyển đổi sang hệ Mét và Radian.
        self.presets = {
            "initial": [230.0, 0.0, 0.0, 40.0],
            "home": [230.0, 0.0, 0.0, 40.0],
            "pos1": [230.0, 80.0, 00.0, 75.0], # tiep can muc tieu
            "pos2": [230.0, 80.0, -10.0, 75.0],   # dua tay kep vao vat
            "pos3": [230.0, 80.0, 0.0, 10.0],    # kep vat
            "pos4": [230.0, 80.0, 90.0, 10.0],  # 
            "pos5": [230.0, 0.0, 90.0, 10.0],  # vao vi tri tha va
            "pos6": [230.0, 0.0, 90.0, 40.0],  # tha vat
        }

    # ==============================
    # FEEDBACK CALLBACK (Hiển thị Real-time)
    # ==============================
    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        # In đè lên dòng hiện tại để terminal không bị trôi
        sys.stdout.write(f"\r  -> Đang tới: Arm1: {fb.current_arm1:.1f}mm | Arm2: {fb.current_arm2:.1f}° | Arm3: {fb.current_arm3:.1f}° | Grip: {fb.current_gripper:.1f}°   ")
        sys.stdout.flush()

    # ==============================
    # SEND GOAL LOGIC
    # ==============================
    def move(self, name):
        target = self.presets.get(name)
        if not target:
            self.get_logger().error(f"Preset '{name}' không tồn tại!")
            return False

        goal_msg = MoveArm.Goal()
        goal_msg.arm1 = float(target[0])
        goal_msg.arm2 = float(target[1])
        goal_msg.arm3 = float(target[2])
        goal_msg.gripper = float(target[3])

        # Chờ Server phản hồi (timeout 5s an toàn)
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server không phản hồi! Vui lòng kiểm tra lại Server.")
            return False

        self.get_logger().info(f"Đang gửi lệnh tới [{name}]...")
        
        # Gửi goal và đăng ký hàm nhận feedback
        send_goal_future = self.client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"Lệnh tới [{name}] đã bị Server từ chối.")
            return False

        # Chờ cho đến khi Server báo cáo đã di chuyển xong
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        # Xuống dòng sau khi in xong chuỗi feedback
        print() 

        # Kiểm tra mã trạng thái trả về
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Đã tới [{name}] thành công!")
            return True
        else:
            self.get_logger().error(f"Di chuyển tới [{name}] thất bại với mã lỗi: {status}")
            return False

    # ==============================
    # CHUỖI PICK & PLACE
    # ==============================
    def run_cycle(self):
        self.get_logger().info("\n=== BẮT ĐẦU CHU KỲ GẮP THẢ ===")

        sequence = [
            "home",
            "pos1",  # tới vật (mở kẹp)
            "pos2",  # đóng kẹp
            "pos3",  # nâng lên
            "pos4",  # di chuyển
            "pos5",
            "pos6",  # thả vật
            "home"
        ]

        for step in sequence:
            success = self.move(step)

            if not success:
                self.get_logger().error("CÓ LỖI XẢY RA -> DỪNG CHU KỲ ĐỂ ĐẢM BẢO AN TOÀN")
                return

            # Dừng một nhịp ngắn (1s) giữa mỗi động tác để tay máy ổn định rung lắc
            time.sleep(1.0) 

        self.get_logger().info("=== HOÀN TẤT CHU KỲ ===\n")


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceClient()

    try:
        while rclpy.ok():
            node.run_cycle()
            node.get_logger().info("Nghỉ 3 giây trước khi lặp lại...")
            time.sleep(3.0)  # Thời gian chờ giữa các chu kỳ
            
    except KeyboardInterrupt:
        node.get_logger().info("\nNhận lệnh CTRL+C, đang tắt Client an toàn...")
    finally:
        node.client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

