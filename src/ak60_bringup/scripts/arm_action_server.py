#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray
from ak60_bringup.action import MoveArm

import time
import threading
import math

class RobotArmActionServer(Node):

    def __init__(self):
        super().__init__('robot_arm_action_server')

        self._action_server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            execute_callback=self.execute_callback
        )

        self.joint_state_sub = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.joint_states_callback,
            10
        )

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )

        self.state_lock = threading.Lock()
        
        # current_position: Vị trí đọc từ phần cứng
        self.current_position = [None, None, None, None]
        # current_setpoint: Mục tiêu nội suy (Tránh lỗi giật tay máy)
        self.current_setpoint = [0.0, 0.0, 0.0, 0.0] 
        
        self.is_initialized = False
        self.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'gripper_joint']

        self.get_logger().info("Action Server sẵn sàng. Đang chờ đồng bộ vị trí...")

    def joint_states_callback(self, msg):
        with self.state_lock:
            for i, name in enumerate(msg.joint_names):
                if name in self.joint_names:
                    idx = self.joint_names.index(name)
                    
                    interface_value_msg = msg.interface_values[i]
                    if 'position' in interface_value_msg.interface_names:
                        pos_idx = interface_value_msg.interface_names.index('position')
                        sys_val = interface_value_msg.values[pos_idx]

                        # CHUYỂN ĐỔI VỀ MM VÀ ĐỘ
                        if idx == 0:
                            converted_val = sys_val * 1000.0  # Mét -> mm
                        else:
                            converted_val = math.degrees(sys_val) # Rad -> Độ

                        self.current_position[idx] = converted_val

            # CHỈ ĐỒNG BỘ SETPOINT 1 LẦN DUY NHẤT LÚC KHỞI ĐỘNG (Giống hệt Teleop)
            if not self.is_initialized and all(pos is not None for pos in self.current_position):
                self.current_setpoint = list(self.current_position)
                self.is_initialized = True
                self.get_logger().info("Đã đồng bộ Setpoint ban đầu! Có thể nhận lệnh.")

    def clamp(self, val, min_val, max_val):
        return max(min(float(val), float(max_val)), float(min_val))

    def execute_callback(self, goal_handle):
        self.get_logger().info("Đang thực hiện lệnh di chuyển...")

        with self.state_lock:
            if not self.is_initialized:
                self.get_logger().error("Lỗi: Chưa đồng bộ được vị trí ban đầu!")
                goal_handle.abort()
                return MoveArm.Result(success=False, message="No joint data")
            
            # Lấy điểm xuất phát từ Setpoint của lệnh trước đó (Tránh giật cục)
            setpoint = list(self.current_setpoint)

        goal = goal_handle.request
        
        target = [
            self.clamp(goal.arm1, 0.0, 760.0),
            self.clamp(goal.arm2, -10.0, 90.0),
            self.clamp(goal.arm3, -90.0, 90.0),
            self.clamp(goal.gripper, 0.0, 90.0)
        ]

        feedback_msg = MoveArm.Feedback()
        
        # Cấu hình tốc độ riêng biệt để các khớp chạy mượt và kịp nhau
        # Arm 1: 5.0 mm/vòng lặp (~100 mm/giây)
        # Các Arm quay: 2.0 độ/vòng lặp (~40 độ/giây)
        steps = [10.0, 3.0, 3.0, 20.0] 
        
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("Hành động bị hủy bởi Client.")
                return MoveArm.Result(success=False, message="Canceled")

            done = True
            for i in range(4):
                diff = target[i] - setpoint[i]
                if abs(diff) > steps[i]:
                    setpoint[i] += steps[i] if diff > 0 else -steps[i]
                    done = False
                else:
                    setpoint[i] = target[i]

            # LƯU LẠI SETPOINT TOÀN CỤC SAU MỖI BƯỚC NỘI SUY
            with self.state_lock:
                self.current_setpoint = list(setpoint)

            # ========================================================
            # CHUYỂN SANG MÉT/RADIAN ĐỂ PUBLISH
            # ========================================================
            sys_setpoint = [0.0, 0.0, 0.0, 0.0]
            sys_setpoint[0] = setpoint[0] / 1000.0  # mm -> Mét
            sys_setpoint[1] = math.radians(setpoint[1])
            sys_setpoint[2] = math.radians(setpoint[2])
            sys_setpoint[3] = math.radians(setpoint[3])

            cmd_msg = Float64MultiArray()
            cmd_msg.data = sys_setpoint
            self.command_pub.publish(cmd_msg)

            # Phản hồi về Client
            with self.state_lock:
                current_actual_pos = list(self.current_position)

            feedback_msg.current_arm1 = current_actual_pos[0]
            feedback_msg.current_arm2 = current_actual_pos[1]
            feedback_msg.current_arm3 = current_actual_pos[2]
            feedback_msg.current_gripper = current_actual_pos[3]
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.05)

            if done:
                break

        goal_handle.succeed()
        self.get_logger().info("Di chuyển hoàn tất!")
        
        return MoveArm.Result(success=True, message="Success")


def main(args=None):
    rclpy.init(args=args)
    node = RobotArmActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Đang tắt Server...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

