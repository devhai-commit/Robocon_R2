#!/usr/bin/env python3
import time
import py_trees
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from ak60_bringup.action import WallAlignment, FollowTarget, MoveArm, ArmSequence


class WallAlignmentBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, window_degrees, goal_distance, timeout_sec= 5.0):
        super().__init__(name)
        self.ros_node = ros_node
        self.window_degrees = float(window_degrees)
        self.goal_distance  = float(goal_distance)
        self.timeout_sec    = float(timeout_sec)
        self.client = ActionClient(self.ros_node, WallAlignment, 'align_and_approach_wall')
        self.goal_handle = self.goal_future = self.result_future = None
        self.start_time = None

    def setup(self, **kwargs):
        # Đưa lệnh chờ server về đúng vị trí (chạy 1 lần lúc bật máy, có giới hạn thời gian)
        return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        # Giải phóng hàm initialise để BT Tick cực nhanh
        self.goal_handle = self.goal_future = self.result_future = None
        self.start_time = time.time()  # Bắt đầu bấm giờ cho cơ chế Timeout
        
        self.ros_node.get_logger().info(f"[{self.name}] Đã gửi Lệnh: Quét {self.window_degrees}°, dừng cách {self.goal_distance}m")
        goal_msg = WallAlignment.Goal()
        goal_msg.window_degrees = self.window_degrees
        goal_msg.goal_distance  = self.goal_distance
        
        # Gửi lệnh đi nhưng KHÔNG block luồng chờ callback
        self.goal_future = self.client.send_goal_async(goal_msg)

    def update(self):
        # 1. Cơ chế Timeout chống kẹt (Ép dừng nếu quá số giây quy định)
        if self.start_time:
            elapsed = time.time() - self.start_time
            if elapsed > self.timeout_sec:
                self.ros_node.get_logger().warn(f"[{self.name}] ⏳ TIMEOUT ({self.timeout_sec}s). Bỏ qua để đi tiếp.")
                if getattr(self, 'goal_handle', None):
                    self.goal_handle.cancel_goal_async() # Bắt buộc xe dừng lắc lư
                return py_trees.common.Status.FAILURE

        # 2. Logic Polling an toàn (Tương thích 100% với ROS 2 Executor)
        if self.goal_future is None or not self.goal_future.done():
            return py_trees.common.Status.RUNNING
            
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted:
                self.ros_node.get_logger().error(f"[{self.name}] Server TỪ CHỐI lệnh!")
                return py_trees.common.Status.FAILURE
                
        if self.result_future is None:
            self.result_future = self.goal_handle.get_result_async()
            
        if not self.result_future.done():
            return py_trees.common.Status.RUNNING
            
        return (py_trees.common.Status.SUCCESS
                if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED
                else py_trees.common.Status.FAILURE)

    def terminate(self, new_status):
        # Nếu bị ngắt (như bấm nút Stop), tự động hủy Goal an toàn
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None):
            self.goal_handle.cancel_goal_async()

class MoveArmBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, target_pose):
        super().__init__(name)
        self.ros_node = ros_node
        self.target = target_pose
        self.client = ActionClient(self.ros_node, MoveArm, 'move_arm')
        self.goal_handle = self.goal_future = self.result_future = None

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="current_arm1_z", access=py_trees.common.Access.READ)

    def setup(self, **kwargs): return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_handle = self.goal_future = self.result_future = None
        goal_msg = MoveArm.Goal()

        arm1_val = self.target[0]
        if arm1_val == "auto":
            arm1_val = self.blackboard.current_arm1_z if self.blackboard.exists("current_arm1_z") else 125.0

        goal_msg.arm1, goal_msg.arm2, goal_msg.arm3, goal_msg.gripper = float(arm1_val), float(self.target[1]), float(self.target[2]), float(self.target[3])
        self.goal_future = self.client.send_goal_async(goal_msg)

    def update(self):
        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
        if self.result_future is None: self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        return (py_trees.common.Status.SUCCESS
                if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED
                else py_trees.common.Status.FAILURE)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None):
            self.goal_handle.cancel_goal_async()


class ArmSequenceBTNode(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, ros_node: Node, pose_name: str, duration: float = 1.0):
        super().__init__(name)
        self.ros_node  = ros_node
        self.pose_name = pose_name
        self.duration  = float(duration)
        self.action_client = None
        self._goal_future = self._result_future = self._goal_handle = None

    def setup(self, **kwargs):
        self.action_client = ActionClient(self.ros_node, ArmSequence, 'arm_sequence_controller')
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.ros_node.get_logger().error(f"[{self.name}] Không tìm thấy 'arm_sequence_controller'!")
            return False
        return True

    def initialise(self):
        self.ros_node.get_logger().info(f"[{self.name}] Gửi lệnh ESP32: '{self.pose_name}'")
        goal_msg = ArmSequence.Goal()
        goal_msg.pose_name = self.pose_name
        goal_msg.duration  = self.duration
        self._goal_future = self.action_client.send_goal_async(goal_msg)
        self._result_future = self._goal_handle = None

    def update(self):
        if self._goal_future and not self._goal_future.done(): return py_trees.common.Status.RUNNING
        if self._goal_future and self._goal_handle is None:
            self._goal_handle = self._goal_future.result()
            if not self._goal_handle.accepted:
                self.ros_node.get_logger().error(f"[{self.name}] Server từ chối '{self.pose_name}'!")
                return py_trees.common.Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
        if self._result_future and not self._result_future.done(): return py_trees.common.Status.RUNNING
        if self._result_future and self._result_future.done():
            if self._result_future.result().result.success:
                self.ros_node.get_logger().info(f"[{self.name}] Hoàn thành '{self.pose_name}'!")
                return py_trees.common.Status.SUCCESS
            self.ros_node.get_logger().error(f"[{self.name}] Thất bại '{self.pose_name}'!")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if (new_status == py_trees.common.Status.INVALID
                and self._goal_handle
                and not getattr(self._result_future, 'done', lambda: True)()):
            self.ros_node.get_logger().warn(f"[{self.name}] Hủy lệnh '{self.pose_name}'...")
            self._goal_handle.cancel_goal_async()


class RosWaitBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, delay_sec):
        super().__init__(name)
        self.ros_node   = ros_node
        self.delay_sec  = delay_sec
        self.start_time = None

    def initialise(self):
        self.start_time = self.ros_node.get_clock().now()
        self.ros_node.get_logger().info(f"[{self.name}] Đang chờ {self.delay_sec}s...")

    def update(self):
        elapsed = (self.ros_node.get_clock().now() - self.start_time).nanoseconds / 1e9
        return (py_trees.common.Status.SUCCESS
                if elapsed >= self.delay_sec
                else py_trees.common.Status.RUNNING)

class FollowTargetBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, target_id, desired_distance_mm, timeout_sec=15.0):
        super().__init__(name)
        self.ros_node = ros_node
        self.target_id = target_id
        self.desired_distance_mm = float(desired_distance_mm)
        self.timeout_sec = float(timeout_sec)  # Bổ sung Timeout bảo vệ
        self.client = ActionClient(self.ros_node, FollowTarget, 'follow_target')
        
        # GIỮ NGUYÊN CƠ CHẾ ĐĂNG KÝ BLACKBOARD CHUẨN CỦA BẠN
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="latest_box_detection", access=py_trees.common.Access.WRITE)
        
        self.goal_handle = self.goal_future = self.result_future = None
        self.start_time = None
        self._real_detected = False

    def setup(self, **kwargs): 
        return self.client.wait_for_server(timeout_sec=5.0)
    
    def _feedback_cb(self, feedback_msg):
        # Feedback co field tracking_state ("real" / "fake" / "empty") tu tracking_server.
        state = getattr(feedback_msg.feedback, 'tracking_state', '').upper()
        if state == "REAL" and not self._real_detected:
            self._real_detected = True
            self.blackboard.latest_box_detection = "REAL"
            self.ros_node.get_logger().info(
                f"[{self.name}] ✅ Feedback REAL → lock blackboard latest_box_detection='REAL'"
            )

    def initialise(self):
        self.goal_handle = self.goal_future = self.result_future = None
        self.start_time = self.ros_node.get_clock().now()  # Bấm giờ
        self._real_detected = False
        
        self.ros_node.get_logger().info(f"[{self.name}] 👁️ Tìm mục tiêu ID={self.target_id}...")
        goal_msg = FollowTarget.Goal()
        goal_msg.target_id = self.target_id
        goal_msg.desired_distance_mm = float(self.desired_distance_mm)
        self.goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)

    def update(self):
        # 1. CƠ CHẾ TIMEOUT CHỐNG KẸT
        if self.start_time:
            elapsed = (self.ros_node.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > self.timeout_sec:
                self.ros_node.get_logger().error(f"[{self.name}] ⏳ TIMEOUT ({self.timeout_sec}s)! Camera/AI không phản hồi.")
                if getattr(self, 'goal_handle', None):
                    self.goal_handle.cancel_goal_async()
                return py_trees.common.Status.FAILURE

        # 2. LOGIC POLLING KHÔNG ĐÓNG BĂNG HỆ THỐNG
        if self.goal_future is None or not self.goal_future.done(): 
            return py_trees.common.Status.RUNNING
            
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: 
                self.ros_node.get_logger().error(f"[{self.name}] AI Server TỪ CHỐI lệnh!")
                return py_trees.common.Status.FAILURE
                
        if self.result_future is None: 
            self.result_future = self.goal_handle.get_result_async()
            
        if not self.result_future.done(): 
            return py_trees.common.Status.RUNNING
        
        # 3. XỬ LÝ KẾT QUẢ VÀ GHI BLACKBOARD
        result = self.result_future.result()
        tracking_status = result.result.message.upper()
        
        if not self._real_detected or tracking_status == "REAL":
            self.blackboard.latest_box_detection = tracking_status
            self.ros_node.get_logger().info(f"[{self.name}] 🧠 AI trả về: {tracking_status}")
        else:
            self.ros_node.get_logger().info(
                f"[{self.name}] 🧠 AI trả về: {tracking_status} (giu blackboard='REAL' tu feedback)"
            )
        
        return (py_trees.common.Status.SUCCESS
                if result.status == GoalStatus.STATUS_SUCCEEDED and tracking_status == "REAL"
                else py_trees.common.Status.FAILURE)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None):
            self.ros_node.get_logger().warn(f"[{self.name}] 🛑 Bị ngắt, tự động hủy lệnh AI.")
            self.goal_handle.cancel_goal_async()
