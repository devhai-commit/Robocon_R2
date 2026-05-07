#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# --- Standard ROS 2 Action / Msgs ---
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

# --- Custom Actions ---
from ak60_bringup.action import ClimbStep
from ak60_bringup.action import FollowTarget
from ak60_bringup.action import MoveArm
from ak60_bringup.action import WallAlignment
from ak60_bringup.action import ArmSequence

import math
import py_trees
import py_trees_ros
import time
import threading
import operator

# ==========================================================
# 0. HỆ TỌA ĐỘ LƯỚI & BLACKBOARD UTILITIES 
# ==========================================================
GRID_CELL_SIZE = 1.2  
GRID_ORIGIN_X = 0.0  
GRID_ORIGIN_Y = 0.0  

GRID_MIN_COL, GRID_MAX_COL = 1, 3
GRID_MIN_ROW, GRID_MAX_ROW = 1, 4

def manhattan_distance(c1, r1, c2, r2):
    return abs(c1 - c2) + abs(r1 - r2)

# ==========================================================
# 1. BEHAVIORS: ĐIỀU HƯỚNG TỰ DO & THEO LƯỚI (GIỮ NGUYÊN)
# ==========================================================
class GoToRelativePoseBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, dx, dy, target_yaw_deg, odom_topic='/odometry/filtered'):
        super().__init__(name)
        self.ros_node = ros_node
        self.dx = float(dx)
        self.dy = float(dy)
        self.target_yaw_deg = float(target_yaw_deg) 
        self.client = ActionClient(self.ros_node, NavigateToPose, 'navigate_to_pose')
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False
        self.current_pose = None
        self.odom_sub = self.ros_node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg): self.current_pose = msg.pose.pose
    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    def euler_to_quaternion(self, yaw): return [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]
    def setup(self, **kwargs): return self.client.wait_for_server(timeout_sec=5.0)
    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False

    def update(self):
        if not self.goal_sent:
            if self.current_pose is None: return py_trees.common.Status.RUNNING

            curr_x = self.current_pose.position.x
            curr_y = self.current_pose.position.y
            curr_yaw = self.get_yaw(self.current_pose.orientation)

            target_x = curr_x + self.dx * math.cos(curr_yaw) - self.dy * math.sin(curr_yaw)
            target_y = curr_y + self.dx * math.sin(curr_yaw) + self.dy * math.cos(curr_yaw)
            target_yaw = math.radians(self.target_yaw_deg)

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'odom'
            goal_msg.pose.header.stamp = self.ros_node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = target_x
            goal_msg.pose.pose.position.y = target_y
            q = self.euler_to_quaternion(target_yaw)
            goal_msg.pose.pose.orientation.z = q[2]
            goal_msg.pose.pose.orientation.w = q[3]

            self.goal_future = self.client.send_goal_async(goal_msg)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
        if self.result_future is None: self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED else py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()


class TurnToTargetCellBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, odom_topic='/odometry/filtered'):
        super().__init__(name)
        self.ros_node = ros_node
        self.client = ActionClient(self.ros_node, NavigateToPose, 'navigate_to_pose')
        self.blackboard = py_trees.blackboard.Blackboard()
        self.current_pose = None
        self.goal_sent = False
        self.odom_sub = self.ros_node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg): self.current_pose = msg.pose.pose
    def euler_to_quaternion(self, yaw): return [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]
    def setup(self, **kwargs): return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False
        curr_c, curr_r = self.blackboard.robot_pos
        target_c, target_r = self.blackboard.target_cell

        if target_c > curr_c: default_yaw = 0.0      
        elif target_c < curr_c: default_yaw = 180.0  
        elif target_r > curr_r: default_yaw = 90.0   
        elif target_r < curr_r: default_yaw = -90.0  
        else: default_yaw = 0.0

        if (target_c, target_r) in [(2, 1), (1, 4), (3, 4)]:
            self.target_yaw = 0.0
        else:
            self.target_yaw = default_yaw

    def update(self):
        if self.target_yaw is None: return py_trees.common.Status.SUCCESS
        if not self.goal_sent:
            if self.current_pose is None: return py_trees.common.Status.RUNNING

            target_x = self.current_pose.position.x
            target_y = self.current_pose.position.y
            target_yaw_rad = math.radians(self.target_yaw)

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'odom'
            goal_msg.pose.header.stamp = self.ros_node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = target_x
            goal_msg.pose.pose.position.y = target_y
            q = self.euler_to_quaternion(target_yaw_rad)
            goal_msg.pose.pose.orientation.z = q[2]
            goal_msg.pose.pose.orientation.w = q[3]

            self.goal_future = self.client.send_goal_async(goal_msg)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
        if self.result_future is None: self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED else py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()

class MoveRelativeOdomBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, climb_dist=0.3, flat_dist=1.2, odom_topic='/odometry/filtered'):
        super().__init__(name)
        self.ros_node = ros_node
        self.climb_dist = float(climb_dist)
        self.flat_dist = float(flat_dist)
        self.client = ActionClient(self.ros_node, NavigateToPose, 'navigate_to_pose')
        self.blackboard = py_trees.blackboard.Blackboard()
        self.current_pose = None
        self.goal_sent = False
        self.odom_sub = self.ros_node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg): self.current_pose = msg.pose.pose
    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    def euler_to_quaternion(self, yaw): return [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]
    def setup(self, **kwargs): return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False

    def update(self):
        if not self.goal_sent:
            if self.current_pose is None: return py_trees.common.Status.RUNNING

            curr_c, curr_r = self.blackboard.robot_pos
            target_c, target_r = self.blackboard.target_cell
            
            curr_x = self.current_pose.position.x
            curr_y = self.current_pose.position.y
            curr_yaw = self.get_yaw(self.current_pose.orientation)

            dc = target_c - curr_c
            dr = target_r - curr_r
            if dc == 0 and dr == 0: return py_trees.common.Status.SUCCESS
            
            path_yaw = math.atan2(dr, dc)
            just_climbed = getattr(self.blackboard, 'just_climbed', False)
            dist = self.climb_dist if just_climbed else self.flat_dist
            
            target_x = curr_x + dist * math.cos(path_yaw)
            target_y = curr_y + dist * math.sin(path_yaw)
            target_yaw = curr_yaw  

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'odom'
            goal_msg.pose.header.stamp = self.ros_node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = target_x
            goal_msg.pose.pose.position.y = target_y
            q = self.euler_to_quaternion(target_yaw)
            goal_msg.pose.pose.orientation.z = q[2]
            goal_msg.pose.pose.orientation.w = q[3]

            self.goal_future = self.client.send_goal_async(goal_msg)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
        if self.result_future is None: self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING

        if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.blackboard.robot_pos = self.blackboard.target_cell
            visit_path = self.blackboard.visit_path
            if visit_path and len(visit_path) >= 2 and visit_path[-2] == self.blackboard.robot_pos:
                visit_path.pop() 
            else:
                visit_path.append(self.blackboard.robot_pos) 
                
            self.blackboard.just_climbed = False 
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()


# ==========================================================
# 2. BEHAVIORS: ĐỌC PARAMETER & LEO BẬC (GIỮ NGUYÊN)
# ==========================================================
class ClimbStepBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, velocity):
        super().__init__(name)
        self.ros_node = ros_node
        self.velocity = float(velocity)
        self.client = ActionClient(self.ros_node, ClimbStep, 'climb_step')
        self.goal_future = self.result_future = self.goal_handle = None

    def setup(self, **kwargs): return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        self.ros_node.get_logger().info(f"[{self.name}] 🧗 Đang chạy ClimbStep với V={self.velocity}m/s")
        goal_msg = ClimbStep.Goal()
        goal_msg.velocity = self.velocity
        self.goal_future = self.client.send_goal_async(goal_msg)

    def update(self):
        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
        if self.result_future is None: self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED else py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()

class IsClimbingUpCondition(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node):
        super().__init__(name)
        self.ros_node = ros_node
        self.blackboard = py_trees.blackboard.Blackboard()
        self.elevation_map = {}
        self.default_elevation = 400.0

    def setup(self, **kwargs):
        cols = self.ros_node.get_parameter('map_cols').value
        rows = self.ros_node.get_parameter('map_rows').value
        heights = self.ros_node.get_parameter('map_heights').value
        self.default_elevation = self.ros_node.get_parameter('default_elevation').value
        self.elevation_map = {(c, r): float(h) for c, r, h in zip(cols, rows, heights)}
        return True

    def update(self):
        curr_c, curr_r = self.blackboard.robot_pos
        target_c, target_r = self.blackboard.target_cell
        h_current = self.elevation_map.get((curr_c, curr_r), self.default_elevation)
        h_target = self.elevation_map.get((target_c, target_r), self.default_elevation)
        
        if h_target > h_current: return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class DynamicClimbStepBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node):
        super().__init__(name)
        self.ros_node = ros_node
        self.client = ActionClient(self.ros_node, ClimbStep, 'climb_step')
        self.goal_future = self.result_future = self.goal_handle = None
        self.needs_climb = False
        self.blackboard = py_trees.blackboard.Blackboard()
        self.elevation_map = {}
        self.default_elevation = 400.0

    def setup(self, **kwargs):
        cols = self.ros_node.get_parameter('map_cols').value
        rows = self.ros_node.get_parameter('map_rows').value
        heights = self.ros_node.get_parameter('map_heights').value
        self.default_elevation = self.ros_node.get_parameter('default_elevation').value
        self.elevation_map = {(c, r): float(h) for c, r, h in zip(cols, rows, heights)}
        return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        curr_c, curr_r = self.blackboard.robot_pos
        target_c, target_r = self.blackboard.target_cell

        h_current = self.elevation_map.get((curr_c, curr_r), self.default_elevation)
        h_target = self.elevation_map.get((target_c, target_r), self.default_elevation)
        delta_h = h_target - h_current

        if delta_h == 0:
            self.needs_climb = False
            self.blackboard.just_climbed = False
            return

        self.needs_climb = True
        velocity = 0.3 if delta_h > 0 else 0.1
        goal_msg = ClimbStep.Goal()
        goal_msg.velocity = velocity
        self.goal_future = self.client.send_goal_async(goal_msg)

    def update(self):
        if not self.needs_climb: return py_trees.common.Status.SUCCESS
        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
        if self.result_future is None: self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING

        if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED: 
            self.blackboard.just_climbed = True  
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self.needs_climb and getattr(self, 'goal_handle', None):
            self.goal_handle.cancel_goal_async()


# ==========================================================
# 3. CÁC BEHAVIOR PHẦN CỨNG (ARM, VISION, LIDAR, WAIT)
# ==========================================================
class WallAlignmentBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, window_degrees, goal_distance):
        super().__init__(name)
        self.ros_node, self.window_degrees, self.goal_distance = ros_node, float(window_degrees), float(goal_distance)
        self.client = ActionClient(self.ros_node, WallAlignment, 'align_and_approach_wall')
        self.goal_handle = self.goal_future = self.result_future = None

    def setup(self, **kwargs): return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_handle = self.goal_future = self.result_future = None
        goal_msg = WallAlignment.Goal()
        goal_msg.window_degrees, goal_msg.goal_distance = self.window_degrees, self.goal_distance
        self.goal_future = self.client.send_goal_async(goal_msg)

    def update(self):
        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
        if self.result_future is None: self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED else py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()

# --- [CẬP NHẬT 1]: Node AI Vision đọc thông báo từ Action Server ---
class FollowTargetBehavior(py_trees.behaviour.Behaviour):
    """
    Gọi Action Server AI Vision. 
    Lưu trực tiếp kết quả (real/fake/empty/depth_lost) vào Blackboard để xử lý logic.
    """
    def __init__(self, name, ros_node, target_id, desired_distance_mm):
        super().__init__(name)
        self.ros_node = ros_node
        self.target_id = target_id
        self.desired_distance_mm = float(desired_distance_mm)
        self.client = ActionClient(self.ros_node, FollowTarget, 'follow_target')
        self.goal_handle = self.goal_future = self.result_future = None
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, **kwargs): return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_handle = self.goal_future = self.result_future = None
        self.ros_node.get_logger().info(f"[{self.name}] 👁️ Tìm mục tiêu ID={self.target_id}...")
        goal_msg = FollowTarget.Goal()
        goal_msg.target_id = self.target_id
        goal_msg.desired_distance_mm = self.desired_distance_mm
        self.goal_future = self.client.send_goal_async(goal_msg)

    def update(self):
        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
        if self.result_future is None: self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        
        result = self.result_future.result()
        
        # Lưu TRỰC TIẾP thông điệp trả về vào Blackboard (real, fake, empty...)
        tracking_status = result.result.message.upper()
        self.blackboard.latest_box_detection = tracking_status
        
        self.ros_node.get_logger().info(f"[{self.name}] AI Trả về trạng thái: {tracking_status}")
        
        if result.status == GoalStatus.STATUS_SUCCEEDED and tracking_status == "REAL":
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()

class MoveArmBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, target_pose):
        super().__init__(name)
        self.ros_node, self.target = ros_node, target_pose
        self.client = ActionClient(self.ros_node, MoveArm, 'move_arm')
        self.goal_handle = self.goal_future = self.result_future = None

    def setup(self, **kwargs): return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_handle = self.goal_future = self.result_future = None
        goal_msg = MoveArm.Goal()
        goal_msg.arm1, goal_msg.arm2, goal_msg.arm3, goal_msg.gripper = map(float, self.target)
        self.goal_future = self.client.send_goal_async(goal_msg)

    def update(self):
        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
        if self.result_future is None: self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED else py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()

# Cần import Action ở đầu file (nếu chưa có)
# from ak60_bringup.action import ArmSequence

class ArmSequenceBTNode(py_trees.behaviour.Behaviour):
    """
    Điều khiển tay máy ESP32 thông qua Action Server 'esp32_arm_server'.
    """
    def __init__(self, name: str, ros_node: Node, pose_name: str, duration: float = 1.0):
        super().__init__(name)
        self.ros_node = ros_node
        self.pose_name = pose_name
        self.duration = float(duration)
        self.action_client = None
        self._goal_future = self._result_future = self._goal_handle = None

    def setup(self, **kwargs):
        # Import đúng interface theo file Action Server của bạn
        from ak60_bringup.action import ArmSequence 
        self.action_client = ActionClient(self.ros_node, ArmSequence, 'esp32_arm_server')
        
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.ros_node.get_logger().error(f"[{self.name}] ❌ Không tìm thấy Action Server 'esp32_arm_server'!")
            return False
        return True

    def initialise(self):
        from ak60_bringup.action import ArmSequence
        self.ros_node.get_logger().info(f"[{self.name}] 🦾 Gửi lệnh tay gắp ESP32: '{self.pose_name}'")
        
        goal_msg = ArmSequence.Goal()
        goal_msg.pose_name = self.pose_name
        goal_msg.duration = self.duration
        
        self._goal_future = self.action_client.send_goal_async(goal_msg)
        self._result_future = self._goal_handle = None

    def update(self):
        if self._goal_future and not self._goal_future.done(): return py_trees.common.Status.RUNNING
        
        if self._goal_future and self._goal_handle is None:
            self._goal_handle = self._goal_future.result()
            if not self._goal_handle.accepted: 
                self.ros_node.get_logger().error(f"[{self.name}] ❌ Server từ chối lệnh '{self.pose_name}'!")
                return py_trees.common.Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()

        if self._result_future and not self._result_future.done(): return py_trees.common.Status.RUNNING

        if self._result_future and self._result_future.done():
            result = self._result_future.result().result
            if result.success:
                self.ros_node.get_logger().info(f"[{self.name}] ✅ Đã hoàn thành '{self.pose_name}'!")
                return py_trees.common.Status.SUCCESS
            else: 
                self.ros_node.get_logger().error(f"[{self.name}] ❌ Thất bại khi chạy '{self.pose_name}'!")
                return py_trees.common.Status.FAILURE
                
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self._goal_handle and not getattr(self._result_future, 'done', lambda: True)():
            self.ros_node.get_logger().warn(f"[{self.name}] 🛑 Hủy lệnh đang chạy '{self.pose_name}'...")
            self._goal_handle.cancel_goal_async()


class RosWaitBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, delay_sec):
        super().__init__(name)
        self.ros_node = ros_node
        self.delay_sec = delay_sec
        self.start_time = None

    def initialise(self):
        self.start_time = self.ros_node.get_clock().now()
        self.ros_node.get_logger().info(f"[{self.name}] Đang chờ {self.delay_sec} giây...")

    def update(self):
        # Tính thời gian đã trôi qua kể từ initialise()
        elapsed_time = (self.ros_node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_time >= self.delay_sec:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

# ==========================================================
# 4. HÀM TẠO CHUỖI PICK & PLACE 
# ==========================================================
def build_tool_assembly_sequence(ros_node):
    """Chuỗi gắp dụng cụ bằng ESP32 khép kín theo các PREDEFINED_POSES"""
    seq = py_trees.composites.Sequence(name="Quy_Trinh_Lay_Dung_Cu", memory=True)
    
    seq.add_children([
        ArmSequenceBTNode("Init_Home", ros_node, "home", duration=2.0),
        ArmSequenceBTNode("Approach_J4", ros_node, "approach_j4", duration=1.0),
        ArmSequenceBTNode("Approach_J3", ros_node, "approach_j3", duration=2.0),
        ArmSequenceBTNode("Grasp_Tool", ros_node, "grasp", duration=1.0),
        ArmSequenceBTNode("Move_Back_J4", ros_node, "move_back_J4", duration=1.0),
        ArmSequenceBTNode("Approach_J1", ros_node, "approach_j1", duration=1.0),
        # ArmSequenceBTNode("Move_Back_J3", ros_node, "move_back_J3", duration=1.0),
        # ArmSequenceBTNode("Final_Home", ros_node, "home", duration=2.0),
        ArmSequenceBTNode("Assemble_Action", ros_node, "assemble_sequence", duration=20.0),
    ])
    
    return seq

def build_pick_and_place_sequence(ros_node):
    presets = {
        "home": [400.0, 0.0, 00.0, 45.0],     # vi tri nghi
        "pos1": [400.0, 80.0, -10.0, 75.0],   # tiep can
        "pos2": [400.0, 80.0, -10.0, 15.0],   # kep
        "pos3": [400.0, 80.0, 90.0, 15.0],    # lat j3  
        "pos4": [400.0, 5.0, 90.0, 15.0],     # lat j2
        "pos5": [400.0, 0.0, 90.0, 45.0],     # nha hop
    }
    arm_sequence = py_trees.composites.Sequence(name="Chuỗi Gắp Hộp", memory=True)
    steps = [
        ("Home_1", "home"), ("Approach", "pos1"), ("Lower", "pos2"), 
        ("Grip", "pos3"), ("Lift", "pos4"), ("Drop", "pos5"), ("Home_2", "home")
    ]
    for step_name, preset_key in steps:
        arm_sequence.add_child(MoveArmBehavior(f"Arm: {step_name}", ros_node, target_pose=presets[preset_key]))
        if step_name != "Home_2":
            arm_sequence.add_child(RosWaitBehavior(f"Wait", ros_node, delay_sec=1.0))
    return arm_sequence

# ==========================================================
# 5. LOGIC LƯỚI & QUẢN LÝ BLACKBOARD
# ==========================================================
class DecideNextGridCellAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, mode):
        super().__init__(name)
        self.ros_node, self.mode = ros_node, mode
        self.blackboard = py_trees.blackboard.Blackboard()
        self.EXIT_CELLS = [(1, 4), (3, 4)] 

    def update(self):
        curr_c, curr_r = self.blackboard.robot_pos
        grid_status, visit_path = self.blackboard.grid_status, self.blackboard.visit_path
        
        possible_neighbors = [(curr_c + dc, curr_r + dr) for dc, dr in [(-1, 0), (1, 0), (0, -1), (0, 1)] 
                              if GRID_MIN_COL <= curr_c + dc <= GRID_MAX_COL and GRID_MIN_ROW <= curr_r + dr <= GRID_MAX_ROW]

        accessible_neighbors = [n for n in possible_neighbors if grid_status[n]['state'] == 'accessible']
        if not accessible_neighbors: return py_trees.common.Status.FAILURE

        unvisited_neighbors = [n for n in accessible_neighbors if not grid_status[n]['visited']]
        target_cell = None

        if self.mode == "EXIT":
            source_list = unvisited_neighbors if unvisited_neighbors else [n for n in accessible_neighbors if grid_status[n]['visited']]
            closest_cell, min_dist = None, float('inf')
            for cell in source_list:
                dist = min([manhattan_distance(cell[0], cell[1], e[0], e[1]) for e in self.EXIT_CELLS])
                if dist < min_dist or (dist == min_dist and cell[1] > closest_cell[1]):
                    min_dist, closest_cell = dist, cell
            target_cell = closest_cell
        else: 
            if unvisited_neighbors: target_cell = sorted(unvisited_neighbors, key=lambda n: (n[1], n[0]), reverse=True)[0]
            else: target_cell = visit_path[-2] if len(visit_path) >= 2 else visit_path[0]

        if target_cell:
            self.blackboard.target_cell = target_cell
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IncrementRealCountAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
    def update(self):
        self.blackboard.real_box_count += 1
        return py_trees.common.Status.SUCCESS

class MarkAsVisitedAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
    def update(self):
        tc = self.blackboard.target_cell
        self.blackboard.grid_status[tc]['scanned'] = True
        self.blackboard.grid_status[tc]['state'] = 'accessible'
        return py_trees.common.Status.SUCCESS

class MarkAsObstacleAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
    def update(self):
        tc = self.blackboard.target_cell
        self.blackboard.grid_status[tc]['scanned'] = True
        self.blackboard.grid_status[tc]['state'] = 'obstacle'
        return py_trees.common.Status.SUCCESS

class IsAtExitCellCondition(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.EXIT_CELLS = [(1, 4), (3, 4)]
    def update(self): return py_trees.common.Status.SUCCESS if self.blackboard.robot_pos in self.EXIT_CELLS else py_trees.common.Status.FAILURE

class CheckBlackboardValue(py_trees.behaviour.Behaviour):
    """Behavior tự viết để kiểm tra giá trị Blackboard, chống lỗi version py_trees"""
    def __init__(self, name, variable_name, expected_value, op=operator.eq):
        super().__init__(name)
        self.variable_name = variable_name
        self.expected_value = expected_value
        self.op = op
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        val = getattr(self.blackboard, self.variable_name, None)
        if val is not None and self.op(val, self.expected_value):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class KeepRunningUntilSuccess(py_trees.decorators.Decorator):
    """
    Decorator: Ép Node con chạy liên tục. 
    Nếu Node con trả về FAILURE -> Đổi thành RUNNING để tick lại vào chu kỳ sau.
    Nếu Node con trả về SUCCESS -> Trả về SUCCESS để thoát vòng lặp.
    """
    def __init__(self, name, child):
        # Truyền node con (child) vào thẳng constructor của Decorator
        super().__init__(name=name, child=child)

    def update(self):
        # self.decorated chính là node con (child)
        if getattr(self, 'decorated', None) is None:
            return py_trees.common.Status.FAILURE
            
        if self.decorated.status == py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.SUCCESS
            
        # Dù fail hay đang chạy, đều báo RUNNING để Behavior Tree lặp lại
        return py_trees.common.Status.RUNNING

def build_move_subtree(ros_node):
    """Tạo ra một bản sao mới của chuỗi hành động xử lý và đi vào ô lưới"""
    move_subtree = py_trees.composites.Sequence("Process & Enter Cell", memory=True)
    
    move_subtree.add_child(TurnToTargetCellBehavior("Turn To Face Cell", ros_node))
    
    align_vision = WallAlignmentBehavior("Align Before Vision", ros_node, window_degrees=20.0, goal_distance=0.3)
    move_subtree.add_child(py_trees.decorators.FailureIsSuccess("Ignore Align Fail 1", align_vision))

    # Gọi AI chạy 1 lần duy nhất
    move_subtree.add_child(py_trees.decorators.FailureIsSuccess("Run AI", 
        FollowTargetBehavior("Run AI (ID=1)", ros_node, target_id=1, desired_distance_mm=250.0)
    ))
    
    box_logic_selector = py_trees.composites.Selector("Box Logic (Real/Fake/Empty)", memory=False)

    real_seq = py_trees.composites.Sequence("Hunt: REAL Box", memory=True)
    real_seq.add_child(CheckBlackboardValue("Is Real?", "latest_box_detection", "REAL", operator.eq))
    real_seq.add_child(build_pick_and_place_sequence(ros_node)) 
    real_seq.add_child(IncrementRealCountAction("Logic: Add Score"))
    real_seq.add_child(MarkAsVisitedAction("Logic: Mark Accessible"))

    fake_seq = py_trees.composites.Sequence("Hunt: FAKE Box", memory=True)
    fake_seq.add_child(CheckBlackboardValue("Is Fake?", "latest_box_detection", "FAKE", operator.eq))
    fake_seq.add_child(MarkAsObstacleAction("Logic: Mark Obstacle"))
    fake_seq.add_child(py_trees.behaviours.Failure("Block Route")) 

    empty_seq = py_trees.composites.Sequence("Hunt: EMPTY", memory=True)
    empty_seq.add_child(MarkAsVisitedAction("Logic: Mark Accessible")) 

    box_logic_selector.add_children([real_seq, fake_seq, empty_seq])
    move_subtree.add_child(box_logic_selector)

    climb_up_align_seq = py_trees.composites.Sequence("Align if Climbing Up", memory=True)
    climb_up_align_seq.add_child(IsClimbingUpCondition("Check Elevation > 0", ros_node))
    climb_up_align_seq.add_child(WallAlignmentBehavior("Align Before Climb", ros_node, window_degrees=20.0, goal_distance=0.3))
    move_subtree.add_child(py_trees.decorators.FailureIsSuccess("Ignore Align Fail 2", climb_up_align_seq))

    move_subtree.add_child(DynamicClimbStepBehavior("Hardware: Auto Climb", ros_node))
    move_subtree.add_child(MoveRelativeOdomBehavior("Hardware: Move Relative into Cell Center", ros_node, climb_dist=0.3, flat_dist=1.2))
    
    return move_subtree

# ==========================================================
# 6. DỰNG CÂY QUYẾT ĐỊNH
# ==========================================================
def create_tree(ros_node):
    root = py_trees.composites.Sequence("Main Mission", memory=True)
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.real_box_count = 0
    blackboard.robot_pos = (2, 0) 
    blackboard.visit_path = [blackboard.robot_pos] 
    blackboard.grid_status = {(c, r): {'scanned': False, 'state': 'accessible', 'visited': False} for c in range(1, 4) for r in range(1, 5)}

    # --- PHASE 1: LẤY DỤNG CỤ VÀ TIẾN RA CỬA LƯỚI ---
    init_seq = py_trees.composites.Sequence("Khoi_Dong_Va_Lay_Dung_Cu", memory=True)
    init_seq.add_child(GoToRelativePoseBehavior("Tien_Lay_Dung_Cu", ros_node, dx=0.3, dy=1.0, target_yaw_deg=0.0))
    init_seq.add_child(build_tool_assembly_sequence(ros_node))
    init_seq.add_child(GoToRelativePoseBehavior("Tien_Ra_Cua_Cell", ros_node, dx=1.6, dy=2.5, target_yaw_deg=0.0))
    init_seq.add_child(py_trees.behaviours.SetBlackboardVariable("Target=(2,1)", "target_cell", (2, 1), overwrite=True))
    root.add_child(init_seq)

    # --- PHASE 3: KỊCH BẢN THOÁT KHỎI SA BÀN ---
    exit_sequence = py_trees.composites.Sequence("Chuoi_Thoat_Khoi_San", memory=True)
    exit_sequence.add_child(GoToRelativePoseBehavior("Thoat: Tien 0.25m", ros_node, dx=0.25, dy=0.0, target_yaw_deg=0.0))
    exit_sequence.add_child(ClimbStepBehavior("Thoat: Xuong bac v=0.1", ros_node, velocity=0.1))
    
    exit_sequence.add_child(GoToRelativePoseBehavior("Thoat: Tien 0.25m sau xuong", ros_node, dx=0.25, dy=0.0, target_yaw_deg=0.0))
    exit_sequence.add_child(py_trees.decorators.FailureIsSuccess("Ignore Align Exit", 
        WallAlignmentBehavior("Thoat: Align Lidar 1", ros_node, window_degrees=60.0, goal_distance=0.4)))
    
    branch_exit_selector = py_trees.composites.Selector("Chon_Huong_Dich_Trai", memory=False)
    
    seq_3_4 = py_trees.composites.Sequence("Dich_Trai_Tu_3_4", memory=True)
    seq_3_4.add_child(CheckBlackboardValue("La o (3,4)?", "robot_pos", (3, 4), operator.eq))
    seq_3_4.add_child(GoToRelativePoseBehavior("Thoat: Sang trai 1.3m", ros_node, dx=0.0, dy=1.3, target_yaw_deg=0.0))
    
    seq_1_4 = py_trees.composites.Sequence("Dich_Trai_Tu_1_4", memory=True)
    seq_1_4.add_child(CheckBlackboardValue("La o (1,4)?", "robot_pos", (1, 4), operator.eq))
    seq_1_4.add_child(GoToRelativePoseBehavior("Thoat: Sang trai 3.7m", ros_node, dx=0.0, dy=3.7, target_yaw_deg=0.0))
    
    branch_exit_selector.add_children([seq_3_4, seq_1_4])
    exit_sequence.add_child(branch_exit_selector)
    
    exit_sequence.add_child(py_trees.decorators.FailureIsSuccess("Ignore Align Ramp", 
        WallAlignmentBehavior("Thoat: Align Lidar Truoc Doc", ros_node, window_degrees=60.0, goal_distance=0.4)))
    exit_sequence.add_child(ClimbStepBehavior("Thoat: Len doc cuoi v=0.5", ros_node, velocity=0.5))
    exit_sequence.add_child(GoToRelativePoseBehavior("Thoat: Quay phai 90 do", ros_node, dx=0.0, dy=0.0, target_yaw_deg=-90.0))
    exit_sequence.add_child(GoToRelativePoseBehavior("Thoat: Tien thang 4m", ros_node, dx=4.0, dy=0.0, target_yaw_deg=-90.0))

    # --- LOOP ĐIỀU HƯỚNG ---
    mode_selector = py_trees.composites.Selector("Mode", memory=False)
    
    exit_check_seq = py_trees.composites.Sequence("Mode: EXIT Goal", memory=True)
    exit_check_seq.add_child(CheckBlackboardValue("Real>=2?", "real_box_count", 2, operator.ge))
    exit_check_seq.add_child(IsAtExitCellCondition("At Exit?"))
    exit_check_seq.add_child(exit_sequence) 
    exit_check_seq.add_child(py_trees.behaviours.Success("Mission Accomplished!"))
    mode_selector.add_child(exit_check_seq)

    path_exit_seq = py_trees.composites.Sequence("Mode: FIND Exit", memory=True)
    path_exit_seq.add_child(CheckBlackboardValue("Real>=2?", "real_box_count", 2, operator.ge))
    path_exit_seq.add_child(DecideNextGridCellAction("Route: Exit", ros_node, mode="EXIT"))
    # GỌI HÀM THAY VÌ TRUYỀN BIẾN
    path_exit_seq.add_child(build_move_subtree(ros_node)) 
    mode_selector.add_child(path_exit_seq)

    path_hunt_seq = py_trees.composites.Sequence("Mode: HUNT", memory=True)
    path_hunt_seq.add_child(DecideNextGridCellAction("Route: Explore", ros_node, mode="EXPLORE"))
    # GỌI HÀM THAY VÌ TRUYỀN BIẾN
    path_hunt_seq.add_child(build_move_subtree(ros_node)) 
    mode_selector.add_child(path_hunt_seq)

    loop_decorator = KeepRunningUntilSuccess("Loop: Explore/Exit", child=mode_selector)
    
    grid_phase_seq = py_trees.composites.Sequence("Vao_Luoi_Va_Lap", memory=True)
    # GỌI HÀM LẦN NỮA CHO Ô MỒI ĐẦU TIÊN (2,1)
    grid_phase_seq.add_child(build_move_subtree(ros_node)) 
    grid_phase_seq.add_child(loop_decorator) 
    
    root.add_child(grid_phase_seq)

    return root

# ==========================================================
# 7. MAIN LOOP / THREADING
# ==========================================================
def bt_ticker_thread(node, tree, blackboard):
    node.get_logger().info("🚀 Bắt đầu tick Behavior Tree...")
    while rclpy.ok():
        tree.tick_once()
        py_trees.display.print_ascii_tree(tree, show_status=True)
        time.sleep(0.1) 

def main(args=None):
    rclpy.init(args=args)
    node = Node('app_manager_bt_node')
    
    node.declare_parameter('map_cols', [2, 1, 3, 2])
    node.declare_parameter('map_rows', [1, 2, 2, 3])
    node.declare_parameter('map_heights', [600.0, 200.0, 200.0, 600.0])
    node.declare_parameter('default_elevation', 400.0)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    blackboard = py_trees.blackboard.Blackboard()
    tree_root = create_tree(node)
    
    behaviour_tree = py_trees_ros.trees.BehaviourTree(tree_root)
    if not behaviour_tree.setup(timeout=30.0, node=node):
         node.get_logger().error("❌ Không thể setup BT (Lỗi Server Timeout).")
         return

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    bt_thread = threading.Thread(target=bt_ticker_thread, args=(node, behaviour_tree, blackboard))
    bt_thread.daemon = True
    bt_thread.start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        node.get_logger().warn('Dừng khẩn cấp do người dùng.')
    finally:
        behaviour_tree.interrupt()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()

