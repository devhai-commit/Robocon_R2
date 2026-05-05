#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
import py_trees
import math
import operator
import json

from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from ak60_bringup.action import ClimbStep, FollowTarget, MoveArm, WallAlignment, ArmSequence

GRID_CELL_SIZE = 1.2  
GRID_ORIGIN_X = 0.0  
GRID_ORIGIN_Y = 0.0  
GRID_MIN_COL, GRID_MAX_COL = 1, 3
GRID_MIN_ROW, GRID_MAX_ROW = 1, 4

def manhattan_distance(c1, r1, c2, r2):
    return abs(c1 - c2) + abs(r1 - r2)

# ==========================================
# CÁC CLASS LOGIC & BLACKBOARD
# ==========================================
class WaitForStartSignalBehavior(py_trees.behaviour.Behaviour):
    """Đứng chờ tín hiệu JSON từ GUI"""
    def __init__(self, name, ros_node):
        super().__init__(name)
        self.ros_node = ros_node
        self.blackboard = py_trees.blackboard.Blackboard()
        self.start_received = False
        self.sub = self.ros_node.create_subscription(String, '/gui_start_signal', self.signal_callback, 10)

    def signal_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("command") == "START":
                self.blackboard.priority_col = data.get("priority_col", 2)
                self.blackboard.target_boxes_list = [tuple(box) for box in data.get("target_boxes", [])]
                self.start_received = True
        except Exception as e:
            self.ros_node.get_logger().error(f"Lỗi giải mã GUI: {e}")

    def update(self):
        if self.start_received: return py_trees.common.Status.SUCCESS
        self.ros_node.get_logger().info(f"[{self.name}] 🟡 Đang chờ ấn nút START trên màn hình...", throttle_duration_sec=2.0)
        return py_trees.common.Status.RUNNING

class CalculatePickElevationAction(py_trees.behaviour.Behaviour):
    """Tính toán cao độ Arm 1 dựa trên chênh lệch bậc địa hình"""
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
        
        if h_target < h_current: self.blackboard.current_arm1_z = 0.0       # Thấp hơn
        elif h_target > h_current: self.blackboard.current_arm1_z = 325.0   # Cao hơn
        else: self.blackboard.current_arm1_z = 125.0                        # Bằng nhau
            
        self.ros_node.get_logger().info(f"[{self.name}] 🤖 Arm 1 Z = {self.blackboard.current_arm1_z}mm")
        return py_trees.common.Status.SUCCESS

class DecideNextGridCellAction(py_trees.behaviour.Behaviour):
    """Quyết định ô tiếp theo dựa trên GUI Config"""
    def __init__(self, name, ros_node, mode):
        super().__init__(name)
        self.ros_node, self.mode = ros_node, mode
        self.blackboard = py_trees.blackboard.Blackboard()
        self.EXIT_CELLS = [(1, 4), (3, 4)] 

    def update(self):
        curr_c, curr_r = self.blackboard.robot_pos
        grid_status, visit_path = self.blackboard.grid_status, self.blackboard.visit_path
        p_col = getattr(self.blackboard, 'priority_col', 2)
        t_boxes = getattr(self.blackboard, 'target_boxes_list', [])

        possible_neighbors = [(curr_c + dc, curr_r + dr) for dc, dr in [(-1, 0), (1, 0), (0, -1), (0, 1)] 
                              if GRID_MIN_COL <= curr_c + dc <= GRID_MAX_COL and GRID_MIN_ROW <= curr_r + dr <= GRID_MAX_ROW]
        accessible_neighbors = [n for n in possible_neighbors if grid_status[n]['state'] == 'accessible']
        if not accessible_neighbors: return py_trees.common.Status.FAILURE
        unvisited = [n for n in accessible_neighbors if not grid_status[n]['visited']]

        if self.mode == "EXPLORE":
            target_cell = None
            target_neighbors = [n for n in unvisited if n in t_boxes]
            if target_neighbors: 
                target_cell = sorted(target_neighbors, key=lambda n: (n[1], n[0]), reverse=True)[0]
            elif any(n[0] == p_col for n in unvisited):
                target_cell = sorted([n for n in unvisited if n[0] == p_col], key=lambda n: n[1], reverse=True)[0]
            elif unvisited: 
                target_cell = sorted(unvisited, key=lambda n: (n[1], n[0]), reverse=True)[0]
            else: 
                target_cell = visit_path[-2] if len(visit_path) >= 2 else visit_path[0]
        else:
            closest_cell, min_dist = None, float('inf')
            for cell in accessible_neighbors:
                dist = min([manhattan_distance(cell[0], cell[1], e[0], e[1]) for e in self.EXIT_CELLS])
                if dist < min_dist: min_dist, closest_cell = dist, cell
            target_cell = closest_cell

        if target_cell:
            self.blackboard.target_cell = target_cell
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class CheckBlackboardValue(py_trees.behaviour.Behaviour):
    def __init__(self, name, variable_name, expected_value, op=operator.eq):
        super().__init__(name)
        self.variable_name = variable_name
        self.expected_value = expected_value
        self.op = op
        self.blackboard = py_trees.blackboard.Blackboard()
    def update(self):
        val = getattr(self.blackboard, self.variable_name, None)
        if val is not None and self.op(val, self.expected_value): return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class KeepRunningUntilSuccess(py_trees.decorators.Decorator):
    def __init__(self, name, child): super().__init__(name=name, child=child)
    def update(self):
        if getattr(self, 'decorated', None) is None: return py_trees.common.Status.FAILURE
        if self.decorated.status == py_trees.common.Status.SUCCESS: return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class RosWaitBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, delay_sec):
        super().__init__(name)
        self.ros_node, self.delay_sec = ros_node, delay_sec
        self.start_time = None
    def initialise(self): self.start_time = self.ros_node.get_clock().now()
    def update(self):
        elapsed = (self.ros_node.get_clock().now() - self.start_time).nanoseconds / 1e9
        return py_trees.common.Status.SUCCESS if elapsed >= self.delay_sec else py_trees.common.Status.RUNNING

class IncrementRealCountAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name); self.blackboard = py_trees.blackboard.Blackboard()
    def update(self): self.blackboard.real_box_count += 1; return py_trees.common.Status.SUCCESS

class MarkAsVisitedAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name); self.blackboard = py_trees.blackboard.Blackboard()
    def update(self):
        self.blackboard.grid_status[self.blackboard.target_cell]['scanned'] = True
        self.blackboard.grid_status[self.blackboard.target_cell]['state'] = 'accessible'
        return py_trees.common.Status.SUCCESS

class MarkAsObstacleAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name); self.blackboard = py_trees.blackboard.Blackboard()
    def update(self):
        self.blackboard.grid_status[self.blackboard.target_cell]['scanned'] = True
        self.blackboard.grid_status[self.blackboard.target_cell]['state'] = 'obstacle'
        return py_trees.common.Status.SUCCESS

class IsAtExitCellCondition(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name); self.blackboard = py_trees.blackboard.Blackboard()
    def update(self): return py_trees.common.Status.SUCCESS if self.blackboard.robot_pos in [(1,4), (3,4)] else py_trees.common.Status.FAILURE


# ==========================================
# CÁC CLASS HARDWARE & ĐIỀU HƯỚNG
# ==========================================
class GoToRelativePoseBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, dx, dy, target_yaw_deg, odom_topic='/odometry/filtered'):
        super().__init__(name)
        self.ros_node, self.dx, self.dy, self.target_yaw_deg = ros_node, float(dx), float(dy), float(target_yaw_deg)
        self.client = ActionClient(self.ros_node, NavigateToPose, 'navigate_to_pose')
        self.current_pose = None
        self.odom_sub = self.ros_node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg): self.current_pose = msg.pose.pose
    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    def euler_to_quaternion(self, yaw): return [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]
    
    def setup(self, **kwargs):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.ros_node.get_logger().error(f"[{self.name}] ❌ TẮT SERVER: 'navigate_to_pose'")
            raise RuntimeError("Mất kết nối Nav2 Server")
        return True

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False

    def update(self):
        if not self.goal_sent:
            if self.current_pose is None: return py_trees.common.Status.RUNNING
            curr_yaw = self.get_yaw(self.current_pose.orientation)
            target_x = self.current_pose.position.x + self.dx * math.cos(curr_yaw) - self.dy * math.sin(curr_yaw)
            target_y = self.current_pose.position.y + self.dx * math.sin(curr_yaw) + self.dy * math.cos(curr_yaw)
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'odom'
            goal_msg.pose.header.stamp = self.ros_node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y = target_x, target_y
            q = self.euler_to_quaternion(math.radians(self.target_yaw_deg))
            goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.w = q[2], q[3]

            self.goal_future = self.client.send_goal_async(goal_msg)
            self.goal_sent = True; return py_trees.common.Status.RUNNING

        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
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
        self.odom_sub = self.ros_node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg): self.current_pose = msg.pose.pose
    def euler_to_quaternion(self, yaw): return [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]
    def setup(self, **kwargs):
        if not self.client.wait_for_server(timeout_sec=5.0): raise RuntimeError("Mất kết nối Nav2 Server")
        return True

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False
        curr_c, curr_r = self.blackboard.robot_pos
        target_c, target_r = self.blackboard.target_cell

        if target_c > curr_c: default_yaw = 90.0      
        elif target_c < curr_c: default_yaw = -90.0  
        elif target_r > curr_r: default_yaw = 0.0   
        elif target_r < curr_r: default_yaw = -180.0  
        else: default_yaw = 0.0
        self.target_yaw = 0.0 if (target_c, target_r) in [(2, 1), (1, 4), (3, 4)] else default_yaw

    def update(self):
        if self.target_yaw is None: return py_trees.common.Status.SUCCESS
        if not self.goal_sent:
            if self.current_pose is None: return py_trees.common.Status.RUNNING
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'odom'
            goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y = self.current_pose.position.x, self.current_pose.position.y
            q = self.euler_to_quaternion(math.radians(self.target_yaw))
            goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.w = q[2], q[3]
            self.goal_future = self.client.send_goal_async(goal_msg)
            self.goal_sent = True; return py_trees.common.Status.RUNNING

        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED else py_trees.common.Status.FAILURE
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()


class MoveRelativeOdomBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, climb_dist=0.3, flat_dist=1.0, odom_topic='/odometry/filtered'):
        super().__init__(name)
        self.ros_node, self.climb_dist, self.flat_dist = ros_node, float(climb_dist), float(flat_dist)
        self.client = ActionClient(self.ros_node, NavigateToPose, 'navigate_to_pose')
        self.blackboard = py_trees.blackboard.Blackboard()
        self.current_pose = None
        self.odom_sub = self.ros_node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg): self.current_pose = msg.pose.pose
    def get_yaw(self, q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    def euler_to_quaternion(self, yaw): return [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]
    def setup(self, **kwargs):
        if not self.client.wait_for_server(timeout_sec=5.0): raise RuntimeError("Mất kết nối Nav2 Server")
        return True

    def initialise(self): self.goal_future = self.result_future = self.goal_handle = None; self.goal_sent = False
    def update(self):
        if not self.goal_sent:
            if self.current_pose is None: return py_trees.common.Status.RUNNING
            curr_c, curr_r = self.blackboard.robot_pos
            target_c, target_r = self.blackboard.target_cell
            curr_yaw = self.get_yaw(self.current_pose.orientation)

            dc, dr = target_c - curr_c, target_r - curr_r
            if dc == 0 and dr == 0: return py_trees.common.Status.SUCCESS
            
            dist = self.climb_dist if getattr(self.blackboard, 'just_climbed', False) else self.flat_dist
            path_yaw = math.atan2(dr, dc)
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'odom'
            goal_msg.pose.pose.position.x = self.current_pose.position.x + dist * math.cos(path_yaw)
            goal_msg.pose.pose.position.y = self.current_pose.position.y + dist * math.sin(path_yaw)
            q = self.euler_to_quaternion(curr_yaw)
            goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.w = q[2], q[3]

            self.goal_future = self.client.send_goal_async(goal_msg)
            self.goal_sent = True; return py_trees.common.Status.RUNNING

        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING

        if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.blackboard.robot_pos = self.blackboard.target_cell
            if len(self.blackboard.visit_path) >= 2 and self.blackboard.visit_path[-2] == self.blackboard.robot_pos:
                self.blackboard.visit_path.pop() 
            else: self.blackboard.visit_path.append(self.blackboard.robot_pos) 
            self.blackboard.just_climbed = False 
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()


class ClimbStepBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, velocity):
        super().__init__(name); self.ros_node, self.velocity = ros_node, float(velocity)
        self.client = ActionClient(self.ros_node, ClimbStep, 'climb_step')
    def setup(self, **kwargs):
        if not self.client.wait_for_server(timeout_sec=5.0): raise RuntimeError("Mất kết nối ClimbStep")
        return True
    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        goal_msg = ClimbStep.Goal(); goal_msg.velocity = self.velocity
        self.goal_future = self.client.send_goal_async(goal_msg)
    def update(self):
        if getattr(self, 'goal_future', None) is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED else py_trees.common.Status.FAILURE
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()

class IsClimbingUpCondition(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node):
        super().__init__(name); self.ros_node = ros_node; self.blackboard = py_trees.blackboard.Blackboard()
    def setup(self, **kwargs):
        self.elevation_map = {(c, r): float(h) for c, r, h in zip(self.ros_node.get_parameter('map_cols').value, self.ros_node.get_parameter('map_rows').value, self.ros_node.get_parameter('map_heights').value)}
        self.default_elevation = self.ros_node.get_parameter('default_elevation').value; return True
    def update(self):
        h_current = self.elevation_map.get(self.blackboard.robot_pos, self.default_elevation)
        h_target = self.elevation_map.get(self.blackboard.target_cell, self.default_elevation)
        return py_trees.common.Status.SUCCESS if h_target > h_current else py_trees.common.Status.FAILURE

class DynamicClimbStepBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node):
        super().__init__(name); self.ros_node = ros_node; self.client = ActionClient(self.ros_node, ClimbStep, 'climb_step')
        self.blackboard = py_trees.blackboard.Blackboard()
    def setup(self, **kwargs):
        self.elevation_map = {(c, r): float(h) for c, r, h in zip(self.ros_node.get_parameter('map_cols').value, self.ros_node.get_parameter('map_rows').value, self.ros_node.get_parameter('map_heights').value)}
        self.default_elevation = self.ros_node.get_parameter('default_elevation').value
        if not self.client.wait_for_server(timeout_sec=5.0): raise RuntimeError("Mất kết nối ClimbStep")
        return True
    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        h_current = self.elevation_map.get(self.blackboard.robot_pos, self.default_elevation)
        h_target = self.elevation_map.get(self.blackboard.target_cell, self.default_elevation)
        delta_h = h_target - h_current
        if delta_h == 0:
            self.needs_climb = False; self.blackboard.just_climbed = False; return
        self.needs_climb = True
        goal_msg = ClimbStep.Goal(); goal_msg.velocity = 0.3 if delta_h > 0 else 0.1
        self.goal_future = self.client.send_goal_async(goal_msg)
    def update(self):
        if not getattr(self, 'needs_climb', False): return py_trees.common.Status.SUCCESS
        if self.goal_future is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED: 
            self.blackboard.just_climbed = True; return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'needs_climb', False) and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()


class WallAlignmentBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, window_degrees, goal_distance):
        super().__init__(name); self.ros_node, self.window_degrees, self.goal_distance = ros_node, float(window_degrees), float(goal_distance)
        self.client = ActionClient(self.ros_node, WallAlignment, 'align_and_approach_wall')
    def setup(self, **kwargs):
        if not self.client.wait_for_server(timeout_sec=5.0): raise RuntimeError("Mất kết nối Lidar Align")
        return True
    def initialise(self):
        self.goal_handle = self.goal_future = self.result_future = None
        goal_msg = WallAlignment.Goal(); goal_msg.window_degrees, goal_msg.goal_distance = self.window_degrees, self.goal_distance
        self.goal_future = self.client.send_goal_async(goal_msg)
    def update(self):
        if getattr(self, 'goal_future', None) is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED else py_trees.common.Status.FAILURE
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()

class FollowTargetBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, target_id, desired_distance_mm, label_if_found="BOX"):
        super().__init__(name); self.ros_node, self.target_id, self.desired_distance_mm = ros_node, float(target_id), float(desired_distance_mm)
        self.label_if_found = label_if_found; self.client = ActionClient(self.ros_node, FollowTarget, 'follow_target')
        self.blackboard = py_trees.blackboard.Blackboard()
    def setup(self, **kwargs):
        if not self.client.wait_for_server(timeout_sec=5.0): raise RuntimeError("Mất kết nối YOLO Vision")
        return True
    def initialise(self):
        self.goal_handle = self.goal_future = self.result_future = None
        goal_msg = FollowTarget.Goal(); goal_msg.target_id, goal_msg.desired_distance_mm = self.target_id, self.desired_distance_mm
        self.goal_future = self.client.send_goal_async(goal_msg)
    def update(self):
        if getattr(self, 'goal_future', None) is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        result = self.result_future.result(); tracking_status = result.result.message.upper()
        self.blackboard.latest_box_detection = tracking_status
        return py_trees.common.Status.SUCCESS if result.status == GoalStatus.STATUS_SUCCEEDED and tracking_status == "REAL" else py_trees.common.Status.FAILURE
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()

class MoveArmBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, target_pose):
        super().__init__(name); self.ros_node, self.target = ros_node, target_pose
        self.client = ActionClient(self.ros_node, MoveArm, 'move_arm')
        self.blackboard = py_trees.blackboard.Blackboard()
    def setup(self, **kwargs):
        if not self.client.wait_for_server(timeout_sec=5.0): raise RuntimeError("Mất kết nối AK60 Arm")
        return True
    def initialise(self):
        self.goal_handle = self.goal_future = self.result_future = None
        goal_msg = MoveArm.Goal()
        # Tính năng Tự động lấy Z (Auto)
        arm1_val = self.target[0]
        if arm1_val == "auto": arm1_val = getattr(self.blackboard, 'current_arm1_z', 125.0)
        goal_msg.arm1, goal_msg.arm2, goal_msg.arm3, goal_msg.gripper = float(arm1_val), float(self.target[1]), float(self.target[2]), float(self.target[3])
        self.goal_future = self.client.send_goal_async(goal_msg)
    def update(self):
        if getattr(self, 'goal_future', None) is None or not self.goal_future.done(): return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED else py_trees.common.Status.FAILURE
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None): self.goal_handle.cancel_goal_async()

class ArmSequenceBTNode(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, ros_node, pose_name: str, duration: float = 1.0):
        super().__init__(name); self.ros_node, self.pose_name, self.duration = ros_node, pose_name, float(duration)
        self.action_client = None
    def setup(self, **kwargs):
        self.action_client = ActionClient(self.ros_node, ArmSequence, 'arm_sequence_controller')
        if not self.action_client.wait_for_server(timeout_sec=5.0): raise RuntimeError("Mất kết nối ESP32 Arm")
        return True
    def initialise(self):
        self._goal_future = self._result_future = self._goal_handle = None
        goal_msg = ArmSequence.Goal(); goal_msg.pose_name, goal_msg.duration = self.pose_name, self.duration
        self._goal_future = self.action_client.send_goal_async(goal_msg)
    def update(self):
        if getattr(self, '_goal_future', None) and not self._goal_future.done(): return py_trees.common.Status.RUNNING
        if self._goal_future and self._goal_handle is None:
            self._goal_handle = self._goal_future.result()
            if not self._goal_handle.accepted: return py_trees.common.Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
        if self._result_future and not self._result_future.done(): return py_trees.common.Status.RUNNING
        if self._result_future and self._result_future.done():
            return py_trees.common.Status.SUCCESS if self._result_future.result().result.success else py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, '_goal_handle', None) and not getattr(self._result_future, 'done', lambda: True)():
            self._goal_handle.cancel_goal_async()

            