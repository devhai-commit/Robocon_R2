#!/usr/bin/env python3
import operator
import py_trees
import json
from std_msgs.msg import String
from r2_bt.config import GRID_MIN_COL, GRID_MAX_COL, GRID_MIN_ROW, GRID_MAX_ROW, manhattan_distance

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
                # Đổi sang dùng .set() của PyTrees v2
                self.blackboard.set("priority_col", data.get("priority_col", 2))
                self.blackboard.set("target_boxes_list", [tuple(box) for box in data.get("target_boxes", [])])
                self.start_received = True
        except Exception as e:
            self.ros_node.get_logger().error(f"Lỗi giải mã GUI: {e}")

    def update(self):
        if self.start_received: return py_trees.common.Status.SUCCESS
        self.ros_node.get_logger().info(f"[{self.name}] 🟡 Đang chờ ấn nút START trên màn hình...", throttle_duration_sec=2.0)
        return py_trees.common.Status.RUNNING

# class CalculatePickElevationAction(py_trees.behaviour.Behaviour):
#     """Tính toán cao độ Arm 1 dựa trên chênh lệch bậc địa hình"""
#     def __init__(self, name, ros_node):
#         super().__init__(name)
#         self.ros_node = ros_node
#         self.blackboard = py_trees.blackboard.Blackboard()
#         self.elevation_map = {}
#         self.default_elevation = 400.0

#     def setup(self, **kwargs):
#         cols = self.ros_node.get_parameter('map_cols').value
#         rows = self.ros_node.get_parameter('map_rows').value
#         heights = self.ros_node.get_parameter('map_heights').value
#         self.default_elevation = self.ros_node.get_parameter('default_elevation').value
#         self.elevation_map = {(c, r): float(h) for c, r, h in zip(cols, rows, heights)}
#         return True

#     def update(self):
#         rp = self.blackboard.get("robot_pos")
#         tc = self.blackboard.get("target_cell")
#         curr_c, curr_r = rp if rp else (2, 0)
#         target_c, target_r = tc if tc else (2, 1)

#         h_current = self.elevation_map.get((curr_c, curr_r), self.default_elevation)
#         h_target = self.elevation_map.get((target_c, target_r), self.default_elevation)

#         if h_target < h_current: self.blackboard.set("current_arm1_z", 0.0)
#         elif h_target > h_current: self.blackboard.set("current_arm1_z", 325.0)
#         else: self.blackboard.set("current_arm1_z", 125.0)

#         self.ros_node.get_logger().info(f"[{self.name}] 🤖 Arm 1 Z = {self.blackboard.get('current_arm1_z')}mm")
#         return py_trees.common.Status.SUCCESS

class DecideNextGridCellAction(py_trees.behaviour.Behaviour):
    EXIT_CELLS = [(1, 4), (3, 4)]

    def __init__(self, name, ros_node, mode):
        super().__init__(name)
        self.ros_node, self.mode = ros_node, mode
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        rp = self.blackboard.get("robot_pos")
        curr_c, curr_r = rp if rp else (2, 0)
        grid_status = self.blackboard.get("grid_status") or {}
        visit_path = self.blackboard.get("visit_path") or []
        
        p_col = self.blackboard.get("priority_col")
        if p_col is None: p_col = 2
        t_boxes = self.blackboard.get("target_boxes_list") or []

        possible_neighbors = [(curr_c + dc, curr_r + dr) for dc, dr in [(-1, 0), (1, 0), (0, -1), (0, 1)] 
                              if GRID_MIN_COL <= curr_c + dc <= GRID_MAX_COL and GRID_MIN_ROW <= curr_r + dr <= GRID_MAX_ROW]
        
        accessible_neighbors = [n for n in possible_neighbors if n in grid_status and grid_status[n]['state'] == 'accessible']
        if not accessible_neighbors: return py_trees.common.Status.FAILURE
        unvisited = [n for n in accessible_neighbors if not grid_status[n]['visited']]

        target_cell = None
        if self.mode == "EXPLORE":
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
            self.blackboard.set("target_cell", target_cell)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IncrementRealCountAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        val = self.blackboard.get("real_box_count") or 0
        self.blackboard.set("real_box_count", val + 1)
        return py_trees.common.Status.SUCCESS

class MarkAsVisitedAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        tc = self.blackboard.get("target_cell")
        gs = self.blackboard.get("grid_status")
        if tc and gs:
            gs[tc]['scanned'] = True
            gs[tc]['state']   = 'accessible'
            self.blackboard.set("grid_status", gs)
        return py_trees.common.Status.SUCCESS

class MarkAsObstacleAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        tc = self.blackboard.get("target_cell")
        gs = self.blackboard.get("grid_status")
        if tc and gs:
            gs[tc]['scanned'] = True
            gs[tc]['state']   = 'obstacle'
            self.blackboard.set("grid_status", gs)
        return py_trees.common.Status.SUCCESS

class IsAtExitCellCondition(py_trees.behaviour.Behaviour):
    EXIT_CELLS = [(1, 4), (3, 4)]

    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        return (py_trees.common.Status.SUCCESS
                if self.blackboard.get("robot_pos") in self.EXIT_CELLS
                else py_trees.common.Status.FAILURE)

class CheckBlackboardValue(py_trees.behaviour.Behaviour):
    def __init__(self, name, variable_name, expected_value, op=operator.eq):
        super().__init__(name)
        self.variable_name  = variable_name
        self.expected_value = expected_value
        self.op             = op
        self.blackboard     = py_trees.blackboard.Blackboard()

    def update(self):
        # Đã loại bỏ getattr nguy hiểm, thay bằng .get() an toàn
        val = self.blackboard.get(self.variable_name)
        if val is not None and self.op(val, self.expected_value):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class KeepRunningUntilSuccess(py_trees.decorators.Decorator):
    def __init__(self, name, child):
        super().__init__(name=name, child=child)

    def update(self):
        if getattr(self, 'decorated', None) is None:
            return py_trees.common.Status.FAILURE
        if self.decorated.status == py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    
    