#!/usr/bin/env python3
import json
import py_trees

from r2_bt.blackboard import bb
from r2_bt.behaviors.blackboard_ops import CheckBB, CheckBlackboardValue  # noqa: F401  (re-export)
from r2_bt.config import (
    GRID_MIN_COL, GRID_MAX_COL, GRID_MIN_ROW, GRID_MAX_ROW, manhattan_distance,
)


class WaitForStartSignalBehavior(py_trees.behaviour.Behaviour):
    """Đứng chờ tín hiệu JSON từ GUI.

    Đọc ``bb.gui_start_raw`` (do ``ROSTopicToBB`` ghi từ topic /gui_start_signal),
    parse JSON, ghi ``priority_col`` và ``target_boxes_list`` vào blackboard.
    """

    def __init__(self, name, ros_node):
        super().__init__(name)
        self.ros_node = ros_node

    def update(self):
        raw = bb.gui_start_raw
        if raw is None:
            self.ros_node.get_logger().info(
                f"[{self.name}] 🟡 Đang chờ ấn nút START trên màn hình...",
                throttle_duration_sec=2.0,
            )
            return py_trees.common.Status.RUNNING

        try:
            data = json.loads(raw)
        except (ValueError, TypeError) as e:
            self.ros_node.get_logger().error(f"Lỗi giải mã GUI: {e}")
            return py_trees.common.Status.RUNNING

        if data.get("command") != "START":
            return py_trees.common.Status.RUNNING

        bb.priority_col = data.get("priority_col", 2)
        bb.target_boxes_list = [tuple(box) for box in data.get("target_boxes", [])]
        return py_trees.common.Status.SUCCESS


class CalculatePickElevationAction(py_trees.behaviour.Behaviour):
    """Tính toán cao độ Arm 1 dựa trên chênh lệch bậc địa hình."""

    def __init__(self, name, ros_node):
        super().__init__(name)
        self.ros_node = ros_node
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
        curr_c, curr_r = bb.get("robot_pos", (2, 0))
        target_c, target_r = bb.get("target_cell", (2, 1))

        h_current = self.elevation_map.get((curr_c, curr_r), self.default_elevation)
        h_target = self.elevation_map.get((target_c, target_r), self.default_elevation)

        if h_target < h_current:
            bb.current_arm1_z = 0.0
        elif h_target > h_current:
            bb.current_arm1_z = 325.0
        else:
            bb.current_arm1_z = 125.0

        self.ros_node.get_logger().info(
            f"[{self.name}] 🤖 Arm 1 Z = {bb.current_arm1_z}mm"
        )
        return py_trees.common.Status.SUCCESS


class DecideNextGridCellAction(py_trees.behaviour.Behaviour):
    EXIT_CELLS = [(1, 4), (3, 4)]

    def __init__(self, name, ros_node, mode):
        super().__init__(name)
        self.ros_node, self.mode = ros_node, mode

    def update(self):
        curr_c, curr_r = bb.get("robot_pos", (2, 0))
        grid_status = bb.get("grid_status", {}) or {}
        visit_path = bb.get("visit_path", []) or []
        p_col = bb.get("priority_col", 2)
        t_boxes = bb.get("target_boxes_list", []) or []

        possible_neighbors = [
            (curr_c + dc, curr_r + dr)
            for dc, dr in [(-1, 0), (1, 0), (0, -1), (0, 1)]
            if GRID_MIN_COL <= curr_c + dc <= GRID_MAX_COL
            and GRID_MIN_ROW <= curr_r + dr <= GRID_MAX_ROW
        ]

        accessible_neighbors = [
            n for n in possible_neighbors
            if n in grid_status and grid_status[n]['state'] == 'accessible'
        ]
        if not accessible_neighbors:
            return py_trees.common.Status.FAILURE
        unvisited = [n for n in accessible_neighbors if not grid_status[n]['visited']]

        target_cell = None
        if self.mode == "EXPLORE":
            target_neighbors = [n for n in unvisited if n in t_boxes]
            if target_neighbors:
                target_cell = sorted(target_neighbors, key=lambda n: (n[1], n[0]), reverse=True)[0]
            elif any(n[0] == p_col for n in unvisited):
                target_cell = sorted(
                    [n for n in unvisited if n[0] == p_col],
                    key=lambda n: n[1], reverse=True,
                )[0]
            elif unvisited:
                target_cell = sorted(unvisited, key=lambda n: (n[1], n[0]), reverse=True)[0]
            else:
                target_cell = visit_path[-2] if len(visit_path) >= 2 else visit_path[0]
        else:
            closest_cell, min_dist = None, float('inf')
            for cell in accessible_neighbors:
                dist = min(
                    manhattan_distance(cell[0], cell[1], e[0], e[1])
                    for e in self.EXIT_CELLS
                )
                if dist < min_dist:
                    min_dist, closest_cell = dist, cell
            target_cell = closest_cell

        if target_cell:
            bb.target_cell = target_cell
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IncrementRealCountAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        bb.real_box_count = (bb.real_box_count or 0) + 1
        return py_trees.common.Status.SUCCESS


class MarkAsVisitedAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        tc = bb.target_cell
        gs = bb.grid_status
        if tc and gs:
            gs[tc]['scanned'] = True
            gs[tc]['state'] = 'accessible'
            gs[tc]['visited'] = True
            bb.grid_status = gs
        return py_trees.common.Status.SUCCESS


class MarkAsObstacleAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        tc = bb.target_cell
        gs = bb.grid_status
        if tc and gs:
            gs[tc]['scanned'] = True
            gs[tc]['state'] = 'obstacle'
            bb.grid_status = gs
        return py_trees.common.Status.SUCCESS


class IsAtExitCellCondition(py_trees.behaviour.Behaviour):
    EXIT_CELLS = [(1, 4), (3, 4)]

    def __init__(self, name):
        super().__init__(name)

    def update(self):
        return (
            py_trees.common.Status.SUCCESS
            if bb.robot_pos in self.EXIT_CELLS
            else py_trees.common.Status.FAILURE
        )


class KeepRunningUntilSuccess(py_trees.decorators.Decorator):
    def __init__(self, name, child):
        super().__init__(name=name, child=child)

    def update(self):
        if getattr(self, 'decorated', None) is None:
            return py_trees.common.Status.FAILURE
        if self.decorated.status == py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
