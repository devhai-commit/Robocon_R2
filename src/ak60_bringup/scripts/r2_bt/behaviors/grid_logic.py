#!/usr/bin/env python3
import json
import py_trees

from r2_bt.config import (
    GRID_MIN_COL, GRID_MAX_COL, GRID_MIN_ROW, GRID_MAX_ROW, manhattan_distance,
)


_EXIT_CELLS = [(1, 4), (3, 4)]


class WaitForStartSignalBehavior(py_trees.behaviour.Behaviour):
    """Đứng chờ tín hiệu JSON từ GUI.

    Đọc ``gui_start_raw`` (do ``ROSTopicToBB`` ghi từ topic /gui_start_signal),
    parse JSON, ghi ``priority_col`` và ``target_boxes_list`` vào blackboard.
    """

    def __init__(self, name, ros_node):
        super().__init__(name)
        self.ros_node = ros_node
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="gui_start_raw", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="priority_col", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_boxes_list", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="entrance_boxes", access=py_trees.common.Access.WRITE)

    def update(self):
        if not self.blackboard.exists("gui_start_raw"):
            self.ros_node.get_logger().info(
                f"[{self.name}] 🟡 Đang chờ ấn nút START trên màn hình...",
                throttle_duration_sec=2.0,
            )
            return py_trees.common.Status.RUNNING

        raw = self.blackboard.gui_start_raw
        try:
            data = json.loads(raw)
        except (ValueError, TypeError) as e:
            self.ros_node.get_logger().error(f"Lỗi giải mã GUI: {e}")
            return py_trees.common.Status.RUNNING

        if data.get("command") != "START":
            return py_trees.common.Status.RUNNING

        self.blackboard.priority_col = data.get("priority_col", 2)
        self.blackboard.target_boxes_list = [tuple(box) for box in data.get("target_boxes", [])]
        self.blackboard.entrance_boxes = [tuple(box) for box in data.get("entrance_boxes", [])]
        return py_trees.common.Status.SUCCESS


class CalculatePickElevationAction(py_trees.behaviour.Behaviour):
    """Tính toán cao độ Arm 1 dựa trên chênh lệch bậc địa hình."""

    def __init__(self, name, ros_node):
        super().__init__(name)
        self.ros_node = ros_node
        self.elevation_map = {}
        self.default_elevation = 400.0
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="robot_pos", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_cell", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_arm1_z", access=py_trees.common.Access.WRITE)

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

        if h_target < h_current:
            self.blackboard.current_arm1_z = 0.0
        elif h_target > h_current:
            self.blackboard.current_arm1_z = 325.0
        else:
            self.blackboard.current_arm1_z = 125.0

        self.ros_node.get_logger().info(
            f"[{self.name}] 🤖 Arm 1 Z = {self.blackboard.current_arm1_z}mm"
        )
        return py_trees.common.Status.SUCCESS


class DecideNextGridCellAction(py_trees.behaviour.Behaviour):
    EXIT_CELLS = _EXIT_CELLS

    def __init__(self, name, ros_node, mode):
        super().__init__(name)
        self.ros_node, self.mode = ros_node, mode
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="robot_pos", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="grid_status", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="visit_path", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="priority_col", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_boxes_list", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_cell", access=py_trees.common.Access.WRITE)

    def _drop_completed_target(self, target_boxes, grid_status, curr_pos):
        if not target_boxes:
            return target_boxes
        front = target_boxes[0]
        cell_state = grid_status.get(front, {})
        if cell_state.get('state') == 'obstacle':
            return target_boxes[1:]
        if front == curr_pos and (cell_state.get('visited') or cell_state.get('scanned')):
            return target_boxes[1:]
        return target_boxes

    def _choose_step_toward(self, accessible_neighbors, target_cell):
        if not target_cell:
            return None
        return min(
            accessible_neighbors,
            key=lambda cell: (
                manhattan_distance(cell[0], cell[1], target_cell[0], target_cell[1]),
                cell[1],
                cell[0],
            ),
        )

    def update(self):
        curr_c, curr_r = self.blackboard.robot_pos
        grid_status = self.blackboard.grid_status or {}
        visit_path = self.blackboard.visit_path or []
        p_col = self.blackboard.priority_col if self.blackboard.exists("priority_col") else 2
        t_boxes = self.blackboard.target_boxes_list if self.blackboard.exists("target_boxes_list") else []

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

        if t_boxes:
            updated_boxes = self._drop_completed_target(t_boxes, grid_status, (curr_c, curr_r))
            if updated_boxes != t_boxes:
                self.blackboard.target_boxes_list = list(updated_boxes)
                t_boxes = updated_boxes

        target_cell = None
        if self.mode == "EXPLORE":
            if t_boxes:
                target_cell = self._choose_step_toward(accessible_neighbors, t_boxes[0])
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
            target_cell = min(
                accessible_neighbors,
                key=lambda cell: (
                    min(manhattan_distance(cell[0], cell[1], e[0], e[1]) for e in self.EXIT_CELLS),
                    -cell[1],
                    -cell[0],
                ),
            )

        if target_cell:
            self.blackboard.target_cell = target_cell
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IncrementRealCountAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        # WRITE access cho phép cả đọc lẫn ghi.
        self.blackboard.register_key(key="real_box_count", access=py_trees.common.Access.WRITE)

    def update(self):
        current = self.blackboard.real_box_count if self.blackboard.exists("real_box_count") else 0
        self.blackboard.real_box_count = (current or 0) + 1
        return py_trees.common.Status.SUCCESS


class MarkAsVisitedAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="target_cell", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="grid_status", access=py_trees.common.Access.WRITE)

    def update(self):
        if not (self.blackboard.exists("target_cell") and self.blackboard.exists("grid_status")):
            return py_trees.common.Status.SUCCESS
        tc = self.blackboard.target_cell
        gs = self.blackboard.grid_status
        if tc and gs:
            gs[tc]['scanned'] = True
            gs[tc]['state'] = 'accessible'
            gs[tc]['visited'] = True
            self.blackboard.grid_status = gs
        return py_trees.common.Status.SUCCESS


class MarkAsObstacleAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="target_cell", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="grid_status", access=py_trees.common.Access.WRITE)

    def update(self):
        if not (self.blackboard.exists("target_cell") and self.blackboard.exists("grid_status")):
            return py_trees.common.Status.SUCCESS
        tc = self.blackboard.target_cell
        gs = self.blackboard.grid_status
        if tc and gs:
            gs[tc]['scanned'] = True
            gs[tc]['state'] = 'obstacle'
            self.blackboard.grid_status = gs
        return py_trees.common.Status.SUCCESS


class IsAtExitCellCondition(py_trees.behaviour.Behaviour):
    EXIT_CELLS = _EXIT_CELLS

    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="robot_pos", access=py_trees.common.Access.READ)

    def update(self):
        if not self.blackboard.exists("robot_pos"):
            return py_trees.common.Status.FAILURE
        return (
            py_trees.common.Status.SUCCESS
            if self.blackboard.robot_pos in self.EXIT_CELLS
            else py_trees.common.Status.FAILURE
        )


class KeepRunningUntilSuccess(py_trees.decorators.Decorator):
    def __init__(self, name, child):
        super().__init__(name=name, child=child)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="robot_pos", access=py_trees.common.Access.READ)

    def update(self):
        if getattr(self, 'decorated', None) is None:
            return py_trees.common.Status.FAILURE
        if self.decorated.status == py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.SUCCESS
        if (self.blackboard.exists("robot_pos")
                and self.blackboard.robot_pos in _EXIT_CELLS):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
