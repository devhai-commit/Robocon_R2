#!/usr/bin/env python3
import py_trees
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from ak60_bringup.action import ClimbStep

from r2_bt.blackboard import bb


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
        self.ros_node.get_logger().info(f"[{self.name}] Đang chạy ClimbStep V={self.velocity}m/s")
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
        return (py_trees.common.Status.SUCCESS
                if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED
                else py_trees.common.Status.FAILURE)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None):
            self.goal_handle.cancel_goal_async()


class IsClimbingUpCondition(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node):
        super().__init__(name)
        self.ros_node = ros_node
        self.elevation_map = {}
        self.default_elevation = 0.0

    def setup(self, **kwargs):
        cols    = self.ros_node.get_parameter('map_cols').value
        rows    = self.ros_node.get_parameter('map_rows').value
        heights = self.ros_node.get_parameter('map_heights').value
        self.default_elevation = self.ros_node.get_parameter('default_elevation').value
        self.elevation_map = {(c, r): float(h) for c, r, h in zip(cols, rows, heights)}
        return True

    def update(self):
        curr_c, curr_r = bb.get("robot_pos", (2, 0))
        target_c, target_r = bb.get("target_cell", (2, 1))

        h_curr   = self.elevation_map.get((curr_c,   curr_r),   self.default_elevation)
        h_target = self.elevation_map.get((target_c, target_r), self.default_elevation)
        return (py_trees.common.Status.SUCCESS
                if h_target > h_curr
                else py_trees.common.Status.FAILURE)


class HasElevationChangeCondition(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node):
        super().__init__(name)
        self.ros_node = ros_node
        self.elevation_map = {}
        self.default_elevation = 0.0

    def setup(self, **kwargs):
        cols    = self.ros_node.get_parameter('map_cols').value
        rows    = self.ros_node.get_parameter('map_rows').value
        heights = self.ros_node.get_parameter('map_heights').value
        self.default_elevation = self.ros_node.get_parameter('default_elevation').value
        self.elevation_map = {(c, r): float(h) for c, r, h in zip(cols, rows, heights)}
        return True

    def update(self):
        curr_c, curr_r = bb.get("robot_pos", (2, 0))
        target_c, target_r = bb.get("target_cell", (2, 1))

        h_curr   = self.elevation_map.get((curr_c,   curr_r),   self.default_elevation)
        h_target = self.elevation_map.get((target_c, target_r), self.default_elevation)
        return (py_trees.common.Status.SUCCESS
                if h_target != h_curr
                else py_trees.common.Status.FAILURE)


class DynamicClimbStepBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node):
        super().__init__(name)
        self.ros_node = ros_node
        self.client = ActionClient(self.ros_node, ClimbStep, 'climb_step')
        self.goal_future = self.result_future = self.goal_handle = None
        self.needs_climb = False
        self.elevation_map = {}
        self.default_elevation = 0.0

    def setup(self, **kwargs):
        cols    = self.ros_node.get_parameter('map_cols').value
        rows    = self.ros_node.get_parameter('map_rows').value
        heights = self.ros_node.get_parameter('map_heights').value
        self.default_elevation = self.ros_node.get_parameter('default_elevation').value
        self.elevation_map = {(c, r): float(h) for c, r, h in zip(cols, rows, heights)}
        return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        curr_c, curr_r = bb.get("robot_pos", (2, 0))
        target_c, target_r = bb.get("target_cell", (2, 1))

        h_curr   = self.elevation_map.get((curr_c,   curr_r),   self.default_elevation)
        h_target = self.elevation_map.get((target_c, target_r), self.default_elevation)
        delta_h  = h_target - h_curr

        if delta_h == 0:
            self.needs_climb = False
            bb.just_climbed = False
            return

        self.needs_climb = True
        goal_msg = ClimbStep.Goal()
        goal_msg.velocity = 0.3 if delta_h > 0 else 0.1
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
            bb.just_climbed = True
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if (new_status == py_trees.common.Status.INVALID
                and self.needs_climb
                and getattr(self, 'goal_handle', None)):
            self.goal_handle.cancel_goal_async()
