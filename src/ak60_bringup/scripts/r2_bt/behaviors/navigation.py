#!/usr/bin/env python3
import math
import py_trees
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus

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

    def euler_to_quaternion(self, yaw): return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]

    def setup(self, **kwargs): return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False

    def update(self):
        if not self.goal_sent:
            if self.current_pose is None:
                return py_trees.common.Status.RUNNING
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
        if self.result_future is None: 
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done(): return py_trees.common.Status.RUNNING
        return (py_trees.common.Status.SUCCESS
                if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED
                else py_trees.common.Status.FAILURE)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None):
            self.goal_handle.cancel_goal_async()

class TurnToTargetCellBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, odom_topic='/odometry/filtered'):
        super().__init__(name)
        self.ros_node = ros_node
        self.client = ActionClient(self.ros_node, NavigateToPose, 'navigate_to_pose')
        self.blackboard = py_trees.blackboard.Blackboard()
        self.current_pose = None
        self.goal_sent = False
        self.target_yaw = None
        self.odom_sub = self.ros_node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg): self.current_pose = msg.pose.pose

    def euler_to_quaternion(self, yaw): return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]

    def setup(self, **kwargs): return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False
        
        rp = self.blackboard.get("robot_pos")
        tc = self.blackboard.get("target_cell")
        if not rp or not tc:
            self.target_yaw = None
            return
            
        curr_c, curr_r = rp
        target_c, target_r = tc
        
        if target_c > curr_c:   default_yaw = 0.0
        elif target_c < curr_c: default_yaw = 180.0
        elif target_r > curr_r: default_yaw = 90.0
        elif target_r < curr_r: default_yaw = -90.0
        else:                   default_yaw = 0.0
        self.target_yaw = 0.0 if (target_c, target_r) in [(2, 1), (1, 4), (3, 4)] else default_yaw

    def update(self):
        if self.target_yaw is None: return py_trees.common.Status.SUCCESS
        if not self.goal_sent:
            if self.current_pose is None: return py_trees.common.Status.RUNNING
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'odom'
            goal_msg.pose.header.stamp = self.ros_node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = self.current_pose.position.x
            goal_msg.pose.pose.position.y = self.current_pose.position.y
            q = self.euler_to_quaternion(math.radians(self.target_yaw))
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
        return (py_trees.common.Status.SUCCESS
                if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED
                else py_trees.common.Status.FAILURE)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None):
            self.goal_handle.cancel_goal_async()

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

    def euler_to_quaternion(self, yaw): return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]

    def setup(self, **kwargs): return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False

    def update(self):
        if not self.goal_sent:
            if self.current_pose is None: return py_trees.common.Status.RUNNING
            
            rp = self.blackboard.get("robot_pos")
            tc = self.blackboard.get("target_cell")
            curr_c, curr_r = rp if rp else (2, 0)
            target_c, target_r = tc if tc else (2, 1)
            
            curr_x = self.current_pose.position.x
            curr_y = self.current_pose.position.y
            curr_yaw = self.get_yaw(self.current_pose.orientation)
            dc = target_c - curr_c
            dr = target_r - curr_r
            if dc == 0 and dr == 0: return py_trees.common.Status.SUCCESS
            
            path_yaw = math.atan2(dr, dc)
            just_climbed = self.blackboard.get("just_climbed")
            dist = self.climb_dist if just_climbed else self.flat_dist
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'odom'
            goal_msg.pose.header.stamp = self.ros_node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = curr_x + dist * math.cos(path_yaw)
            goal_msg.pose.pose.position.y = curr_y + dist * math.sin(path_yaw)
            q = self.euler_to_quaternion(curr_yaw)
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
            tc = self.blackboard.get("target_cell")
            self.blackboard.set("robot_pos", tc)
            
            vp = self.blackboard.get("visit_path") or []
            if vp and len(vp) >= 2 and vp[-2] == tc:
                vp.pop()
            else:
                vp.append(tc)
            self.blackboard.set("visit_path", vp)
            self.blackboard.set("just_climbed", False)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None):
            self.goal_handle.cancel_goal_async()

            