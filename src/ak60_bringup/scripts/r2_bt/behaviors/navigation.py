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
        self.current_pose = None
        self.goal_sent = False
        self.target_yaw = None
        self.phase = 1  # 0=backup, 1=turn
        self.odom_sub = self.ros_node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="robot_pos", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_cell", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="just_picked", access=py_trees.common.Access.READ)

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

        just_picked = self.blackboard.just_picked if self.blackboard.exists("just_picked") else "no"

        if not (self.blackboard.exists("robot_pos") and self.blackboard.exists("target_cell")):
            self.target_yaw = None
            self.phase = 1
            return

        rp = self.blackboard.robot_pos
        tc = self.blackboard.target_cell
        if not rp or not tc:
            self.target_yaw = None
            self.phase = 1
            return

        curr_c, curr_r = rp
        target_c, target_r = tc

        if target_c > curr_c:   default_yaw = 90.0
        elif target_c < curr_c: default_yaw = -90.0
        elif target_r > curr_r: default_yaw = 0.0
        elif target_r < curr_r: default_yaw = 180.0
        else:                   default_yaw = 0.0
        self.target_yaw = 0.0 if (target_c, target_r) in [(2, 1), (1, 4), (3, 4)] else default_yaw

        # Nếu vừa gắp và cần di chuyển ngang (trái/phải), lui 0.2m trước để tránh va vật
        needs_backup = (just_picked == "yes" and target_c != curr_c)
        self.phase = 0 if needs_backup else 1

    def update(self):
        if self.phase == 0:
            return self._update_backup()
        return self._update_turn()

    def _update_backup(self):
        if not self.goal_sent:
            if self.current_pose is None:
                return py_trees.common.Status.RUNNING
            curr_x = self.current_pose.position.x
            curr_y = self.current_pose.position.y
            curr_yaw = self.get_yaw(self.current_pose.orientation)
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'odom'
            goal_msg.pose.header.stamp = self.ros_node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = curr_x + (-0.2) * math.cos(curr_yaw)
            goal_msg.pose.pose.position.y = curr_y + (-0.2) * math.sin(curr_yaw)
            q = self.euler_to_quaternion(curr_yaw)
            goal_msg.pose.pose.orientation.z = q[2]
            goal_msg.pose.pose.orientation.w = q[3]
            self.goal_future = self.client.send_goal_async(goal_msg)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING
        if self.goal_future is None or not self.goal_future.done():
            return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted:
                return py_trees.common.Status.FAILURE
        if self.result_future is None:
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done():
            return py_trees.common.Status.RUNNING
        if self.result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
            return py_trees.common.Status.FAILURE
        # Backup xong → chuyển sang phase xoay
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False
        self.phase = 1
        return py_trees.common.Status.RUNNING

    def _update_turn(self):
        if self.target_yaw is None:
            return py_trees.common.Status.SUCCESS
        if not self.goal_sent:
            if self.current_pose is None:
                return py_trees.common.Status.RUNNING
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
        if self.goal_future is None or not self.goal_future.done():
            return py_trees.common.Status.RUNNING
        if self.goal_handle is None:
            self.goal_handle = self.goal_future.result()
            if not self.goal_handle.accepted:
                return py_trees.common.Status.FAILURE
        if self.result_future is None:
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done():
            return py_trees.common.Status.RUNNING
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
        self.current_pose = None
        self.goal_sent = False
        self.odom_sub = self.ros_node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="robot_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_cell", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="just_climbed", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="visit_path", access=py_trees.common.Access.WRITE)

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

            if not (self.blackboard.exists("robot_pos") and self.blackboard.exists("target_cell")):
                return py_trees.common.Status.FAILURE

            curr_c, curr_r = self.blackboard.robot_pos
            target_c, target_r = self.blackboard.target_cell

            curr_x = self.current_pose.position.x
            curr_y = self.current_pose.position.y
            curr_yaw = self.get_yaw(self.current_pose.orientation)

            dc = target_c - curr_c
            dr = target_r - curr_r
            if dc == 0 and dr == 0: return py_trees.common.Status.SUCCESS

            just_climbed = self.blackboard.just_climbed if self.blackboard.exists("just_climbed") else False
            dist = self.climb_dist if just_climbed else self.flat_dist

            # [FIX LỖI ĐI NGANG]: Sử dụng trực tiếp curr_yaw thay vì tính path_yaw từ sa bàn
            # Điều này đảm bảo xe luôn tiến thẳng tới trước mặt (relative X) bất kể sa bàn bị lệch
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'odom'
            goal_msg.pose.header.stamp = self.ros_node.get_clock().now().to_msg()

            goal_msg.pose.pose.position.x = curr_x + dist * math.cos(curr_yaw)
            goal_msg.pose.pose.position.y = curr_y + dist * math.sin(curr_yaw)

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
            tc = self.blackboard.target_cell
            self.blackboard.robot_pos = tc

            vp = self.blackboard.visit_path if self.blackboard.exists("visit_path") else []
            vp = list(vp) if vp else []
            if vp and len(vp) >= 2 and vp[-2] == tc:
                vp.pop()
            else:
                vp.append(tc)
            self.blackboard.visit_path = vp
            self.blackboard.just_climbed = False
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None):
            self.goal_handle.cancel_goal_async()
