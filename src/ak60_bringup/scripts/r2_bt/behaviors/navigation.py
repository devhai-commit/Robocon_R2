#!/usr/bin/env python3
import math
import os
import sys
import py_trees
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_transformations

from ak60_bringup.action import K230Align


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
        self.target_yaw = 0.0 if (curr_c, curr_r) in [(1, 4), (3, 4)] else default_yaw

        # Nếu vừa gắp và cần di chuyển ngang (trái/phải), lui 0.2m trước để tránh va vật
        needs_backup = (just_picked == "yes" and target_c != curr_c)
        self.phase = 0 if needs_backup else 1

    def update(self):
        # if self.phase == 0:
        #     return self._update_backup()
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


class K230AlignBehavior(py_trees.behaviour.Behaviour):
    """Gọi action server 'k230_align' để căn giữa robot theo K230.

    Server (k230_align_server.py) xử lý toàn bộ PID + yaw-lock + frame-counting.
    Behavior này chỉ là ActionClient — gửi goal, chờ kết quả.
    SUCCESS / FAILURE theo result.success từ server.
    """

    def __init__(self, name, ros_node):
        super().__init__(name)
        self.ros_node = ros_node
        self.client   = ActionClient(self.ros_node, K230Align, 'k230_align')
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent   = False

    def setup(self, **kwargs):
        return self.client.wait_for_server(timeout_sec=30.0)

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent   = False

    def update(self):
        if not self.goal_sent:
            self.goal_future = self.client.send_goal_async(K230Align.Goal())
            self.goal_sent   = True
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
                if self.result_future.result().result.success
                else py_trees.common.Status.FAILURE)

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None):
            self.goal_handle.cancel_goal_async()


class GoToEntranceBoxBehavior(py_trees.behaviour.Behaviour):
    """Di chuyển lần lượt qua tất cả entrance_boxes rồi ghi target_cell lên blackboard.

    Mỗi entrance_box (col, row) tra cứu (dx, dy) riêng trong box_coords.
    Sau khi đi hết queue, ghi target_cell = entrance_boxes[0] để hành vi tiếp theo
    biết cần vào ô nào. Box không có trong box_coords sẽ bị bỏ qua.
    Nếu entrance_boxes rỗng → ghi target_cell = default_target rồi SUCCESS ngay.

    Ví dụ:
        GoToEntranceBoxBehavior("Di_Den_Entrance_Box", ros_node, box_coords={
            (1, 1): (1.6, -2.2),
            (2, 1): (1.6, -2.6),
            (3, 1): (1.6, -3.0),
        }, default_target=(2, 1))
    """

    def __init__(self, name, ros_node, box_coords: dict, default_target=(2, 1), odom_topic='/odometry/filtered'):
        super().__init__(name)
        self.ros_node = ros_node
        self.box_coords = {tuple(k): tuple(v) for k, v in box_coords.items()}
        self.default_target = tuple(default_target)
        self.client = ActionClient(self.ros_node, NavigateToPose, 'navigate_to_pose')
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False
        self.current_pose = None
        self.current_idx = 0
        self.move_queue = []
        self.target_cell_value = self.default_target
        self.odom_sub = self.ros_node.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="entrance_boxes", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_cell", access=py_trees.common.Access.WRITE)

    def _odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def euler_to_quaternion(self, yaw):
        return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]

    def setup(self, **kwargs):
        return self.client.wait_for_server(timeout_sec=5.0)

    def initialise(self):
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False
        self.current_idx = 0
        boxes = list(self.blackboard.entrance_boxes) if self.blackboard.exists("entrance_boxes") else []
        matched = [tuple(box) for box in boxes if tuple(box) in self.box_coords]
        self.move_queue = [self.box_coords[box] for box in matched]
        self.target_cell_value = (2, 1)  

    def _send_goal(self, dx, dy):
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        curr_yaw = self.get_yaw(self.current_pose.orientation)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.header.stamp = self.ros_node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = curr_x + dx * math.cos(curr_yaw) - dy * math.sin(curr_yaw)
        goal_msg.pose.pose.position.y = curr_y + dx * math.sin(curr_yaw) + dy * math.cos(curr_yaw)
        q = self.euler_to_quaternion(0.0)
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        self.goal_future = self.client.send_goal_async(goal_msg)
        self.goal_sent = True

    def update(self):
        if self.current_idx >= len(self.move_queue):
            self.blackboard.target_cell = self.target_cell_value
            return py_trees.common.Status.SUCCESS

        dx, dy = self.move_queue[self.current_idx]

        if not self.goal_sent:
            if self.current_pose is None:
                return py_trees.common.Status.RUNNING
            self._send_goal(dx, dy)
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

        self.current_idx += 1
        self.goal_future = self.result_future = self.goal_handle = None
        self.goal_sent = False
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and getattr(self, 'goal_handle', None):
            self.goal_handle.cancel_goal_async()



class ResetPoseBehavior(py_trees.behaviour.Behaviour):
    """
    Node Cây Hành Vi: Ép bộ lọc Odom/EKF nhận tọa độ (X, Y, Yaw) mới.
    """
    def __init__(self, name, ros_node, x=0.0, y=0.0, yaw_deg=0.0, topic_name='/set_pose', frame_id='odom'):
        super().__init__(name)
        self.ros_node = ros_node
        self.target_x = float(x)
        self.target_y = float(y)
        self.target_yaw_rad = math.radians(float(yaw_deg))
        
        self.topic_name = topic_name
        self.frame_id = frame_id
        self.publisher = None

    def setup(self, **kwargs):
        # Khởi tạo Publisher khi Cây Hành Vi được setup
        self.publisher = self.ros_node.create_publisher(
            PoseWithCovarianceStamped, 
            self.topic_name, 
            10
        )
        return True

    def update(self):
        # Khởi tạo bản tin Pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # 1. Thiết lập tọa độ X, Y
        msg.pose.pose.position.x = self.target_x
        msg.pose.pose.position.y = self.target_y
        msg.pose.pose.position.z = 0.0

        # 2. Chuyển đổi góc Yaw (Độ -> Radian -> Quaternion)
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.target_yaw_rad)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # 3. [QUAN TRỌNG] Đặt ma trận hiệp phương sai (Covariance) cực nhỏ
        # Điều này "ra lệnh" cho EKF tin tưởng 100% vào tọa độ này và loại bỏ dữ liệu cũ.
        msg.pose.covariance[0] = 1e-9   # Độ tin cậy trục X
        msg.pose.covariance[7] = 1e-9   # Độ tin cậy trục Y
        msg.pose.covariance[35] = 1e-9  # Độ tin cậy góc Yaw (Z)

        # 4. Gửi bản tin
        self.publisher.publish(msg)
        
        self.ros_node.get_logger().info(
            f"[{self.name}] 📍 Đã Reset Odom -> X: {self.target_x}m, Y: {self.target_y}m, Yaw: {math.degrees(self.target_yaw_rad):.1f}°"
        )

        # Vì việc publish diễn ra ngay lập tức, node này luôn trả về SUCCESS
        return py_trees.common.Status.SUCCESS