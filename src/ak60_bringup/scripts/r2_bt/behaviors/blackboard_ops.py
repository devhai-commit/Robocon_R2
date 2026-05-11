#!/usr/bin/env python3
"""Helper behaviour: subscribe ROS topic và ghi vào py_trees blackboard.

Theo idiom Topics2BB của py_trees: mọi ROS subscribe gom vào nhánh
Topics2BB chạy song song với nhánh chính. Các behaviour ở nhánh chính
chỉ ĐỌC blackboard, không tự subscribe.

Set/Check blackboard dùng trực tiếp behaviour có sẵn của py_trees:
  - ``py_trees.behaviours.SetBlackboardVariable``
  - ``py_trees.behaviours.CheckBlackboardVariableValue``
  - ``py_trees.behaviours.WaitForBlackboardVariable``
"""

from __future__ import annotations

from typing import Any, Optional

import py_trees


class ROSTopicToBB(py_trees.behaviour.Behaviour):
    """Subscribe topic ROS và ghi vào py_trees blackboard.

    Args:
        ros_node:    rclpy Node để subscribe.
        topic_name:  ví dụ ``/gui_start_signal``.
        topic_type:  class message (``std_msgs.msg.String``, ...).
        key:         key trên blackboard sẽ được ghi.
        field:       tên field của message để ghi (vd ``data``).
                     Nếu ``None``, ghi nguyên message vào blackboard.
        qos_profile: QoSProfile hoặc int depth (default 10).

    Tick:
        - RUNNING khi key chưa có dữ liệu nào.
        - SUCCESS sau khi nhận được message đầu tiên.
    """

    def __init__(
        self,
        name: str,
        ros_node: Any,
        topic_name: str,
        topic_type: Any,
        key: str,
        field: Optional[str] = None,
        qos_profile: Any = 10,
    ) -> None:
        super().__init__(name)
        self.ros_node = ros_node
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.key = key
        self.field = field
        self.qos_profile = qos_profile
        self.subscription = None

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=self.key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs: Any) -> bool:
        self.subscription = self.ros_node.create_subscription(
            self.topic_type, self.topic_name, self._callback, self.qos_profile,
        )
        return True

    def _callback(self, msg: Any) -> None:
        value = getattr(msg, self.field) if self.field else msg
        self.blackboard.set(self.key, value)

    def update(self) -> py_trees.common.Status:
        if self.blackboard.exists(self.key):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        # Subscriber sống cùng ros_node; không destroy ở đây.
        pass


class BlackboardLoggerBehavior(py_trees.behaviour.Behaviour):
    """Log giá trị các biến blackboard mỗi tick.

    - Luôn log khi giá trị thay đổi (delta log).
    - Log tóm tắt định kỳ mỗi ``log_every_n`` tick.
    - Luôn trả về SUCCESS để không chặn cây.
    """

    _LOG_KEYS = (
        "real_box_count",
        "robot_pos",
        "visit_path",
        "grid_status",
        "target_cell",
        "just_climbed",
        "current_arm1_z",
        "priority_col",
        "target_boxes_list",
        "gui_start_raw",
    )

    def __init__(self, name: str, ros_node: Any, log_every_n: int = 30) -> None:
        super().__init__(name)
        self.ros_node = ros_node
        self.log_every_n = log_every_n
        self._tick_n = 0
        self._prev: dict = {}

        self.bb = self.attach_blackboard_client(name=name)
        for key in self._LOG_KEYS:
            self.bb.register_key(key=key, access=py_trees.common.Access.READ)

    def _fmt(self, key: str, value: Any) -> str:
        if key == "grid_status":
            visited = sum(1 for v in value.values() if v.get("visited"))
            scanned = sum(1 for v in value.values() if v.get("scanned"))
            return f"visited={visited}/12 scanned={scanned}/12"
        if key == "visit_path":
            tail = value[-3:] if len(value) >= 3 else value
            return f"len={len(value)} tail={tail}"
        if key == "target_boxes_list":
            return f"len={len(value)} {value}"
        return repr(value)

    def update(self) -> py_trees.common.Status:
        self._tick_n += 1
        logger = self.ros_node.get_logger()

        for key in self._LOG_KEYS:
            try:
                val = self.bb.get(key)
            except KeyError:
                continue
            if self._prev.get(key) != val:
                self._prev[key] = val
                logger.info(f"[BB CHANGED] {key}: {self._fmt(key, val)}")

        if self._tick_n % self.log_every_n == 0:
            parts = []
            for key in self._LOG_KEYS:
                try:
                    val = self.bb.get(key)
                    parts.append(f"{key}={self._fmt(key, val)}")
                except KeyError:
                    pass
            logger.info(f"[BB tick={self._tick_n}] " + " | ".join(parts))

        return py_trees.common.Status.SUCCESS


__all__ = ["ROSTopicToBB", "BlackboardLoggerBehavior"]
