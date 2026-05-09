#!/usr/bin/env python3
"""BT behaviours dùng custom blackboard (``r2_bt.blackboard.bb``).

Thay thế các behaviour gốc của py_trees / py_trees_ros mà mình
không muốn dùng nữa:

  - ``SetBB``         thay  py_trees.behaviours.SetBlackboardVariable
  - ``CheckBB``       thay  CheckBlackboardValue (giữ tên cũ, alias bên dưới)
  - ``ROSTopicToBB``  thay  py_trees_ros.subscribers.ToBlackboard
"""

from __future__ import annotations

import operator
from typing import Any, Callable, Optional

import py_trees

from r2_bt.blackboard import bb


class SetBB(py_trees.behaviour.Behaviour):
    """Set ``bb[key] = value`` rồi return SUCCESS.

    overwrite=False → chỉ set nếu key chưa tồn tại.
    """

    def __init__(self, name: str, key: str, value: Any, overwrite: bool = True) -> None:
        super().__init__(name)
        self.key = key
        self.value = value
        self.overwrite = overwrite

    def update(self) -> py_trees.common.Status:
        if self.overwrite or self.key not in bb:
            bb.set(self.key, self.value)
        return py_trees.common.Status.SUCCESS


class CheckBB(py_trees.behaviour.Behaviour):
    """SUCCESS nếu ``op(bb[key], expected)`` đúng, FAILURE nếu sai/key chưa set."""

    def __init__(
        self,
        name: str,
        key: str,
        expected: Any,
        op: Callable[[Any, Any], bool] = operator.eq,
    ) -> None:
        super().__init__(name)
        self.key = key
        self.expected = expected
        self.op = op

    def update(self) -> py_trees.common.Status:
        val = bb.get(self.key)
        if val is not None and self.op(val, self.expected):
            return py_trees.common.Status.SUCCESS
        self.logger.debug(
            f"[{self.name}] FAIL: bb[{self.key!r}]={val!r} vs {self.expected!r}"
        )
        return py_trees.common.Status.FAILURE


class ROSTopicToBB(py_trees.behaviour.Behaviour):
    """Subscribe topic ROS và ghi vào blackboard.

    Args:
        ros_node:      rclpy Node để subscribe.
        topic_name:    ví dụ ``"/gui_start_signal"``.
        topic_type:    class message (``std_msgs.msg.String``, …).
        key:           key trên blackboard sẽ được ghi.
        field:         tên field của message để ghi (vd ``"data"``).
                       Nếu ``None``, ghi nguyên message vào blackboard.
        qos_profile:   QoSProfile hoặc int depth (default 10).

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

    def setup(self, **kwargs: Any) -> bool:
        self.subscription = self.ros_node.create_subscription(
            self.topic_type, self.topic_name, self._callback, self.qos_profile,
        )
        return True

    def _callback(self, msg: Any) -> None:
        bb.set(self.key, getattr(msg, self.field) if self.field else msg)

    def update(self) -> py_trees.common.Status:
        return (
            py_trees.common.Status.SUCCESS
            if self.key in bb
            else py_trees.common.Status.RUNNING
        )

    def terminate(self, new_status: py_trees.common.Status) -> None:
        # Subscriber sống cùng ros_node; không destroy ở đây.
        pass


# Alias để code cũ import "CheckBlackboardValue" vẫn chạy.
CheckBlackboardValue = CheckBB


__all__ = ["SetBB", "CheckBB", "CheckBlackboardValue", "ROSTopicToBB"]
