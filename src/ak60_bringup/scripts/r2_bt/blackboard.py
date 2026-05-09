#!/usr/bin/env python3
"""
Custom blackboard cho r2_bt — singleton, không cần register key, đọc luôn an toàn.

Khác py_trees blackboard:
  - KHÔNG cần ``attach_blackboard_client`` / ``register_key``.
  - Đọc key chưa set trả về ``None`` (hoặc default), KHÔNG raise ``KeyError``.
  - Truy cập qua attribute (``bb.robot_pos``) hoặc subscript (``bb["robot_pos"]``).
  - Thread-safe (RLock).
  - Có watcher: ``bb.watch("key", callback)`` gọi callback khi key đổi giá trị.

Ví dụ:
    from r2_bt.blackboard import bb

    # Ghi
    bb.robot_pos = (2, 0)
    bb["target_cell"] = (2, 1)
    bb.set("real_box_count", 0)
    bb.update({"priority_col": 2, "visit_path": [(2, 0)]})

    # Đọc — không bao giờ raise
    pos = bb.robot_pos                    # None nếu chưa set
    pos = bb.get("robot_pos", (2, 0))     # default fallback
    if "grid_status" in bb: ...

    # Inspect
    print(bb.dump())                      # snapshot dict

    # Watcher (ví dụ: log mỗi khi robot di chuyển)
    bb.watch("robot_pos", lambda old, new: print(f"{old} → {new}"))
"""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Iterator, List, Tuple


# Tên các thuộc tính nội bộ — không lưu vào storage khi gán.
_RESERVED = frozenset({"_storage", "_lock", "_watchers"})


class _Sentinel:
    """Sentinel cho phân biệt 'chưa có key' vs 'có key với giá trị None'."""
    __slots__ = ()
    def __repr__(self) -> str: return "<MISSING>"


_MISSING = _Sentinel()

WatcherFn = Callable[[Any, Any], None]


class _Blackboard:
    """Storage key-value singleton dùng cho behavior tree."""

    def __init__(self) -> None:
        object.__setattr__(self, "_storage", {})
        object.__setattr__(self, "_lock", threading.RLock())
        object.__setattr__(self, "_watchers", {})

    # -------------------- READ --------------------

    def get(self, key: str, default: Any = None) -> Any:
        with self._lock:
            return self._storage.get(key, default)

    def __getattr__(self, key: str) -> Any:
        # __getattr__ chỉ được gọi khi attribute lookup thường thất bại.
        # Tránh đụng các dunder name (pickle, copy, ...).
        if key.startswith("_"):
            raise AttributeError(key)
        return self._storage.get(key, None)

    def __getitem__(self, key: str) -> Any:
        return self._storage.get(key, None)

    def __contains__(self, key: str) -> bool:
        return key in self._storage

    def has(self, key: str) -> bool:
        return key in self._storage

    # -------------------- WRITE --------------------

    def set(self, key: str, value: Any) -> None:
        with self._lock:
            old = self._storage.get(key, _MISSING)
            self._storage[key] = value
            callbacks: List[WatcherFn] = list(self._watchers.get(key, ()))
        # Fire watchers ngoài lock để tránh deadlock khi watcher truy cập bb.
        if old is _MISSING or old != value:
            old_for_cb = None if old is _MISSING else old
            for cb in callbacks:
                try:
                    cb(old_for_cb, value)
                except Exception:
                    pass

    def __setattr__(self, key: str, value: Any) -> None:
        if key in _RESERVED:
            object.__setattr__(self, key, value)
            return
        self.set(key, value)

    def __setitem__(self, key: str, value: Any) -> None:
        self.set(key, value)

    def setdefault(self, key: str, default: Any) -> Any:
        """Trả về giá trị hiện tại, hoặc set default nếu chưa có."""
        with self._lock:
            if key not in self._storage:
                self._storage[key] = default
            return self._storage[key]

    def update(self, mapping: Dict[str, Any]) -> None:
        """Set nhiều key cùng lúc — gọi ``set`` cho từng key (fire watcher)."""
        for k, v in mapping.items():
            self.set(k, v)

    # -------------------- DELETE --------------------

    def __delattr__(self, key: str) -> None:
        with self._lock:
            self._storage.pop(key, None)

    def __delitem__(self, key: str) -> None:
        with self._lock:
            self._storage.pop(key, None)

    def clear(self) -> None:
        """Xoá hết dữ liệu (giữ lại watchers)."""
        with self._lock:
            self._storage.clear()

    # -------------------- INSPECT --------------------

    def keys(self) -> List[str]:
        with self._lock:
            return list(self._storage.keys())

    def items(self) -> List[Tuple[str, Any]]:
        with self._lock:
            return list(self._storage.items())

    def dump(self) -> Dict[str, Any]:
        """Snapshot dict — an toàn để log/print."""
        with self._lock:
            return dict(self._storage)

    def __iter__(self) -> Iterator[str]:
        return iter(self.keys())

    def __len__(self) -> int:
        return len(self._storage)

    def __repr__(self) -> str:
        return f"<Blackboard {self.dump()!r}>"

    # -------------------- WATCHERS --------------------

    def watch(self, key: str, callback: WatcherFn) -> Callable[[], None]:
        """Đăng ký callback ``cb(old, new)`` mỗi khi ``key`` thay đổi.
        Trả về hàm unwatch() để huỷ đăng ký.
        """
        with self._lock:
            self._watchers.setdefault(key, []).append(callback)

        def unwatch() -> None:
            with self._lock:
                lst = self._watchers.get(key)
                if lst and callback in lst:
                    lst.remove(callback)

        return unwatch


# Singleton — import trực tiếp ``bb`` ở mọi nơi.
bb: _Blackboard = _Blackboard()


__all__ = ["bb"]
