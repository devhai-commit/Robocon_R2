#!/usr/bin/env python3
import time
import threading
import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
# from r2_bt.trees.mission_test import create_tree
from r2_bt.trees.mission import create_tree
from r2_bt.config import FIELD_CONFIGS


def bt_ticker_thread(node, tree):
    node.get_logger().info("Bắt đầu tick Behavior Tree...")
    while rclpy.ok():
        tree.tick()
        print(py_trees.display.ascii_tree(tree.root, show_status=True))
        print("-" * 20 + " BLACKBOARD " + "-" * 20)
        print(py_trees.display.unicode_blackboard())
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = Node('app_manager_bt_node')

    # Chọn sân: 'red' hoặc 'blue'
    # Ví dụ: ros2 run ak60_bringup app_manager_bt_node --ros-args -p field_color:=blue
    node.declare_parameter('field_color', 'red')
    field_color = node.get_parameter('field_color').get_parameter_value().string_value
    if field_color not in FIELD_CONFIGS:
        node.get_logger().warn(f"field_color='{field_color}' không hợp lệ, dùng 'red'.")
        field_color = 'red'

    cfg = FIELD_CONFIGS[field_color]
    node.get_logger().info(
        f"[Config] Sân: {field_color.upper()} | lat={cfg['lat']} | post_ramp_yaw={cfg['post_ramp_yaw']}°"
    )

    node.declare_parameter('map_cols',          cfg['map_cols'])
    node.declare_parameter('map_rows',          cfg['map_rows'])
    node.declare_parameter('map_heights',       cfg['map_heights'])
    node.declare_parameter('default_elevation', cfg['default_elevation'])

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    tree_root      = create_tree(node, field_color=field_color)
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root=tree_root)

    try:
        behaviour_tree.setup(timeout=30.0, node=node)
    except py_trees_ros.exceptions.TimedOutError as e:
        node.get_logger().error(f"Không thể setup BT (Server Timeout): {e}")
        node.destroy_node()
        rclpy.shutdown()
        return
    except Exception as e:
        node.get_logger().error(f"Không thể setup BT: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    # KHÔNG dùng behaviour_tree.tick_tock(...) cùng với bt_ticker_thread —
    # hai nguồn tick song song sẽ mutate deque tick_interval_history khi
    # statistics handler đang iterate, gây "deque mutated during iteration".
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    bt_thread = threading.Thread(
        target=bt_ticker_thread, args=(node, behaviour_tree), daemon=True,
    )
    bt_thread.start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        node.get_logger().warn("Dừng khẩn cấp.")
    finally:
        behaviour_tree.interrupt()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
