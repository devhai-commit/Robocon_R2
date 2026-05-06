#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import time
import threading
import operator

from bt_nodes import *
from bt_sequences import *

# ==========================================================
# KHỞI TẠO CÂY TỔNG THỂ
# ==========================================================
def create_tree(ros_node):
    root = py_trees.composites.Sequence("Main Mission", memory=True)
    
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.real_box_count = 0
    blackboard.robot_pos = (2, 0) 
    blackboard.visit_path = [blackboard.robot_pos] 
    blackboard.grid_status = {(c, r): {'scanned': False, 'state': 'accessible', 'visited': False} for c in range(1, 4) for r in range(1, 5)}

    # --- PHASE 0: GẬP TAY AN TOÀN & CHỜ LỆNH START TỪ MÀN HÌNH ---
    pre_init_seq = py_trees.composites.Sequence("Phase_0_Standby", memory=True)
    pre_init_seq.add_child(ArmSequenceBTNode("Tool_Arm_Home", ros_node, "home_pose_right", duration=2.0))
    pre_init_seq.add_child(MoveArmBehavior("Box_Arm_Home", ros_node, target_pose=[250.0, 0.0, 90.0, 0.0]))
    pre_init_seq.add_child(WaitForStartSignalBehavior("Wait_For_GUI_Strategy", ros_node))
    root.add_child(pre_init_seq)

    # --- PHASE 1: LẤY DỤNG CỤ VÀ TIẾN RA CỬA LƯỚI ---
    init_seq = py_trees.composites.Sequence("Phase_1_Khoi_Dong", memory=True)
    init_seq.add_child(GoToRelativePoseBehavior("Tien_Lay_Dung_Cu", ros_node, dx=0.3, dy=1.0, target_yaw_deg=0.0))
    init_seq.add_child(build_tool_assembly_sequence_right(ros_node))
    init_seq.add_child(GoToRelativePoseBehavior("Tien_Ra_Cua_Grid", ros_node, dx=1.6, dy=-2.5, target_yaw_deg=0.0))
    init_seq.add_child(py_trees.behaviours.SetBlackboardVariable("Target=(2,1)", "target_cell", (2, 1), overwrite=True))
    root.add_child(init_seq)

    # # --- PHASE 3: THOÁT KHỎI SA BÀN ---
    # exit_sequence = py_trees.composites.Sequence("Chuoi_Thoat_Khoi_San", memory=True)
    # exit_sequence.add_child(GoToRelativePoseBehavior("Thoat: Tien 0.25m", ros_node, dx=0.25, dy=0.0, target_yaw_deg=0.0))
    # exit_sequence.add_child(ClimbStepBehavior("Thoat: Xuong bac v=0.1", ros_node, velocity=0.1))
    # exit_sequence.add_child(GoToRelativePoseBehavior("Thoat: Tien 0.25m sau xuong", ros_node, dx=0.25, dy=0.0, target_yaw_deg=0.0))
    # exit_sequence.add_child(py_trees.decorators.FailureIsSuccess("Ignore Align", WallAlignmentBehavior("Align 1", ros_node, 60.0, 0.4)))
    
    # branch_exit_selector = py_trees.composites.Selector("Chon_Huong_Dich_Trai", memory=False)
    # seq_3_4 = py_trees.composites.Sequence("Dich_Trai_Tu_3_4", memory=True)
    # seq_3_4.add_child(CheckBlackboardValue("La o (3,4)?", "robot_pos", (3, 4), operator.eq))
    # seq_3_4.add_child(GoToRelativePoseBehavior("Thoat: Sang trai 1.3m", ros_node, dx=0.0, dy=1.3, target_yaw_deg=0.0))
    # seq_1_4 = py_trees.composites.Sequence("Dich_Trai_Tu_1_4", memory=True)
    # seq_1_4.add_child(CheckBlackboardValue("La o (1,4)?", "robot_pos", (1, 4), operator.eq))
    # seq_1_4.add_child(GoToRelativePoseBehavior("Thoat: Sang trai 3.7m", ros_node, dx=0.0, dy=3.7, target_yaw_deg=0.0))
    
    # branch_exit_selector.add_children([seq_3_4, seq_1_4])
    # exit_sequence.add_child(branch_exit_selector)
    # exit_sequence.add_child(py_trees.decorators.FailureIsSuccess("Ignore Align Ramp", WallAlignmentBehavior("Align Truoc Doc", ros_node, 60.0, 0.4)))
    # exit_sequence.add_child(ClimbStepBehavior("Thoat: Len doc cuoi", ros_node, velocity=0.5))
    # exit_sequence.add_child(GoToRelativePoseBehavior("Thoat: Quay 90 do", ros_node, dx=0.0, dy=0.0, target_yaw_deg=-90.0))
    # exit_sequence.add_child(GoToRelativePoseBehavior("Thoat: Tien thang 4m", ros_node, dx=4.0, dy=0.0, target_yaw_deg=-90.0))

    # # --- PHASE 4: ĐẶT 2 HỘP LÊN CỘT SILO (13 BƯỚC) ---
    # place_boxes_seq = py_trees.composites.Sequence("Chuoi_Dat_2_Hop", memory=True)
    # place_boxes_seq.add_child(py_trees.decorators.FailureIsSuccess("Align 1.5m", WallAlignmentBehavior("Align 1.5m", ros_node, 60.0, 1.5)))
    # place_boxes_seq.add_child(py_trees.decorators.FailureIsSuccess("AI Can Tam", FollowTargetBehavior("AI Căn Tâm (ID=1)", ros_node, 1, 400.0, "REAL")))
    # place_boxes_seq.add_child(build_place_sequence_1(ros_node))
    # place_boxes_seq.add_child(py_trees.decorators.FailureIsSuccess("Ep sat 0.15m (1)", WallAlignmentBehavior("Ép sát 0.15m", ros_node, 60.0, 0.15)))
    # place_boxes_seq.add_child(MoveArmBehavior("Mo tay gap 1", ros_node, [400.0, 0.0, 90.0, 45.0]))
    # place_boxes_seq.add_child(GoToRelativePoseBehavior("Lui 0.5m", ros_node, -0.5, 0.0, -90.0))
    # place_boxes_seq.add_child(MoveArmBehavior("Thu tay ve Home (1)", ros_node, [250.0, 0.0, 90.0, 0.0]))
    # place_boxes_seq.add_child(GoToRelativePoseBehavior("Sang phai 0.54m", ros_node, 0.0, -0.54, -90.0))
    # place_boxes_seq.add_child(build_place_sequence_2(ros_node))
    # place_boxes_seq.add_child(py_trees.decorators.FailureIsSuccess("Ep sat 0.15m (2)", WallAlignmentBehavior("Ép sát 0.15m (2)", ros_node, 60.0, 0.15)))
    # place_boxes_seq.add_child(MoveArmBehavior("Mo tay gap 2", ros_node, [400.0, 0.0, 90.0, 45.0]))
    # place_boxes_seq.add_child(GoToRelativePoseBehavior("Lui xa 1.5m", ros_node, -1.5, 0.0, -90.0))
    # place_boxes_seq.add_child(MoveArmBehavior("Thu tay ve Home (2)", ros_node, [230.0, 0.0, 90.0, 0.0]))

    # # --- PHASE 2: ĐIỀU HƯỚNG LƯỚI & GHÉP NỐI ---
    # mode_selector = py_trees.composites.Selector("Mode_Control", memory=False)
    
    # exit_check_seq = py_trees.composites.Sequence("Mode: EXIT_NOW", memory=True)
    # exit_check_seq.add_child(CheckBlackboardValue("Du_2_hop?", "real_box_count", 2, operator.ge))
    # exit_check_seq.add_child(IsAtExitCellCondition("At_Exit_Gate?"))
    # exit_check_seq.add_child(exit_sequence)   
    # exit_check_seq.add_child(place_boxes_seq) 
    # exit_check_seq.add_child(py_trees.behaviours.Success("MISSION_ACCOMPLISHED"))
    # mode_selector.add_child(exit_check_seq)

    # path_exit_seq = py_trees.composites.Sequence("Mode: FIND_EXIT_GATE", memory=True)
    # path_exit_seq.add_child(CheckBlackboardValue("Du_2_hop?", "real_box_count", 2, operator.ge))
    # path_exit_seq.add_child(DecideNextGridCellAction("Route_to_Exit", ros_node, mode="EXIT"))
    # path_exit_seq.add_child(build_move_subtree(ros_node)) 
    # mode_selector.add_child(path_exit_seq)

    # path_hunt_seq = py_trees.composites.Sequence("Mode: HUNTING", memory=True)
    # path_hunt_seq.add_child(DecideNextGridCellAction("Next_Explore_Cell", ros_node, mode="EXPLORE"))
    # path_hunt_seq.add_child(build_move_subtree(ros_node)) 
    # mode_selector.add_child(path_hunt_seq)

    # loop_decorator = KeepRunningUntilSuccess("Grid_Mission_Loop", child=mode_selector)
    
    # grid_phase_seq = py_trees.composites.Sequence("Phase_2_Grid_Traversal", memory=True)
    # grid_phase_seq.add_child(build_move_subtree(ros_node)) # Ô mồi 2,1
    # grid_phase_seq.add_child(loop_decorator) 
    
    # root.add_child(grid_phase_seq)

    # Thêm chốt chặn chống Reset sau khi xong nhiệm vụ!
    root.add_child(py_trees.behaviours.Running("MISSION_COMPLETE_STANDBY"))

    return root

# ==========================================================
# THREAD TICKER & MAIN
# ==========================================================
def bt_ticker_thread(node, tree, blackboard):
    node.get_logger().info("🚀 Bắt đầu tick Behavior Tree...")
    while rclpy.ok():
        try:
            tree.tick() 
            print("\n" + "="*50)
            print(py_trees.display.ascii_tree(tree.root, show_status=True)) 
            time.sleep(0.5) 
        except Exception as e:
            node.get_logger().error(f"Lỗi khi tick cây: {e}")
            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = Node('app_manager_bt_node')
    
    node.declare_parameter('map_cols', [2, 1, 3, 2])
    node.declare_parameter('map_rows', [1, 2, 2, 3])
    node.declare_parameter('map_heights', [600.0, 200.0, 200.0, 600.0])
    node.declare_parameter('default_elevation', 400.0)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    blackboard = py_trees.blackboard.Blackboard()
    tree_root = create_tree(node)
    behaviour_tree = py_trees_ros.trees.BehaviourTree(tree_root)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    node.get_logger().info("Đã bật ROS 2 Executor. Đang quét các Action Server...")

    try:
        behaviour_tree.setup(timeout=60.0, node=node)
        node.get_logger().info("✅ KẾT NỐI SERVER THÀNH CÔNG! Robot vào trạng thái chờ Lệnh GUI...")

        bt_thread = threading.Thread(target=bt_ticker_thread, args=(node, behaviour_tree, blackboard))
        bt_thread.daemon = True
        bt_thread.start()

        while rclpy.ok(): time.sleep(1.0)
            
    except KeyboardInterrupt:
        node.get_logger().warn('Dừng khẩn cấp do người dùng (Ctrl+C).')
    except Exception as e:
        node.get_logger().error(f"❌ HỦY CHẠY CÂY HÀNH VI: {e}")
    finally:
        behaviour_tree.interrupt()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)

if __name__ == '__main__':
    main()

