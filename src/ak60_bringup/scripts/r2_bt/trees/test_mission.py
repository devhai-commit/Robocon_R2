#!/usr/bin/env python3
import operator
import py_trees
from r2_bt.behaviors.navigation import GoToRelativePoseBehavior
from r2_bt.behaviors.climb import ClimbStepBehavior
from r2_bt.behaviors.hardware import WallAlignmentBehavior, MoveArmBehavior, FollowTargetBehavior
from r2_bt.behaviors.grid_logic import (
    DecideNextGridCellAction, IsAtExitCellCondition,
    CheckBlackboardValue, KeepRunningUntilSuccess, WaitForStartSignalBehavior
)
from r2_bt.trees.builders import build_move_subtree, build_place_sequence_1, build_place_sequence_2
from r2_bt.config import FIELD_CONFIGS

def create_tree(ros_node, field_color='red'):
    cfg           = FIELD_CONFIGS.get(field_color, FIELD_CONFIGS['red'])
    lat           = cfg['lat']           # +1.0 = red, -1.0 = blue
    post_ramp_yaw = cfg['post_ramp_yaw'] # -90° red / +90° blue

    # 1. SETUP BLACKBOARD
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set("real_box_count", 0)
    blackboard.set("robot_pos", (2, 0)) # Giả lập robot đã ở sẵn vạch xuất phát lưới
    blackboard.set("visit_path", [(2, 0)])

    grid = {
        (c, r): {'scanned': False, 'state': 'accessible', 'visited': False}
        for c in range(1, 4) for r in range(1, 5)
    }
    blackboard.set("grid_status", grid)

    root = py_trees.composites.Sequence("Test_Mission", memory=True)

    # --- BỎ QUA PHASE 0 VÀ PHASE 1 ---
    # Chỉ giữ lại Node chờ tín hiệu từ giao diện màn hình cảm ứng
    root.add_child(WaitForStartSignalBehavior("Wait_For_GUI_Strategy", ros_node))

    # [MOCK DATA]: Giả lập lệnh cuối cùng của Phase 1 là chỉ định đích đến (2,1)
    root.add_child(py_trees.behaviours.SetBlackboardVariable(
        "Mock_End_Phase_1", "target_cell", (2, 1), overwrite=True,
    ))

    # =====================================================================
    # CHUẨN BỊ SUB-TREE CHO PHASE 3 VÀ PHASE 4 (Đã gộp siêu gọn)
    # =====================================================================
    exit_seq = py_trees.composites.Sequence("Chuoi_Thoat_Khoi_San", memory=True)
    exit_seq.add_child(GoToRelativePoseBehavior("Thoat_Tien_025",     ros_node, dx=0.25, dy=0.0, target_yaw_deg=0.0))
    exit_seq.add_child(ClimbStepBehavior("Thoat_Xuong_Bac",           ros_node, velocity=0.1))
    exit_seq.add_child(GoToRelativePoseBehavior("Thoat_Tien_025_Sau", ros_node, dx=0.25, dy=0.0, target_yaw_deg=0.0))
    exit_seq.add_child(py_trees.decorators.FailureIsSuccess("Ignore_Align_Exit",
        WallAlignmentBehavior("Thoat_Align_1", ros_node, window_degrees=60.0, goal_distance=0.4),
    ))

    branch_selector = py_trees.composites.Selector("Chon_Huong_Thoat", memory=False)
    seq_3_4 = py_trees.composites.Sequence("Thoat_Tu_3_4", memory=True)
    seq_3_4.add_child(CheckBlackboardValue("La_o_3_4?", "robot_pos", (3, 4), operator.eq))
    seq_3_4.add_child(GoToRelativePoseBehavior("Di_Ve_Doc_1_3m", ros_node, dx=0.0, dy=lat*1.3, target_yaw_deg=0.0))

    seq_1_4 = py_trees.composites.Sequence("Thoat_Tu_1_4", memory=True)
    seq_1_4.add_child(CheckBlackboardValue("La_o_1_4?", "robot_pos", (1, 4), operator.eq))
    seq_1_4.add_child(GoToRelativePoseBehavior("Di_Ve_Doc_3_7m", ros_node, dx=0.0, dy=lat*3.7, target_yaw_deg=0.0))

    branch_selector.add_children([seq_3_4, seq_1_4])
    exit_seq.add_child(branch_selector)

    exit_seq.add_child(py_trees.decorators.FailureIsSuccess("Ignore_Align_Ramp",
        WallAlignmentBehavior("Thoat_Align_2", ros_node, window_degrees=60.0, goal_distance=0.4),
    ))
    exit_seq.add_child(ClimbStepBehavior("Thoat_Len_Doc",       ros_node, velocity=0.5))
    exit_seq.add_child(GoToRelativePoseBehavior("Quay_Ve_Arena", ros_node, dx=0.0, dy=0.0, target_yaw_deg=post_ramp_yaw))
    exit_seq.add_child(GoToRelativePoseBehavior("Tien_4m",       ros_node, dx=4.0, dy=0.0, target_yaw_deg=post_ramp_yaw))

    place_boxes_seq = py_trees.composites.Sequence("Chuoi_Dat_2_Hop", memory=True)
    place_boxes_seq.add_child(py_trees.decorators.FailureIsSuccess("Align 1.5m", WallAlignmentBehavior("Align 1.5m", ros_node, 60.0, 1.5)))
    place_boxes_seq.add_child(py_trees.decorators.FailureIsSuccess("AI Can Tam", FollowTargetBehavior("AI Căn Tâm (ID=1)", ros_node, 1, 400.0)))
    place_boxes_seq.add_child(build_place_sequence_1(ros_node))
    place_boxes_seq.add_child(py_trees.decorators.FailureIsSuccess("Ep sat 0.15m (1)", WallAlignmentBehavior("Ép sát 0.15m", ros_node, 60.0, 0.15)))
    place_boxes_seq.add_child(MoveArmBehavior("Mo tay gap 1", ros_node, [400.0, 0.0, 90.0, 45.0]))
    place_boxes_seq.add_child(GoToRelativePoseBehavior("Lui 0.5m", ros_node, -0.5, 0.0, -90.0))
    place_boxes_seq.add_child(MoveArmBehavior("Thu tay ve Home (1)", ros_node, [230.0, 0.0, 90.0, 0.0]))
    place_boxes_seq.add_child(GoToRelativePoseBehavior("Sang phai 0.54m", ros_node, 0.0, -0.54, -90.0))
    place_boxes_seq.add_child(build_place_sequence_2(ros_node))
    place_boxes_seq.add_child(py_trees.decorators.FailureIsSuccess("Ep sat 0.15m (2)", WallAlignmentBehavior("Ép sát 0.15m (2)", ros_node, 60.0, 0.15)))
    place_boxes_seq.add_child(MoveArmBehavior("Mo tay gap 2", ros_node, [400.0, 0.0, 90.0, 45.0]))
    place_boxes_seq.add_child(GoToRelativePoseBehavior("Lui xa 1.5m", ros_node, -1.5, 0.0, -90.0))
    place_boxes_seq.add_child(MoveArmBehavior("Thu tay ve Home (2)", ros_node, [230.0, 0.0, 90.0, 0.0]))

    # GỘP THÀNH SUB-TREE GỌN GÀNG
    phase_3_4_subtree = py_trees.composites.Sequence("Phase_3_4_Thoat_Va_Dat_Hop", memory=True)
    phase_3_4_subtree.add_child(exit_seq)
    phase_3_4_subtree.add_child(place_boxes_seq)

    # =====================================================================
    # --- PHASE 2: LOOP ĐIỀU HƯỚNG LƯỚI SA BÀN ---
    # =====================================================================
    mode_selector = py_trees.composites.Selector("Mode_Selector", memory=False)

    exit_goal_seq = py_trees.composites.Sequence("Mode_EXIT_Goal", memory=True)
    exit_goal_seq.add_child(CheckBlackboardValue("Real_GE_2?", "real_box_count", 2, operator.ge))
    exit_goal_seq.add_child(IsAtExitCellCondition("At_Exit_Cell?"))
    exit_goal_seq.add_child(phase_3_4_subtree) # Gọi nguyên cục Subtree 3-4 ở đây
    exit_goal_seq.add_child(py_trees.behaviours.Success("Mission_Complete"))
    mode_selector.add_child(exit_goal_seq)

    find_exit_seq = py_trees.composites.Sequence("Mode_FIND_Exit", memory=True)
    find_exit_seq.add_child(CheckBlackboardValue("Real_GE_2_Find?", "real_box_count", 2, operator.ge))
    find_exit_seq.add_child(DecideNextGridCellAction("Route_Exit", ros_node, mode="EXIT"))
    find_exit_seq.add_child(build_move_subtree(ros_node))
    mode_selector.add_child(find_exit_seq)

    hunt_seq = py_trees.composites.Sequence("Mode_HUNT", memory=True)
    hunt_seq.add_child(DecideNextGridCellAction("Route_Explore", ros_node, mode="EXPLORE"))
    hunt_seq.add_child(build_move_subtree(ros_node))
    mode_selector.add_child(hunt_seq)

    loop = KeepRunningUntilSuccess("Loop_Explore_Exit", child=mode_selector)

    grid_phase = py_trees.composites.Sequence("Vao_Luoi_Va_Lap", memory=True)
    grid_phase.add_child(build_move_subtree(ros_node)) # Xử lý ô mồi (2,1)
    grid_phase.add_child(loop)                         # Bắt đầu vòng lặp

    root.add_child(grid_phase)
    root.add_child(py_trees.behaviours.Running("TEST_COMPLETE_STANDBY"))

    return root

