#!/usr/bin/env python3
import operator
import py_trees
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from r2_bt.behaviors.blackboard_ops import ROSTopicToBB, BlackboardLoggerBehavior
from r2_bt.behaviors.navigation import GoToRelativePoseBehavior, GoToEntranceBoxBehavior, K230AlignBehavior, ResetPoseBehavior
from r2_bt.jetson_read_uart import start_reader_thread
from r2_bt.behaviors.climb import ClimbStepBehavior
from r2_bt.behaviors.hardware import FollowTargetBehavior, WallAlignmentBehavior, ArmSequenceBTNode, MoveArmBehavior
from r2_bt.behaviors.grid_logic import (
    DecideNextGridCellAction, IsAtExitCellCondition,
    KeepRunningUntilSuccess, WaitForStartSignalBehavior,
    IncrementRealCountAction,
)
from r2_bt.trees.builders import build_tool_assembly_sequence, build_move_subtree, build_pick_and_place_sequence, build_pick_and_place_sequence_dynamic, _build_arm_seq
from r2_bt.config import FIELD_CONFIGS, GRID_CELL_SIZE, PICK_STEPS_COUNT_1


def build_go_to_row1_cell(ros_node, target_box, lat):
    """Xây dựng Sequence di chuyển robot từ Zone 1 (khu vực dụng cụ) đến trước ô hàng 1.

    Args:
        target_box: tuple (col, 1) với col ∈ {1, 2, 3} — chỉ chấp nhận hàng 1
        lat:        hệ số hướng ngang (+1.0 red / -1.0 blue) từ FIELD_CONFIGS
    Returns:
        py_trees.composites.Sequence chứa 2 GoToRelativePoseBehavior:
          1. Thoát khỏi khu vực dụng cụ (dy ngược chiều tiếp cận)
          2. Tiến ra trước ô (col, 1) — offset cột tính từ cột 2 mặc định
    """
    col, row = target_box
    if row != 1:
        raise ValueError(f"target_box phải thuộc hàng 1, nhận được ({col}, {row})")

    seq = py_trees.composites.Sequence(f"Di_Den_Truoc_O_{col}_1", memory=True)

    seq.add_child(GoToRelativePoseBehavior(
        "Thoat_Khu_Dung_Cu", ros_node,
        dx=0.0, dy=-lat * 1.05, target_yaw_deg=0.0,
    ))

    dy_to_cell = lat * (-2.6 + (col - 2) * GRID_CELL_SIZE)
    seq.add_child(GoToRelativePoseBehavior(
        f"Tien_Ra_Cua_O_{col}_1", ros_node,
        dx=1.6, dy=dy_to_cell, target_yaw_deg=0.0,
    ))

    return seq


_SEED_KEYS = (
    "real_box_count",
    "robot_pos",
    "visit_path",
    "grid_status",
    "target_cell",
    "just_climbed",
    "just_picked",
    "current_arm1_z",
    "priority_col",
    "target_boxes_list",
    "entrance_boxes"
)


def _seed_blackboard():
    """Seed các key mặc định lên py_trees blackboard.

    Dùng 1 client riêng tên ``MissionSeed`` để write các key mà
    behaviour khác sẽ đọc. py_trees cho phép nhiều client cùng
    register WRITE 1 key, nên không xung đột với writer runtime.
    """
    seed = py_trees.blackboard.Client(name="MissionSeed")
    for key in _SEED_KEYS:
        seed.register_key(key=key, access=py_trees.common.Access.WRITE)

    seed.real_box_count = 0
    seed.robot_pos = (2, 0)
    seed.visit_path = [(2, 0)]
    seed.grid_status = {
        (c, r): {'scanned': False, 'state': 'accessible', 'visited': False}
        for c in range(1, 4) for r in range(1, 5)
    }
    seed.target_cell = (2, 1)
    seed.just_climbed = False
    seed.just_picked = "no"
    seed.current_arm1_z = 125.0
    seed.priority_col = 2
    seed.target_boxes_list = []
    seed.entrance_boxes = []


def _build_topics_to_bb(ros_node):
    """Nhánh Topics2BB — gom mọi subscribe ROS vào blackboard.

    Mọi callback async của ROS chỉ ghi vào 1 key duy nhất ở đây. Các
    behaviour ở nhánh chính chỉ ĐỌC blackboard, không tự subscribe.
    """
    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)

    gui_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    gui_signal_to_bb = ROSTopicToBB(
        name="GuiSignal2BB",
        ros_node=ros_node,
        topic_name="/gui_start_signal",
        topic_type=String,
        key="gui_start_raw",
        field="data",
        qos_profile=gui_qos,
    )
    topics2bb.add_child(gui_signal_to_bb)
    return topics2bb


def _build_main_mission(ros_node, lat, post_ramp_yaw, side, ai_target_id, r1_id,
                        entrance_box_coords, entrance_default_target):
    main = py_trees.composites.Sequence("Main_Mission", memory=True)

    # --- PHASE 0: GẬP TAY AN TOÀN & CHỜ LỆNH START TỪ MÀN HÌNH ---
    pre_init_seq = py_trees.composites.Sequence("Phase_0_Standby", memory=True)
    # pre_init_seq.add_child(ArmSequenceBTNode("Tool_Arm_Home", ros_node, f"home_pose_{side}", duration=2.0))
    pre_init_seq.add_child(MoveArmBehavior("Box_Arm_Home", ros_node, target_pose=[325.0, 0.0, 0.0, 0.0]))
    pre_init_seq.add_child(WaitForStartSignalBehavior("Wait_For_GUI_Strategy", ros_node))
    pre_init_seq.add_child(ResetPoseBehavior("Reset_Pose_At_Start", ros_node, x=0.0, y=0.0, yaw_deg=0.0))
    main.add_child(pre_init_seq)

    # # --- Phase 1: Lấy dụng cụ và tiến ra cửa lưới ---
    init_seq = py_trees.composites.Sequence("Khoi_Dong_Va_Lay_Dung_Cu", memory=True)
    init_seq.add_child(ArmSequenceBTNode("Tool_Arm_Approach", ros_node, f"approach_j4_{side}", duration=1.0))
    init_seq.add_child(GoToRelativePoseBehavior("Tien_Lay_Dung_Cu", ros_node, dx=0.4, dy=lat*0.6, target_yaw_deg=0.0))
    # init_seq.add_child(GoToRelativePoseBehavior("Tien_Lay_Dung_Cu", ros_node, dx=0.0, dy=lat * 1.0, target_yaw_deg=0.0))
    # init_seq.add_child(GoToRelativePoseBehavior("Tien_Lay_Dung_Cu", ros_node, dx=0.0, dy=-lat * 1.05, target_yaw_deg=0.0))
    init_seq.add_child(py_trees.decorators.FailureIsSuccess(
        "Try_K230_Align",
        K230AlignBehavior("K230_Align_Before_Tool", ros_node),
    ))
    init_seq.add_child(build_tool_assembly_sequence(ros_node, side=side))
    # init_seq.add_child(GoToRelativePoseBehavior("Xoay_Lap", ros_node, dx=0.0, dy=-lat*0.2, target_yaw_deg=180.0))
    init_seq.add_child(ArmSequenceBTNode("Lap_Vu_Khi", ros_node, "assemble_sequence" , duration=20.0))
    # init_seq.add_child(GoToRelativePoseBehavior("Xoay_Ban_Dau", ros_node, dx=0.0, dy=0.0, target_yaw_deg=0.0))
    # init_seq.add_child(GoToRelativePoseBehavior("Tien_Ra_Cua_Cell", ros_node, dx=1.6, dy=-lat * 2.6, target_yaw_deg=0.0))
    init_seq.add_child(GoToEntranceBoxBehavior("Di_Den_Entrance_Box", ros_node,
        box_coords=entrance_box_coords, default_target=entrance_default_target))

    init_seq.add_child(WallAlignmentBehavior("Align_Cua_Cell", ros_node, window_degrees=20.0, goal_distance=0.0))

    entrance_action_sel = py_trees.composites.Selector("Entrance_Action", memory=False)

    col1_seq = py_trees.composites.Sequence("Entrance_1_1", memory=True)
    col1_seq.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="Is_1_1?",
        check=py_trees.common.ComparisonExpression(
            variable="entrance_boxes", value=(1, 1), operator=operator.contains,
        ),
    ))
    col1_seq.add_child(MoveArmBehavior("Box_Arm_Home", ros_node, target_pose=[500.0, 0.0, 0.0, 0.0]))
    col1_seq.add_child(py_trees.decorators.FailureIsSuccess(
        "Try_Follow_Cua_Cell",
        FollowTargetBehavior("Follow_Cua_Cell", ros_node, target_id=ai_target_id, desired_distance_mm=325.0),
    ))
    col1_seq.add_child(_build_arm_seq(ros_node, preset_key='400', steps=PICK_STEPS_COUNT_1))
    col1_seq.add_child(IncrementRealCountAction("Inc_Real_Count_1_1"))
    col1_seq.add_child(GoToRelativePoseBehavior("Lui_Adj_1_1", ros_node, dx=-0.2, dy=0.0, target_yaw_deg=0.0))
    col1_seq.add_child(GoToRelativePoseBehavior("Adj_Col1", ros_node, dx=0.0, dy=lat*1.0, target_yaw_deg=0.0))

    col3_seq = py_trees.composites.Sequence("Entrance_3_1", memory=True)
    col3_seq.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="Is_3_1?",
        check=py_trees.common.ComparisonExpression(
            variable="entrance_boxes", value=(3, 1), operator=operator.contains,
        ),
    ))
    col3_seq.add_child(MoveArmBehavior("Box_Arm_Home", ros_node, target_pose=[500.0, 0.0, 0.0, 0.0]))
    col3_seq.add_child(py_trees.decorators.FailureIsSuccess(
        "Try_Follow_Cua_Cell",
        FollowTargetBehavior("Follow_Cua_Cell", ros_node, target_id=ai_target_id, desired_distance_mm=325.0),
    ))
    col3_seq.add_child(_build_arm_seq(ros_node, preset_key='400', steps=PICK_STEPS_COUNT_1))
    col3_seq.add_child(IncrementRealCountAction("Inc_Real_Count_3_1"))
    col3_seq.add_child(GoToRelativePoseBehavior("Lui_Adj_3_1", ros_node, dx=-0.2, dy=0.0, target_yaw_deg=0.0))
    col3_seq.add_child(GoToRelativePoseBehavior("Adj_Col3", ros_node, dx=0.0, dy=-lat*1.0, target_yaw_deg=0.0))

    entrance_action_sel.add_children([col1_seq, col3_seq, py_trees.behaviours.Success("Skip_2_1")])
    init_seq.add_child(entrance_action_sel)

    main.add_child(init_seq)


    # --- Phase 3: Thoát khỏi sàn ---

    exit_seq = py_trees.composites.Sequence("Chuoi_Thoat_Khoi_San", memory=True)
    # Bắt buộc quay về hướng thẳng dốc (yaw=0) trước khi xuống bậc,
    # tránh robot thoát nhầm sang đường của R1.
    exit_seq.add_child(GoToRelativePoseBehavior("Quay_Huong_Thoat", ros_node, dx=0.0, dy=0.0, target_yaw_deg=0.0))
    exit_seq.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_Truoc_Bac",
        WallAlignmentBehavior("Align_Truoc_Khi_Thoat", ros_node, window_degrees=20.0, goal_distance=0.0),
    ))
    exit_seq.add_child(ClimbStepBehavior("Thoat_Xuong_Bac",           ros_node, velocity=0.1))
    exit_seq.add_child(GoToRelativePoseBehavior("Thoat_Tien_025_Sau", ros_node, dx=0.25, dy=0.0, target_yaw_deg=0.0))
    exit_seq.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_Exit",
        WallAlignmentBehavior("Thoat_Align_1", ros_node, window_degrees=20.0, goal_distance=0.0),
    ))

    branch_selector = py_trees.composites.Selector("Chon_Huong_Thoat", memory=False)

    # Red và blue bị mirror: col3 ↔ col1 đổi khoảng cách vật lý đến dốc.
    dist_col3, dist_col1 = (3.7, 1.1) if lat > 0 else (1.1, 3.7)

    seq_3_4 = py_trees.composites.Sequence("Thoat_Tu_3_4", memory=True)
    seq_3_4.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="La_o_3_4?",
        check=py_trees.common.ComparisonExpression(
            variable="robot_pos", value=(3, 4), operator=operator.eq,
        ),
    ))
    seq_3_4.add_child(GoToRelativePoseBehavior("Di_Ve_Doc_col3", ros_node, dx=0.0, dy=-lat * dist_col3, target_yaw_deg=0.0))

    seq_1_4 = py_trees.composites.Sequence("Thoat_Tu_1_4", memory=True)
    seq_1_4.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="La_o_1_4?",
        check=py_trees.common.ComparisonExpression(
            variable="robot_pos", value=(1, 4), operator=operator.eq,
        ),
    ))
    seq_1_4.add_child(GoToRelativePoseBehavior("Di_Ve_Doc_col1", ros_node, dx=0.0, dy=-lat * dist_col1, target_yaw_deg=0.0))

    branch_selector.add_children([seq_3_4, seq_1_4])

    exit_seq.add_child(branch_selector)

    exit_seq.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_Ramp",
        WallAlignmentBehavior("Thoat_Align_2", ros_node, window_degrees=20.0, goal_distance=0.0),
    ))
    exit_seq.add_child(ClimbStepBehavior("Thoat_Len_Doc",        ros_node, velocity=0.6))
    exit_seq.add_child(GoToRelativePoseBehavior("Quay_Ve_Arena", ros_node, dx=0.0, dy=0.0, target_yaw_deg=post_ramp_yaw))
    exit_seq.add_child(GoToRelativePoseBehavior("Tien_vao_tic_tac_toe",  ros_node, dx=3.5, dy=1.0 * lat, target_yaw_deg=post_ramp_yaw))
    exit_seq.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_Sau_Ramp",
        WallAlignmentBehavior("Align_Sau_Ramp", ros_node, window_degrees=20.0, goal_distance=0.0),
    ))

    # --- PHASE 4: ĐẶT 2 HỘP LÊN GIA (13 BƯỚC) ---
    place_box = py_trees.composites.Sequence("Dat_Hop_Vao_Gia", memory=True)
    # ===== TRacking AI =============================
    place_box.add_child(py_trees.decorators.FailureIsSuccess(
        "Run_AI_Exit",
        FollowTargetBehavior("Run_AI_Exit", ros_node, target_id=r1_id, desired_distance_mm=300.0)))
# ===== Lay va dat hop 2 len gia =============================
    place_box.add_child(MoveArmBehavior("Move_Arm_pose_11", ros_node, target_pose=[590.0, 0.0, 90.0, 180.0]))
    place_box.add_child(MoveArmBehavior("Move_Arm_pose_12", ros_node, target_pose=[640.0, 0.0, 90.0, -30.0]))
    place_box.add_child(MoveArmBehavior("Move_Arm_pose_13", ros_node, target_pose=[780.0, 0.0, 90.0, -30.0]))
    place_box.add_child(MoveArmBehavior("Move_Arm_pose_14", ros_node, target_pose=[780.0, 0.0, -70.0, -30.0]))
    place_box.add_child(WallAlignmentBehavior("Align_Exit_Cell", ros_node, window_degrees=20.0, goal_distance=0.35, timeout_sec=10.0))
    place_box.add_child(MoveArmBehavior("Move_Arm_pose_15", ros_node, target_pose=[780.0, 0.0, -70.0, 180.0]))
    place_box.add_child(GoToRelativePoseBehavior("Tien_0.5m_Exit", ros_node, dx=-0.5, dy=0.0, target_yaw_deg=post_ramp_yaw))
    place_box.add_child(MoveArmBehavior("Move_Arm_pose_16", ros_node, target_pose=[290.0, 0.0, 90.0, 180.0]))

#===== Di chuyen den vi tri dat hop len gia =============================
    place_box.add_child(GoToRelativePoseBehavior("Tien_0.5m_Exit_2", ros_node, dx=0.0, dy=0.54, target_yaw_deg=post_ramp_yaw))
# ===== Lay va dat hop 1 len gia =============================
    place_box.add_child(MoveArmBehavior("Move_Arm_pose_21", ros_node, target_pose=[290.0, 0.0, 90.0, -30.0]))
    place_box.add_child(MoveArmBehavior("Move_Arm_pose_22", ros_node, target_pose=[780.0, 0.0, 90.0, -30.0]))
    place_box.add_child(MoveArmBehavior("Move_Arm_pose_23", ros_node, target_pose=[780.0, 0.0, -70.0, -30.0]))
    place_box.add_child(WallAlignmentBehavior("Align_Exit_Cell", ros_node, window_degrees=20.0, goal_distance=0.4, timeout_sec=10.0))
    place_box.add_child(MoveArmBehavior("Move_Arm_pose_24", ros_node, target_pose=[780.0, 0.0, -70.0, 180.0]))
    place_box.add_child(GoToRelativePoseBehavior("Tien_0.5m_Exit_3", ros_node, dx=-0.5, dy=0.0, target_yaw_deg=post_ramp_yaw))
    place_box.add_child(MoveArmBehavior("Move_Arm_pose_25", ros_node, target_pose=[290.0, 0.0, 0.0, 180.0]))
    

    # --- Phase 2: Loop điều hướng lưới ---

    mode_selector = py_trees.composites.Selector("Mode_Selector", memory=False)

    exit_goal_seq = py_trees.composites.Sequence("Mode_EXIT_Goal", memory=True)
    exit_goal_seq.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="Config_Done?",
        check=py_trees.common.ComparisonExpression(
            variable="target_boxes_list", value=[], operator=operator.eq,
        ),
    ))
    exit_goal_seq.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="Real_GE_2?",
        check=py_trees.common.ComparisonExpression(
            variable="real_box_count", value=2, operator=operator.ge,
        ),
    ))
    exit_goal_seq.add_child(IsAtExitCellCondition("At_Exit_Cell?"))
    mode_selector.add_child(exit_goal_seq)

    find_exit_seq = py_trees.composites.Sequence("Mode_FIND_Exit", memory=True)
    find_exit_seq.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="Config_Done_Find?",
        check=py_trees.common.ComparisonExpression(
            variable="target_boxes_list", value=[], operator=operator.eq,
        ),
    ))
    find_exit_seq.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="Real_GE_2_Find?",
        check=py_trees.common.ComparisonExpression(
            variable="real_box_count", value=2, operator=operator.ge,
        ),
    ))
    find_exit_seq.add_child(DecideNextGridCellAction("Route_Exit", ros_node, mode="EXIT"))
    find_exit_seq.add_child(build_move_subtree(ros_node, ai_target_id))
    find_exit_seq.add_child(py_trees.behaviours.Failure("Loop_Find_Exit"))
    mode_selector.add_child(find_exit_seq)

    hunt_seq = py_trees.composites.Sequence("Mode_HUNT", memory=True)
    hunt_seq.add_child(DecideNextGridCellAction("Route_Explore", ros_node, mode="EXPLORE"))
    hunt_seq.add_child(build_move_subtree(ros_node, ai_target_id))
    hunt_seq.add_child(py_trees.behaviours.Failure("Loop_Hunt"))
    mode_selector.add_child(hunt_seq)

    loop = KeepRunningUntilSuccess("Loop_Explore_Exit", child=mode_selector)

    grid_phase = py_trees.composites.Sequence("Vao_Luoi_Va_Lap", memory=True)
    grid_phase.add_child(build_move_subtree(ros_node, ai_target_id))
    grid_phase.add_child(loop)
    grid_phase.add_child(exit_seq)

    main.add_child(grid_phase)
    main.add_child(place_box)

    main.add_child(py_trees.behaviours.Running("IDLE_MODE_ACTIVATED"))
    return main

def create_tree(ros_node, field_color='red'):
    cfg = FIELD_CONFIGS.get(field_color, FIELD_CONFIGS['red'])
    lat           = cfg['lat']           # +1.0 = red, -1.0 = blue
    post_ramp_yaw = cfg['post_ramp_yaw'] # -90° red / +90° blue
    side          = cfg['tool_arm_side'] # 'left' hoặc 'right'

    ai_target_id           = cfg['ai_target_id']
    r1_id                  = cfg['r1_id']
    entrance_box_coords    = cfg['entrance_box_coords']
    entrance_default_target = cfg['entrance_default_target']

    _seed_blackboard()
    start_reader_thread()  # Bắt thread đọc UART từ Jetson, ghi dữ liệu vào blackboard

    # Idiom Topics2BB: Parallel chạy song song nhánh subscribe ROS và nhánh logic chính.
    # SuccessOnSelected → root SUCCESS khi Main_Mission xong; Topics2BB cứ chạy nền.
    root = py_trees.composites.Parallel(
        name="Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[],  # sẽ điền sau khi tạo main_mission để dùng đúng instance
            synchronise=False,
        ),
    )

    topics2bb = _build_topics_to_bb(ros_node)
    main_mission = _build_main_mission(ros_node, lat, post_ramp_yaw, side, ai_target_id, r1_id,
                                       entrance_box_coords, entrance_default_target)
    bb_logger = BlackboardLoggerBehavior("BB_Logger", ros_node, log_every_n=30)

    root.add_children([topics2bb, main_mission, bb_logger])
    root.policy = py_trees.common.ParallelPolicy.SuccessOnSelected(
        children=[main_mission],
        synchronise=False,
    )
    return root
