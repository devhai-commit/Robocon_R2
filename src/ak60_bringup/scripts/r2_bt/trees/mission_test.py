#!/usr/bin/env python3
import operator
from pdb import main
import py_trees
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from r2_bt.behaviors.blackboard_ops import ROSTopicToBB, BlackboardLoggerBehavior
from r2_bt.behaviors.navigation import GoToRelativePoseBehavior
from r2_bt.behaviors.climb import ClimbStepBehavior
from r2_bt.behaviors.hardware import FollowTargetBehavior, RosWaitBehavior, WallAlignmentBehavior, ArmSequenceBTNode, MoveArmBehavior
from r2_bt.behaviors.grid_logic import (
    DecideNextGridCellAction, IsAtExitCellCondition,
    KeepRunningUntilSuccess, WaitForStartSignalBehavior,
    IncrementRealCountAction,
)
from r2_bt.trees.builders import build_tool_assembly_sequence, build_move_subtree, build_pick_and_place_sequence
from r2_bt.config import FIELD_CONFIGS


_SEED_KEYS = (
    "real_box_count",
    "robot_pos",
    "visit_path",
    "grid_status",
    "target_cell",
    "just_climbed",
    "current_arm1_z",
    "priority_col",
    "target_boxes_list",
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
    seed.current_arm1_z = 125.0
    seed.priority_col = 2
    seed.target_boxes_list = []


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


def _build_main_mission(ros_node, lat, post_ramp_yaw, side, ai_target_id):
    main = py_trees.composites.Sequence("Main_Mission", memory=True)

    # --- PHASE 0: GẬP TAY AN TOÀN & CHỜ LỆNH START TỪ MÀN HÌNH ---
    pre_init_seq = py_trees.composites.Sequence("Phase_0_Standby", memory=True)
    pre_init_seq.add_child(ArmSequenceBTNode("Tool_Arm_Home", ros_node, f"home_pose_{side}", duration=2.0))
    pre_init_seq.add_child(MoveArmBehavior("Box_Arm_Home", ros_node, target_pose=[250.0, 0.0, 0.0, 0.0]))
    pre_init_seq.add_child(WaitForStartSignalBehavior("Wait_For_GUI_Strategy", ros_node))
    main.add_child(pre_init_seq)

    # # --- Phase 1: Lấy dụng cụ và tiến ra cửa lưới ---
    init_seq = py_trees.composites.Sequence("Khoi_Dong_Va_Lay_Dung_Cu", memory=True)
    init_seq.add_child(ArmSequenceBTNode("Tool_Arm_Approach", ros_node, f"approach_j4_{side}", duration=1.0))
    init_seq.add_child(GoToRelativePoseBehavior("Tien_Lay_Dung_Cu", ros_node, dx=0.18, dy=lat * 1.0, target_yaw_deg=0.0))
    init_seq.add_child(build_tool_assembly_sequence(ros_node, side=side))
    init_seq.add_child(GoToRelativePoseBehavior("Tien_Ra_Cua_Cell", ros_node, dx=1.9, dy=-lat * 2.5, target_yaw_deg=0.0))
    init_seq.add_child(py_trees.behaviours.SetBlackboardVariable(
        name="Set_Target_2_1",
        variable_name="target_cell",
        variable_value=(2, 1),
        overwrite=True,
    ))
    main.add_child(init_seq)

# ==== Lay hop bac 1 =========================
    main.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_Exit",
        WallAlignmentBehavior("Align_Cua_Cell", ros_node, window_degrees=20.0, goal_distance=0.0, timeout_sec=10.0),))
    main.add_child(py_trees.decorators.FailureIsSuccess(
        "Run_AI",
        FollowTargetBehavior("Run_AI_ID1", ros_node, target_id="class5", desired_distance_mm=335.0)))
    
    # main.add_child(GoToRelativePoseBehavior("Tien_0.3m_Before_Pick", ros_node, dx=0.3, dy=0.0, target_yaw_deg=0.0))
    main.add_child(build_pick_and_place_sequence(ros_node))  
    # main.add_child(RosWaitBehavior("Roswait_2s", ros_node, duration_sec=2.0))
    main.add_child(IncrementRealCountAction("Add_Score_Initial"))

# ==== Leen bac 1 , lay hop bac 2 =================================

    main.add_child(ClimbStepBehavior("Len_Bac_Dau", ros_node, velocity=0.3))
    main.add_child(GoToRelativePoseBehavior("Tien_0.3m", ros_node, dx=0.3, dy=0.0, target_yaw_deg=0.0))
    # main.add_child(py_trees.decorators.FailureIsSuccess(
    #     "Ignore_Align_Exit",
    #     WallAlignmentBehavior("Align_Bac_1", ros_node, window_degrees=20.0, goal_distance=0.5, timeout_sec=10.0),))
    main.add_child(py_trees.decorators.FailureIsSuccess(
        "Run_AI",
        FollowTargetBehavior("Run_AI2", ros_node, target_id="class5", desired_distance_mm=600.0)))
    main.add_child(GoToRelativePoseBehavior("Tien_0.3m_Before_Pick_2", ros_node, dx=0.3, dy=0.0, target_yaw_deg=0.0))
    main.add_child(build_pick_and_place_sequence(ros_node)) 

# ===== Leen bac 2 =============================
    main.add_child(ClimbStepBehavior("Len_Bac_2", ros_node, velocity=0.3))
    main.add_child(GoToRelativePoseBehavior("Tien_0.3m_2", ros_node, dx=0.6, dy=0.0, target_yaw_deg=0.0))
# ===== Leen bac 3 =============================    
    main.add_child(ClimbStepBehavior("Xuong_Bac_3", ros_node, velocity=0.3))
    main.add_child(GoToRelativePoseBehavior("Tien_0.3m_3", ros_node, dx=0.6, dy=0.0, target_yaw_deg=0.0))
# ===== xuong bac 4 =============================
    main.add_child(ClimbStepBehavior("Xuong_Bac_4", ros_node, velocity=0.1))
    main.add_child(GoToRelativePoseBehavior("Tien_0.3m_4", ros_node, dx=0.3, dy=0.0, target_yaw_deg=90.0)) 
# ===== Xuong bac 5 =============================
    main.add_child(ClimbStepBehavior("Xuong_Bac_5", ros_node, velocity=0.1))
    main.add_child(GoToRelativePoseBehavior("Tien_0.3m_5", ros_node, dx=0.3, dy=0.0, target_yaw_deg=0.0))
# ===== Xuong bac 6 =============================
    main.add_child(ClimbStepBehavior("Xuong_Bac_6", ros_node, velocity=0.1))
    main.add_child(GoToRelativePoseBehavior("Tien_0.3m_6", ros_node, dx=0.3, dy=0.0, target_yaw_deg=0.0))
# ===== Align bac 6 =============================
    main.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_Bac_6",
        WallAlignmentBehavior("Align_Bac_6", ros_node, window_degrees=20.0, goal_distance=0.0, timeout_sec=5.0),))    
# ===== Di chuyen ve doc 1.3m =============================           
    main.add_child(GoToRelativePoseBehavior("Di_Ve_Doc_1_0m", ros_node, dx=0.0, dy=-lat * 1.1, target_yaw_deg=0.0))  
# ===== Len doc =============================
    main.add_child(ClimbStepBehavior("Len_Doc", ros_node, velocity=0.6))
# ===== Quay ve arena =============================
    main.add_child(GoToRelativePoseBehavior("Quay_Ve_Arena", ros_node, dx=0.0, dy=0.0, target_yaw_deg=-105.0))
# ===== Tien 4m =============================
    main.add_child(GoToRelativePoseBehavior("Tien_4m", ros_node, dx=4.0, dy=0.0, target_yaw_deg=-90.0))
    # main.add_child(GoToRelativePoseBehavior("Tien_4m", ros_node, dx=0.0, dy=-3.0, target_yaw_deg=-90.0))
# ==== Align truoc khi vao arena =============================
    main.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_Before_Arena",
        WallAlignmentBehavior("Align_Before_AI", ros_node, window_degrees=20.0, goal_distance=1.4, timeout_sec=5.0),))

# ===== TRacking AI =============================
    main.add_child(py_trees.decorators.FailureIsSuccess(
        "Run_AI_Exit",
        FollowTargetBehavior("Run_AI_Exit", ros_node, target_id='class1', desired_distance_mm=500.0)))
# ===== Lay va dat hop 2 len gia =============================
    main.add_child(MoveArmBehavior("Move_Arm_pose_11", ros_node, target_pose=[590.0, 0.0, 90.0, 200.0]))
    main.add_child(MoveArmBehavior("Move_Arm_pose_12", ros_node, target_pose=[640.0, 0.0, 90.0, 30.0]))
    main.add_child(MoveArmBehavior("Move_Arm_pose_13", ros_node, target_pose=[780.0, 0.0, 90.0, 30.0]))
    main.add_child(MoveArmBehavior("Move_Arm_pose_14", ros_node, target_pose=[780.0, 0.0, -70.0, 30.0]))
    main.add_child(WallAlignmentBehavior("Align_Exit_Cell", ros_node, window_degrees=20.0, goal_distance=0.2, timeout_sec=10.0))
    main.add_child(MoveArmBehavior("Move_Arm_pose_15", ros_node, target_pose=[780.0, 0.0, -70.0, 200.0]))
    main.add_child(GoToRelativePoseBehavior("Tien_0.5m_Exit", ros_node, dx=-0.5, dy=0.0, target_yaw_deg=0.0))
    main.add_child(MoveArmBehavior("Move_Arm_pose_16", ros_node, target_pose=[290.0, 0.0, 90.0, 200.0]))

#===== Di chuyen den vi tri dat hop len gia =============================
    main.add_child(GoToRelativePoseBehavior("Tien_0.5m_Exit_2", ros_node, dx=0.0, dy=0.54, target_yaw_deg=0.0))
# ===== Lay va dat hop 1 len gia =============================
    main.add_child(MoveArmBehavior("Move_Arm_pose_21", ros_node, target_pose=[290.0, 0.0, 90.0, 30.0]))
    main.add_child(MoveArmBehavior("Move_Arm_pose_22", ros_node, target_pose=[780.0, 0.0, 90.0, 30.0]))
    main.add_child(MoveArmBehavior("Move_Arm_pose_23", ros_node, target_pose=[780.0, 0.0, -70.0, 30.0]))
    main.add_child(WallAlignmentBehavior("Align_Exit_Cell", ros_node, window_degrees=20.0, goal_distance=0.2, timeout_sec=10.0))
    main.add_child(MoveArmBehavior("Move_Arm_pose_24", ros_node, target_pose=[780.0, 0.0, -70.0, 200.0]))
    main.add_child(GoToRelativePoseBehavior("Tien_0.5m_Exit_3", ros_node, dx=-0.5, dy=0.0, target_yaw_deg=0.0))
    main.add_child(MoveArmBehavior("Move_Arm_pose_25", ros_node, target_pose=[290.0, 0.0, 90.0, 200.0]))
    main.add_child(py_trees.behaviours.Running("IDLE_MODE_ACTIVATED"))
    return main

def create_tree(ros_node, field_color='red'):
    cfg = FIELD_CONFIGS.get(field_color, FIELD_CONFIGS['red'])
    lat           = cfg['lat']           # +1.0 = red, -1.0 = blue
    post_ramp_yaw = cfg['post_ramp_yaw'] # -90° red / +90° blue
    side          = cfg['tool_arm_side'] # 'left' hoặc 'right'

    # AI target_id theo phia san: red → 4, blue → 5.
    ai_target_id = "class4" if field_color == 'red' else "class5"

    _seed_blackboard()

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
    main_mission = _build_main_mission(ros_node, lat, post_ramp_yaw, side, ai_target_id)
    bb_logger = BlackboardLoggerBehavior("BB_Logger", ros_node, log_every_n=30)

    root.add_children([topics2bb, main_mission, bb_logger])
    root.policy = py_trees.common.ParallelPolicy.SuccessOnSelected(
        children=[main_mission],
        synchronise=False,
    )
    return root
