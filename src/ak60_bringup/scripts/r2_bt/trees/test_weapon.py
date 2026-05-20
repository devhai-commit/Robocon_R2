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
    pre_init_seq.add_child(ArmSequenceBTNode("Tool_Arm_Home", ros_node, f"home_pose_{side}", duration=2.0))
    pre_init_seq.add_child(MoveArmBehavior("Box_Arm_Home", ros_node, target_pose=[325.0, 0.0, 0.0, 0.0]))
    pre_init_seq.add_child(WaitForStartSignalBehavior("Wait_For_GUI_Strategy", ros_node))
    pre_init_seq.add_child(ResetPoseBehavior("Reset_Pose_At_Start", ros_node, x=0.0, y=0.0, yaw_deg=0.0))
    pre_init_seq.add_child(ArmSequenceBTNode("Tool_Arm_Home2", ros_node, f"approach_j4_{side}", duration=1.0))
    main.add_child(pre_init_seq)

    # # --- Phase 1: Lấy dụng cụ và tiến ra cửa lưới ---
    init_seq = py_trees.composites.Sequence("Khoi_Dong_Va_Lay_Dung_Cu", memory=True)
    init_seq.add_child(GoToRelativePoseBehavior("Tien_Lay_Dung_Cu", ros_node, dx=0.4, dy=lat*0.55, target_yaw_deg=0.0))
    init_seq.add_child(py_trees.decorators.FailureIsSuccess(
        "Try_K230_Align",
        K230AlignBehavior("K230_Align_Before_Tool", ros_node),
    ))
    init_seq.add_child(GoToRelativePoseBehavior("Tien_Lay_Dung_Cu", ros_node, dx=0.0, dy=lat*0.2, target_yaw_deg=0.0))
    init_seq.add_child(build_tool_assembly_sequence(ros_node, side=side))
    main.add_child(init_seq)

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
