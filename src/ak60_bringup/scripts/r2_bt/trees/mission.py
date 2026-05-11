#!/usr/bin/env python3
import operator
import py_trees
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from r2_bt.behaviors.blackboard_ops import ROSTopicToBB, BlackboardLoggerBehavior
from r2_bt.behaviors.navigation import GoToRelativePoseBehavior
from r2_bt.behaviors.climb import ClimbStepBehavior
from r2_bt.behaviors.hardware import FollowTargetBehavior, WallAlignmentBehavior, ArmSequenceBTNode, MoveArmBehavior
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
    "just_picked",
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
    seed.just_picked = "no"
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
    pre_init_seq.add_child(MoveArmBehavior("Box_Arm_Home", ros_node, target_pose=[325.0, 0.0, 0.0, 0.0]))
    pre_init_seq.add_child(WaitForStartSignalBehavior("Wait_For_GUI_Strategy", ros_node))
    main.add_child(pre_init_seq)

    # # --- Phase 1: Lấy dụng cụ và tiến ra cửa lưới ---
    init_seq = py_trees.composites.Sequence("Khoi_Dong_Va_Lay_Dung_Cu", memory=True)
    init_seq.add_child(ArmSequenceBTNode("Tool_Arm_Approach", ros_node, f"approach_j4_{side}", duration=1.0))
    init_seq.add_child(GoToRelativePoseBehavior("Tien_Lay_Dung_Cu", ros_node, dx=0.42, dy=lat * 0.89, target_yaw_deg=0.0))
    init_seq.add_child(build_tool_assembly_sequence(ros_node, side=side))
    init_seq.add_child(GoToRelativePoseBehavior("Tien_Ra_Cua_Cell", ros_node, dx=1.6, dy=-lat * 2.5, target_yaw_deg=0.0))
    init_seq.add_child(py_trees.behaviours.SetBlackboardVariable(
        name="Set_Target_2_1",
        variable_name="target_cell",
        variable_value=(2, 1),
        overwrite=True,
    ))
    main.add_child(init_seq)


    # main.add_child(py_trees.decorators.FailureIsSuccess(
    #     "Ignore_Align_Exit",
    #     WallAlignmentBehavior("Align_Cua_Cell", ros_node, window_degrees=20.0, goal_distance=0.5, timeout_sec=10.0),))
    # main.add_child(py_trees.decorators.FailureIsSuccess(
    #     "Run_AI",
    #     FollowTargetBehavior("Run_AI_ID1", ros_node, target_id="class5", desired_distance_mm=500.0)))

    # main.add_child(py_trees.decorators.FailureIsSuccess(
    #     "Ignore_Align_Enter_Cell",
    #     WallAlignmentBehavior("Align_after_AI", ros_node, window_degrees=20.0, goal_distance=0.1, timeout_sec=10.0),))

    # main.add_child(build_pick_and_place_sequence(ros_node, mode='high'))
    # main.add_child(IncrementRealCountAction("Add_Score_Initial"))

    # --- Phase 3: Thoát khỏi sàn ---

    exit_seq = py_trees.composites.Sequence("Chuoi_Thoat_Khoi_San", memory=True)
    exit_seq.add_child(GoToRelativePoseBehavior("Thoat_Tien_025",     ros_node, dx=0.25, dy=0.0, target_yaw_deg=0.0))
    exit_seq.add_child(ClimbStepBehavior("Thoat_Xuong_Bac",           ros_node, velocity=0.1))
    exit_seq.add_child(GoToRelativePoseBehavior("Thoat_Tien_025_Sau", ros_node, dx=0.25, dy=0.0, target_yaw_deg=0.0))
    exit_seq.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_Exit",
        WallAlignmentBehavior("Thoat_Align_1", ros_node, window_degrees=60.0, goal_distance=0.4),
    ))

    branch_selector = py_trees.composites.Selector("Chon_Huong_Thoat", memory=False)

    seq_3_4 = py_trees.composites.Sequence("Thoat_Tu_3_4", memory=True)
    seq_3_4.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="La_o_3_4?",
        check=py_trees.common.ComparisonExpression(
            variable="robot_pos", value=(3, 4), operator=operator.eq,
        ),
    ))
    seq_3_4.add_child(GoToRelativePoseBehavior("Di_Ve_Doc_1_3m", ros_node, dx=0.0, dy=lat * 1.3, target_yaw_deg=0.0))

    seq_1_4 = py_trees.composites.Sequence("Thoat_Tu_1_4", memory=True)
    seq_1_4.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="La_o_1_4?",
        check=py_trees.common.ComparisonExpression(
            variable="robot_pos", value=(1, 4), operator=operator.eq,
        ),
    ))
    seq_1_4.add_child(GoToRelativePoseBehavior("Di_Ve_Doc_3_7m", ros_node, dx=0.0, dy=lat * 3.7, target_yaw_deg=0.0))

    branch_selector.add_children([seq_3_4, seq_1_4])

    exit_seq.add_child(branch_selector)

    exit_seq.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_Ramp",
        WallAlignmentBehavior("Thoat_Align_2", ros_node, window_degrees=60.0, goal_distance=0.4),
    ))
    exit_seq.add_child(ClimbStepBehavior("Thoat_Len_Doc",        ros_node, velocity=0.5))
    exit_seq.add_child(GoToRelativePoseBehavior("Quay_Ve_Arena", ros_node, dx=0.0, dy=0.0, target_yaw_deg=post_ramp_yaw))
    exit_seq.add_child(GoToRelativePoseBehavior("Tien_4m",       ros_node, dx=4.0, dy=0.0, target_yaw_deg=post_ramp_yaw))


    # --- Phase 2: Loop điều hướng lưới ---

    mode_selector = py_trees.composites.Selector("Mode_Selector", memory=False)

    exit_goal_seq = py_trees.composites.Sequence("Mode_EXIT_Goal", memory=True)
    exit_goal_seq.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="Real_GE_2?",
        check=py_trees.common.ComparisonExpression(
            variable="real_box_count", value=2, operator=operator.ge,
        ),
    ))
    exit_goal_seq.add_child(IsAtExitCellCondition("At_Exit_Cell?"))
    exit_goal_seq.add_child(exit_seq)
    exit_goal_seq.add_child(py_trees.behaviours.Success("Mission_Complete"))
    mode_selector.add_child(exit_goal_seq)

    find_exit_seq = py_trees.composites.Sequence("Mode_FIND_Exit", memory=True)
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

    main.add_child(grid_phase)


    main.add_child(py_trees.behaviours.Running("IDLE_MODE_ACTIVATED"))
    return main


def create_tree(ros_node, field_color='red'):
    cfg = FIELD_CONFIGS.get(field_color, FIELD_CONFIGS['red'])
    lat           = cfg['lat']           # +1.0 = red, -1.0 = blue
    post_ramp_yaw = cfg['post_ramp_yaw'] # -90° red / +90° blue
    side          = cfg['tool_arm_side'] # 'left' hoặc 'right'

    ai_target_id  = cfg['ai_target_id']

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
