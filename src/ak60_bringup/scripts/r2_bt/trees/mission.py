#!/usr/bin/env python3
import operator
import py_trees
import py_trees_ros
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from r2_bt.behaviors.navigation import GoToRelativePoseBehavior
from r2_bt.behaviors.climb import ClimbStepBehavior
from r2_bt.behaviors.hardware import FollowTargetBehavior, WallAlignmentBehavior, ArmSequenceBTNode, MoveArmBehavior
from r2_bt.behaviors.grid_logic import (
    DecideNextGridCellAction, IsAtExitCellCondition,
    CheckBlackboardValue, KeepRunningUntilSuccess, WaitForStartSignalBehavior,
)
from r2_bt.trees.builders import build_tool_assembly_sequence, build_move_subtree
from r2_bt.config import FIELD_CONFIGS


def _seed_blackboard():
    bb_writer = py_trees.blackboard.Client(name="Mission_Init")
    for key in ("real_box_count", "robot_pos", "visit_path", "grid_status"):
        bb_writer.register_key(key=key, access=py_trees.common.Access.WRITE)

    bb_writer.real_box_count = 0
    bb_writer.robot_pos = (2, 0)
    bb_writer.visit_path = [(2, 0)]
    bb_writer.grid_status = {
        (c, r): {'scanned': False, 'state': 'accessible', 'visited': False}
        for c in range(1, 4) for r in range(1, 5)
    }


def _build_topics_to_bb():
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
    gui_signal_to_bb = py_trees_ros.subscribers.ToBlackboard(
        name="GuiSignal2BB",
        topic_name="/gui_start_signal",
        topic_type=String,
        qos_profile=gui_qos,
        blackboard_variables={"gui_start_raw": "data"},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    )
    topics2bb.add_child(gui_signal_to_bb)
    return topics2bb


def _build_main_mission(ros_node, lat, post_ramp_yaw, side):
    main = py_trees.composites.Sequence("Main_Mission", memory=True)

    # --- PHASE 0: GẬP TAY AN TOÀN & CHỜ LỆNH START TỪ MÀN HÌNH ---
    pre_init_seq = py_trees.composites.Sequence("Phase_0_Standby", memory=True)
    pre_init_seq.add_child(ArmSequenceBTNode("Tool_Arm_Home", ros_node, f"home_pose_{side}", duration=2.0))
    pre_init_seq.add_child(MoveArmBehavior("Box_Arm_Home", ros_node, target_pose=[325.0, 0.0, 90.0, 50.0]))
    pre_init_seq.add_child(WaitForStartSignalBehavior("Wait_For_GUI_Strategy", ros_node))
    main.add_child(pre_init_seq)

    # # --- Phase 1: Lấy dụng cụ và tiến ra cửa lưới ---
    init_seq = py_trees.composites.Sequence("Khoi_Dong_Va_Lay_Dung_Cu", memory=True)
    # init_seq.add_child(ArmSequenceBTNode("Tool_Arm_Approach", ros_node, f"approach_j4_{side}", duration=1.0))
    # init_seq.add_child(GoToRelativePoseBehavior("Tien_Lay_Dung_Cu", ros_node, dx=0.22, dy=lat * 0.89, target_yaw_deg=0.0))
    # init_seq.add_child(build_tool_assembly_sequence(ros_node, side=side))
    # init_seq.add_child(GoToRelativePoseBehavior("Tien_Ra_Cua_Cell", ros_node, dx=1.6, dy=-lat * 2.5, target_yaw_deg=0.0))
    init_seq.add_child(py_trees.behaviours.SetBlackboardVariable(
        name="Set_Target_2_1",
        variable_name="target_cell",
        variable_value=(2, 1),
        overwrite=True,
    ))
    main.add_child(init_seq)

    main.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_Exit",
        WallAlignmentBehavior("Align_Cua_Cell", ros_node, window_degrees=20.0, goal_distance=0.5, timeout_sec = 10.0),))
    main.add_child(py_trees.decorators.FailureIsSuccess(
        "Run_AI",
        FollowTargetBehavior("Run_AI_ID1", ros_node, target_id=5.0, desired_distance_mm=500.0)))
    
    main.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_Enter_Cell",
        WallAlignmentBehavior("Align_after_AI", ros_node, window_degrees=20.0, goal_distance=0.1, timeout_sec = 10.0),))

    # --- Phase 3: Thoát khỏi sàn ---
    # exit_seq = py_trees.composites.Sequence("Chuoi_Thoat_Khoi_San", memory=True)
    # exit_seq.add_child(GoToRelativePoseBehavior("Thoat_Tien_025",     ros_node, dx=0.25, dy=0.0, target_yaw_deg=0.0))
    # exit_seq.add_child(ClimbStepBehavior("Thoat_Xuong_Bac",           ros_node, velocity=0.1))
    # exit_seq.add_child(GoToRelativePoseBehavior("Thoat_Tien_025_Sau", ros_node, dx=0.25, dy=0.0, target_yaw_deg=0.0))
    # exit_seq.add_child(py_trees.decorators.FailureIsSuccess(
    #     "Ignore_Align_Exit",
    #     WallAlignmentBehavior("Thoat_Align_1", ros_node, window_degrees=60.0, goal_distance=0.4),
    # ))

    # branch_selector = py_trees.composites.Selector("Chon_Huong_Thoat", memory=False)

    # seq_3_4 = py_trees.composites.Sequence("Thoat_Tu_3_4", memory=True)
    # seq_3_4.add_child(CheckBlackboardValue("La_o_3_4?", "robot_pos", (3, 4), operator.eq))
    # seq_3_4.add_child(GoToRelativePoseBehavior("Di_Ve_Doc_1_3m", ros_node, dx=0.0, dy=lat * 1.3, target_yaw_deg=0.0))

    # seq_1_4 = py_trees.composites.Sequence("Thoat_Tu_1_4", memory=True)
    # seq_1_4.add_child(CheckBlackboardValue("La_o_1_4?", "robot_pos", (1, 4), operator.eq))
    # seq_1_4.add_child(GoToRelativePoseBehavior("Di_Ve_Doc_3_7m", ros_node, dx=0.0, dy=lat * 3.7, target_yaw_deg=0.0))

    # branch_selector.add_children([seq_3_4, seq_1_4])
    # exit_seq.add_child(branch_selector)

    # exit_seq.add_child(py_trees.decorators.FailureIsSuccess(
    #     "Ignore_Align_Ramp",
    #     WallAlignmentBehavior("Thoat_Align_2", ros_node, window_degrees=60.0, goal_distance=0.4),
    # ))
    # exit_seq.add_child(ClimbStepBehavior("Thoat_Len_Doc",        ros_node, velocity=0.5))
    # exit_seq.add_child(GoToRelativePoseBehavior("Quay_Ve_Arena", ros_node, dx=0.0, dy=0.0, target_yaw_deg=post_ramp_yaw))
    # exit_seq.add_child(GoToRelativePoseBehavior("Tien_4m",       ros_node, dx=4.0, dy=0.0, target_yaw_deg=post_ramp_yaw))

    # --- Phase 2: Loop điều hướng lưới ---
    
    # mode_selector = py_trees.composites.Selector("Mode_Selector", memory=False)

    # exit_goal_seq = py_trees.composites.Sequence("Mode_EXIT_Goal", memory=True)
    # exit_goal_seq.add_child(CheckBlackboardValue("Real_GE_2?", "real_box_count", 2, operator.ge))
    # exit_goal_seq.add_child(IsAtExitCellCondition("At_Exit_Cell?"))
    # exit_goal_seq.add_child(exit_seq)
    # exit_goal_seq.add_child(py_trees.behaviours.Success("Mission_Complete"))
    # mode_selector.add_child(exit_goal_seq)

    # find_exit_seq = py_trees.composites.Sequence("Mode_FIND_Exit", memory=True)
    # find_exit_seq.add_child(CheckBlackboardValue("Real_GE_2_Find?", "real_box_count", 2, operator.ge))
    # find_exit_seq.add_child(DecideNextGridCellAction("Route_Exit", ros_node, mode="EXIT"))
    # find_exit_seq.add_child(build_move_subtree(ros_node))
    # mode_selector.add_child(find_exit_seq)

    # hunt_seq = py_trees.composites.Sequence("Mode_HUNT", memory=True)
    # hunt_seq.add_child(DecideNextGridCellAction("Route_Explore", ros_node, mode="EXPLORE"))
    # hunt_seq.add_child(build_move_subtree(ros_node))
    # mode_selector.add_child(hunt_seq)

    # loop = KeepRunningUntilSuccess("Loop_Explore_Exit", child=mode_selector)

    # grid_phase = py_trees.composites.Sequence("Vao_Luoi_Va_Lap", memory=True)
    # grid_phase.add_child(build_move_subtree(ros_node))
    # grid_phase.add_child(loop)

    # main.add_child(grid_phase)

    main.add_child(py_trees.behaviours.Running("IDLE_MODE_ACTIVATED"))
    return main


def create_tree(ros_node, field_color='red'):
    cfg = FIELD_CONFIGS.get(field_color, FIELD_CONFIGS['red'])
    lat           = cfg['lat']           # +1.0 = red, -1.0 = blue
    post_ramp_yaw = cfg['post_ramp_yaw'] # -90° red / +90° blue
    side          = cfg['tool_arm_side'] # 'left' hoặc 'right'

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

    topics2bb = _build_topics_to_bb()
    main_mission = _build_main_mission(ros_node, lat, post_ramp_yaw, side)

    root.add_children([topics2bb, main_mission])
    root.policy = py_trees.common.ParallelPolicy.SuccessOnSelected(
        children=[main_mission],
        synchronise=False,
    )
    return root
