#!/usr/bin/env python3
import operator
import py_trees
from r2_bt.behaviors.hardware import (
    ArmSequenceBTNode, MoveArmBehavior, RosWaitBehavior,
    WallAlignmentBehavior, FollowTargetBehavior,
)
from r2_bt.behaviors.navigation import GoToRelativePoseBehavior, TurnToTargetCellBehavior, MoveRelativeOdomBehavior
from r2_bt.behaviors.climb import HasElevationChangeCondition, DynamicClimbStepBehavior, IsClimbingUpCondition
from r2_bt.behaviors.grid_logic import (
    MarkAsVisitedAction, MarkAsObstacleAction, IncrementRealCountAction,
)
from r2_bt.config import NAV_PARAMS, PICK_PRESETS, PICK_STEPS_COUNT_1, PICK_STEPS_COUNT_2


def build_tool_assembly_sequence(ros_node, side='left'):
    """Chuỗi lấy và lắp ráp vũ khí qua ESP32 arm server.
    side: 'left' hoặc 'right' — lấy từ FIELD_CONFIGS['tool_arm_side'].
    """
    seq = py_trees.composites.Sequence(name="Quy_Trinh_Lay_Dung_Cu", memory=True)
    seq.add_children([
        # ArmSequenceBTNode("Approach_J4",     ros_node, f"approach_j4_{side}",   duration=1.0),
        ArmSequenceBTNode("Approach_J3",     ros_node, f"approach_j3_{side}",   duration=2.0),
        ArmSequenceBTNode("Grasp_Tool",      ros_node, f"grasp_{side}",         duration=1.0),
        ArmSequenceBTNode("Move_Back_J4",    ros_node, f"move_back_J4_{side}",  duration=1.0),
        ArmSequenceBTNode("Move_Back_J1",    ros_node, f"move_back_J1_{side}",  duration=1.0),
        ArmSequenceBTNode("Assemble_Action", ros_node, "assemble_sequence",     duration=20.0),
    ])
    return seq


def _build_arm_seq(ros_node, preset_key, steps):
    preset = PICK_PRESETS[preset_key]
    seq = py_trees.composites.Sequence(name=f"ArmSeq_{preset_key}", memory=True)
    for step_name, key, wait_sec in steps:
        seq.add_child(MoveArmBehavior(f"Arm_{step_name}", ros_node, target_pose=preset[key]))
        if wait_sec > 0.0:
            seq.add_child(RosWaitBehavior(f"Wait_{step_name}", ros_node, delay_sec=wait_sec))
    return seq


def build_pick_and_place_sequence(ros_node, mode='high'):
    """Chuỗi đặt hộp KFS lên giá Tic-Tac-Toe.
    mode='low'  — robot ở bậc thấp hơn.
    mode='high' — robot ở bậc cao hơn.
    Khi real_box_count >= 2 dùng preset _2 (arm1 nâng 500mm) để tránh hộp đặt trước.
    """
    outer = py_trees.composites.Selector(f"Place_{mode.upper()}", memory=False)

    branch_2 = py_trees.composites.Sequence(f"Place_{mode}_Count2", memory=True)
    branch_2.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="Real_GE_2?",
        check=py_trees.common.ComparisonExpression(
            variable="real_box_count", value=1, operator=operator.ge,
        ),
    ))
    branch_2.add_child(_build_arm_seq(ros_node, f"{mode}_2", PICK_STEPS_COUNT_2))

    outer.add_children([branch_2, _build_arm_seq(ros_node, f"{mode}_1", PICK_STEPS_COUNT_1)])
    return outer


def build_pick_and_place_sequence_dynamic(ros_node):
    """Chọn mode high/low tự động lúc runtime dựa trên chiều cao.
    h(target_cell) > h(robot_pos) → mode='high', ngược lại → mode='low'.
    """
    selector = py_trees.composites.Selector("Pick_Dynamic_Mode", memory=False)

    high_branch = py_trees.composites.Sequence("Pick_If_High", memory=True)
    high_branch.add_child(IsClimbingUpCondition("Is_Target_Higher", ros_node))
    high_branch.add_child(build_pick_and_place_sequence(ros_node, mode='high'))

    selector.add_children([
        high_branch,
        build_pick_and_place_sequence(ros_node, mode='low'),
    ])
    return selector


def build_move_subtree(ros_node, ai_target_id='class5'):
    """Subtree xử lý một bước vào ô lưới: quay hướng → vision → leo/di chuyển."""
    subtree = py_trees.composites.Sequence("Process_And_Enter_Cell", memory=True)

    # subtree.add_child(GoToRelativePoseBehavior("Lui_Truoc_Khi_Xoay", ros_node, dx=-0.1, dy=0.0, target_yaw_deg=0.0))
    subtree.add_child(TurnToTargetCellBehavior("Turn_To_Face_Cell", ros_node))

    subtree.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_1",
        py_trees.decorators.Timeout(
            "Align_1_Timeout",
            WallAlignmentBehavior(
                "Align_Before_Vision", ros_node,
                window_degrees=NAV_PARAMS['wall_window_deg'],
                goal_distance=NAV_PARAMS['wall_dist_vision'],
            ),
            duration=NAV_PARAMS['align_timeout_sec'],
        ),
    ))

    subtree.add_child(py_trees.decorators.FailureIsSuccess(
        "Run_AI",
        FollowTargetBehavior(
            "Run_AI_ID1", ros_node,
            target_id=ai_target_id,
            desired_distance_mm=NAV_PARAMS['follow_dist_mm'],
        ),
    ))

    box_selector = py_trees.composites.Selector("Box_Logic", memory=False)

    real_seq = py_trees.composites.Sequence("Hunt_REAL", memory=True)
    real_seq.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="Is_Real?",
        check=py_trees.common.ComparisonExpression(
            variable="latest_box_detection",
            value="REAL",
            operator=operator.eq,
        ),
    ))
    real_seq.add_child(build_pick_and_place_sequence_dynamic(ros_node))
    real_seq.add_child(py_trees.behaviours.SetBlackboardVariable(
        name="Set_Just_Picked",
        variable_name="just_picked",
        variable_value="yes",
        overwrite=True,
    ))
    real_seq.add_child(IncrementRealCountAction("Add_Score"))
    real_seq.add_child(MarkAsVisitedAction("Mark_Accessible"))

    fake_seq = py_trees.composites.Sequence("Hunt_FAKE", memory=True)
    fake_seq.add_child(py_trees.behaviours.CheckBlackboardVariableValue(
        name="Is_Fake?",
        check=py_trees.common.ComparisonExpression(
            variable="latest_box_detection",
            value="FAKE",
            operator=operator.eq,
        ),
    ))
    fake_seq.add_child(MarkAsObstacleAction("Mark_Obstacle"))
    fake_seq.add_child(py_trees.behaviours.Failure("Block_Route"))

    empty_seq = py_trees.composites.Sequence("Hunt_EMPTY", memory=True)
    empty_seq.add_child(MarkAsVisitedAction("Mark_Accessible_Empty"))

    box_selector.add_children([real_seq, fake_seq, empty_seq])
    subtree.add_child(box_selector)

    # climb_check = py_trees.composites.Sequence("Align_If_Elevation_Change", memory=True)
    # climb_check.add_child(HasElevationChangeCondition("Check_Elevation", ros_node))
    # climb_check.add_child(WallAlignmentBehavior(
    #     "Align_Before_Climb", ros_node,
    #     window_degrees=NAV_PARAMS['wall_window_deg'],
    #     goal_distance=NAV_PARAMS['wall_dist_climb'],
    # ))
    # subtree.add_child(py_trees.decorators.FailureIsSuccess("Ignore_Align_2", climb_check))

    subtree.add_child(DynamicClimbStepBehavior("Auto_Climb", ros_node))
    subtree.add_child(MoveRelativeOdomBehavior(
        "Move_Into_Cell", ros_node,
        climb_dist=NAV_PARAMS['climb_dist'],
        flat_dist=NAV_PARAMS['flat_dist'],
    ))
    subtree.add_child(py_trees.behaviours.SetBlackboardVariable(
        name="Reset_Just_Picked",
        variable_name="just_picked",
        variable_value="no",
        overwrite=True,
    ))

    return subtree
