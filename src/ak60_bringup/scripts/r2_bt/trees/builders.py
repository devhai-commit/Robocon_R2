#!/usr/bin/env python3
import operator
import py_trees
from r2_bt.behaviors.hardware import (
    ArmSequenceBTNode, MoveArmBehavior, RosWaitBehavior,
    WallAlignmentBehavior, FollowTargetBehavior,
)
from r2_bt.behaviors.navigation import TurnToTargetCellBehavior, MoveRelativeOdomBehavior
from r2_bt.behaviors.climb import IsClimbingUpCondition, HasElevationChangeCondition, DynamicClimbStepBehavior
from r2_bt.behaviors.grid_logic import (
    CheckBlackboardValue, MarkAsVisitedAction,
    MarkAsObstacleAction, IncrementRealCountAction,
)


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


_PICK_PRESETS = {
    # Robot ở bậc THẤP hơn — gắp lên (arm2 nhỏ, tay hướng về phía trước/lên)
    'low': {
        "home":      [ 0.0,  -5.0,  0.0,  70.0],
        "pre_grasp": [ 0.0,  70.0, -0.5,  70.0],
        "extend":    [ 0.0,  70.0, -0.5,  70.0],
        "close":     [ 0.0,  70.0, -0.5, -28.0],
        "lift":      [ 0.0,   0.0,  0.0, -28.0],
        "retract":   [ 0.0,   0.0,  0.0,  70.0],
    },
    # Robot ở bậc CAO hơn — gắp xuống (arm2 lớn hơn, arm3 âm hơn để cúi xuống)
    # TODO: tune các giá trị này bằng teleop_test_arm.py
    'high': {
        "home":      [ 300.0,  -5.0,   0.0,  70.0],
        "pre_grasp": [ 300.0,  55.0,   0.0,  70.0],
        "extend":    [ 300.0,  55.0,   0.0,  70.0],
        "close":     [ 300.0,  55.0,   0.0, -28.0],
        "lift":      [ 300.0,   0.0,   0.0, -28.0],
        "retract":   [ 300.0,   0.0,   0.0,  70.0],
    },

    'same': { # BỔ SUNG: Robot và hộp Cùng Mặt Phẳng
        "home":      [ 150.0,  -5.0,   0.0,  70.0],
        "pre_grasp": [ 150.0,  60.0,   0.0,  70.0],
        "extend":    [ 150.0,  60.0,   0.0,  70.0],
        "close":     [ 150.0,  60.0,   0.0, -28.0],
        "lift":      [ 150.0,   0.0,   0.0, -28.0],
        "retract":   [ 150.0,   0.0,   0.0,  70.0],
    }
}

_PICK_STEPS = [
    ("Home_1",    "home",      1.0),
    ("Pre_Grasp", "pre_grasp", 1.5),
    ("Extend",    "extend",    1.0),
    ("Close",     "close",     1.0),
    ("Lift",      "lift",      1.5),
    ("Retract",   "retract",   1.5),
    ("Home_2",    "home",      0.0),
]


def build_pick_and_place_sequence(ros_node, mode='low'):
    """Chuỗi gắp hộp KFS bằng MoveArm (4-DOF).
    mode='low'  — robot ở bậc thấp hơn, gắp lên.
    mode='high' — robot ở bậc cao hơn, gắp xuống.
    mode='same' - Robot và hộp Cùng Mặt Phẳng
    """
    presets = _PICK_PRESETS[mode]
    seq = py_trees.composites.Sequence(name=f"Chuoi_Gap_Hop_{mode.upper()}", memory=True)
    for step_name, preset_key, wait_sec in _PICK_STEPS:
        seq.add_child(MoveArmBehavior(f"Arm_{step_name}", ros_node, target_pose=presets[preset_key]))
        if wait_sec > 0.0:
            seq.add_child(RosWaitBehavior(f"Wait_{step_name}", ros_node, delay_sec=wait_sec))
    return seq


def build_place_sequence_1(ros_node):
    seq = py_trees.composites.Sequence(name="Place_Sequence_1", memory=True)
    seq.add_child(MoveArmBehavior("Arm: Dua hop 1 ra truoc", ros_node, target_pose=[400.0, 0.0, 90.0, 0.0]))
    seq.add_child(RosWaitBehavior("Wait", ros_node, delay_sec=1.0))
    return seq

def build_place_sequence_2(ros_node):
    seq = py_trees.composites.Sequence(name="Place_Sequence_2", memory=True)
    seq.add_child(MoveArmBehavior("Arm: Dua hop 2 ra truoc", ros_node, target_pose=[400.0, 0.0, 90.0, 0.0]))
    seq.add_child(RosWaitBehavior("Wait", ros_node, delay_sec=1.0))
    return seq


def build_move_subtree(ros_node):
    subtree = py_trees.composites.Sequence("Process_And_Enter_Cell", memory=True)
    subtree.add_child(TurnToTargetCellBehavior("Turn_To_Face_Cell", ros_node))

    subtree.add_child(py_trees.decorators.FailureIsSuccess(
        "Ignore_Align_1",
        WallAlignmentBehavior("Align_Before_Vision", ros_node, window_degrees=20.0, goal_distance=0.3),
    ))
# ============================ them config target ID ============================================
    subtree.add_child(py_trees.decorators.FailureIsSuccess(
        "Run_AI",
        FollowTargetBehavior("Run_AI_ID1", ros_node, target_id=1, desired_distance_mm=250.0),
    ))
# ==================================================================================================
    box_selector = py_trees.composites.Selector("Box_Logic", memory=False)

    # Lập logic chẻ nhánh 3 Case cao độ cho Hộp REAL
    pick_selector = py_trees.composites.Selector("Pick_By_Elevation", memory=False)
    
    pick_low_seq = py_trees.composites.Sequence("Pick_From_Low", memory=True)
    pick_low_seq.add_child(IsClimbingUpCondition("Is_Going_Up", ros_node))
    pick_low_seq.add_child(build_pick_and_place_sequence(ros_node, mode='low'))
    
    # Check nếu độ cao KHÔNG thay đổi -> Dùng preset 'same'
    pick_high_seq = py_trees.composites.Sequence("Pick_Same_Level", memory=True)
    pick_high_seq.add_child(py_trees.decorators.Inverter(
        "Not_Change_Elevation", 
        HasElevationChangeCondition("Check_Elevation_Change", ros_node)
    ))
    pick_high_seq.add_child(build_pick_and_place_sequence(ros_node, mode='high'))
    
    pick_selector.add_children([pick_low_seq, pick_high_seq, build_pick_and_place_sequence(ros_node, mode='high')])

    real_seq = py_trees.composites.Sequence("Hunt_REAL", memory=True)
    real_seq.add_child(CheckBlackboardValue("Is_Real?", "latest_box_detection", "REAL", operator.eq))
    real_seq.add_child(pick_selector)
    real_seq.add_child(IncrementRealCountAction("Add_Score"))
    real_seq.add_child(MarkAsVisitedAction("Mark_Accessible"))

    fake_seq = py_trees.composites.Sequence("Hunt_FAKE", memory=True)
    fake_seq.add_child(CheckBlackboardValue("Is_Fake?", "latest_box_detection", "FAKE", operator.eq))
    fake_seq.add_child(MarkAsObstacleAction("Mark_Obstacle"))
    fake_seq.add_child(py_trees.behaviours.Failure("Block_Route"))

    # [FIX CỰC KỲ QUAN TRỌNG]: Đã bọc thêm node Check EMPTY để không đâm bậy!
    empty_seq = py_trees.composites.Sequence("Hunt_EMPTY", memory=True)
    empty_seq.add_child(CheckBlackboardValue("Is_Empty?", "latest_box_detection", "EMPTY", operator.eq))
    empty_seq.add_child(MarkAsVisitedAction("Mark_Accessible_Empty"))

    box_selector.add_children([real_seq, fake_seq, empty_seq])
    subtree.add_child(box_selector)

    climb_check = py_trees.composites.Sequence("Align_If_Elevation_Change", memory=True)
    climb_check.add_child(HasElevationChangeCondition("Check_Elevation", ros_node))
    climb_check.add_child(WallAlignmentBehavior("Align_Before_Climb", ros_node, window_degrees=20.0, goal_distance=0.3))
    subtree.add_child(py_trees.decorators.FailureIsSuccess("Ignore_Align_2", climb_check))

    subtree.add_child(DynamicClimbStepBehavior("Auto_Climb", ros_node))
    subtree.add_child(MoveRelativeOdomBehavior("Move_Into_Cell", ros_node, climb_dist=0.3, flat_dist=1.2))

    return subtree