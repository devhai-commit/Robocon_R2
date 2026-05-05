#!/usr/bin/env python3
import py_trees
import operator

# Nhúng các node đã định nghĩa
from bt_nodes import *

def build_tool_assembly_sequence_right(ros_node):
    """Chuỗi gắp dụng cụ bằng ESP32 bên Phải (Theo kịch bản test thành công)"""
    seq = py_trees.composites.Sequence(name="Quy_Trinh_Lay_Dung_Cu", memory=True)
    seq.add_children([
        # ArmSequenceBTNode("Home_init", ros_node, "home_pose_right", duration=1.0),
        ArmSequenceBTNode("Approach_J4", ros_node, "approach_j4_right", duration=1.0),
        ArmSequenceBTNode("Approach_J3", ros_node,"approach_j3_right", duration=2.5),
        ArmSequenceBTNode("Grasp_Tool", ros_node, "grasp_right", duration=1.0),
        ArmSequenceBTNode("Move_Back_J4", ros_node, "move_back_J4_right", duration=1.0),
        ArmSequenceBTNode("Approach_J1", ros_node, "move_back_J1_right", duration=1.5),
        ArmSequenceBTNode("Assemble_Action", ros_node, "assemble_sequence", duration=20.0),
    ])
    return seq

def build_tool_assembly_sequence_left(ros_node):
    """Chuỗi gắp dụng cụ bằng ESP32 bên Phải (Theo kịch bản test thành công)"""
    seq = py_trees.composites.Sequence(name="Quy_Trinh_Lay_Dung_Cu", memory=True)
    seq.add_children([
        # ArmSequenceBTNode("Home_init", ros_node, "home_pose_left", duration=1.0),
        ArmSequenceBTNode("Approach_J4", ros_node, "approach_j4_left", duration=1.0),
        ArmSequenceBTNode("Approach_J3", ros_node,"approach_j3_left", duration=2.5),
        ArmSequenceBTNode("Grasp_Tool", ros_node, "grasp_left", duration=1.0),
        ArmSequenceBTNode("Move_Back_J4", ros_node, "move_back_J4_left", duration=1.0),
        ArmSequenceBTNode("Approach_J1", ros_node, "move_back_J1_left", duration=1.5),
        ArmSequenceBTNode("Assemble_Action", ros_node, "assemble_sequence", duration=20.0),
    ])
    return seq

def build_pick_and_place_sequence(ros_node):
    """Chuỗi Pick hộp dùng AK60 Arm tự động nội suy độ cao Z"""
    presets = {
        "home": [360, 0.0, 90.0, 30.0],
        "pos1": ["auto", 90.0, 90.0, 85.0],   
        "pos2": ["auto", 90.0, -5.0, 85.0],    
        "pos3": ["auto", 90.0, -5.0, 0.0],     
        "pos4": ["auto", 0.0, 90.0, 0.0],     
        "pos5": ["auto", 0.0, 90.0, 50.0],    
    }
    
    arm_sequence = py_trees.composites.Sequence(name="Chuoi_Gap_Hop", memory=True)
    arm_sequence.add_child(CalculatePickElevationAction("Kiem_Tra_Do_Cao_Arm_1", ros_node))
    
    steps = [
        ("Home_1", "home"), ("Approach", "pos1"), ("Lower", "pos2"), 
        ("Grip", "pos3"), ("Lift", "pos4"), ("Drop", "pos5"), ("Home_2", "home")
    ]
    for step_name, preset_key in steps:
        arm_sequence.add_child(MoveArmBehavior(f"Arm: {step_name}", ros_node, target_pose=presets[preset_key]))
        if step_name != "Home_2":
            arm_sequence.add_child(RosWaitBehavior(f"Wait", ros_node, delay_sec=1.0))
    return arm_sequence

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
    """Subtree: Quy trình xử lý quét AI và tiến vào ô lưới"""
    move_subtree = py_trees.composites.Sequence("Process & Enter Cell", memory=True)
    move_subtree.add_child(TurnToTargetCellBehavior("Turn To Face Cell", ros_node))
    
    align_vision = WallAlignmentBehavior("Align Before Vision", ros_node, window_degrees=60.0, goal_distance=0.4)
    move_subtree.add_child(py_trees.decorators.FailureIsSuccess("Ignore Align Fail 1", align_vision))

    move_subtree.add_child(py_trees.decorators.FailureIsSuccess("Run AI", 
        FollowTargetBehavior("Run AI (ID=1)", ros_node, target_id=1, desired_distance_mm=250.0)
    ))
    
    box_logic_selector = py_trees.composites.Selector("Box Logic (Real/Fake/Empty)", memory=False)

    real_seq = py_trees.composites.Sequence("Hunt: REAL Box", memory=True)
    real_seq.add_child(CheckBlackboardValue("Is Real?", "latest_box_detection", "REAL", operator.eq))
    real_seq.add_child(build_pick_and_place_sequence(ros_node)) 
    real_seq.add_child(IncrementRealCountAction("Logic: Add Score"))
    real_seq.add_child(MarkAsVisitedAction("Logic: Mark Accessible"))

    fake_seq = py_trees.composites.Sequence("Hunt: FAKE Box", memory=True)
    fake_seq.add_child(CheckBlackboardValue("Is Fake?", "latest_box_detection", "FAKE", operator.eq))
    fake_seq.add_child(MarkAsObstacleAction("Logic: Mark Obstacle"))
    fake_seq.add_child(py_trees.behaviours.Failure("Block Route")) 

    empty_seq = py_trees.composites.Sequence("Hunt: EMPTY", memory=True)
    empty_seq.add_child(MarkAsVisitedAction("Logic: Mark Accessible")) 

    box_logic_selector.add_children([real_seq, fake_seq, empty_seq])
    move_subtree.add_child(box_logic_selector)

    climb_up_align_seq = py_trees.composites.Sequence("Align if Climbing Up", memory=True)
    climb_up_align_seq.add_child(IsClimbingUpCondition("Check Elevation > 0", ros_node))
    climb_up_align_seq.add_child(WallAlignmentBehavior("Align Before Climb", ros_node, window_degrees=60.0, goal_distance=0.4))
    move_subtree.add_child(py_trees.decorators.FailureIsSuccess("Ignore Align Fail 2", climb_up_align_seq))

    move_subtree.add_child(DynamicClimbStepBehavior("Hardware: Auto Climb", ros_node))
    move_subtree.add_child(MoveRelativeOdomBehavior("Hardware: Move Relative into Cell", ros_node, climb_dist=0.3, flat_dist=1.2))
    
    return move_subtree

