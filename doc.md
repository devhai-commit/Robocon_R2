mission.py — Grid Navigation BT (mission chính)
Phase 0: Standby
Thứ tự	Node	Mô tả
1	MoveArmBehavior("Box_Arm_Home")	Gập tay về [325, 0, 0, 0]
2	WaitForStartSignalBehavior	Chờ GUI → set priority_col, target_boxes_list
Phase 1: Init
Thứ tự	Node	Mô tả
3	SetBlackboardVariable("Set_Target_2_1")	Đặt target_cell = (2,1)
Tien_Lay_Dung_Cu, build_tool_assembly, Tien_Ra_Cua_Cell		(đang comment out)
Phase 2: Loop lưới — build_move_subtree (gọi mỗi lần di ô)

grid_phase:
  ├─ build_move_subtree  ← vào ô đầu tiên (2,1)
  └─ KeepRunningUntilSuccess
       └─ Mode_Selector (Selector)
            ├─ Mode_EXIT_Goal: real_box≥2 AND at_exit_cell → exit_seq → SUCCESS
            ├─ Mode_FIND_Exit: real_box≥2 → DecideNext(EXIT) → build_move_subtree → FAIL (loop)
            └─ Mode_HUNT:       DecideNext(EXPLORE) → build_move_subtree → FAIL (loop)
Mỗi lần vào ô (build_move_subtree) gọi tuần tự:

#	Action Node	Ghi chú
1	TurnToTargetCellBehavior	Quay mặt về ô đích
2	WallAlignmentBehavior("Align_Before_Vision")	Canh tường, timeout 5s, FailureIsSuccess
3	FollowTargetBehavior("Run_AI_ID1", class5, 300mm)	AI tracking hộp, FailureIsSuccess
4	Box_Logic (Selector):	
→ Hunt_REAL: Is_Real? → DeltaPickSequenceBehavior → IncrementRealCountAction → MarkAsVisitedAction	Hộp thật: gắp + đếm điểm
→ Hunt_FAKE: Is_Fake? → MarkAsObstacleAction → Failure	Hộp giả: mark obstacle
→ Hunt_EMPTY: MarkAsVisitedAction	Ô rỗng: mark visited
5	HasElevationChangeCondition + WallAlignmentBehavior("Align_Before_Climb")	Canh tường nếu có bậc, FailureIsSuccess
6	DynamicClimbStepBehavior("Auto_Climb")	Leo/xuống bậc tự động
7	MoveRelativeOdomBehavior("Move_Into_Cell")	Tiến vào ô (climb_dist=0.3, flat_dist=1.2)