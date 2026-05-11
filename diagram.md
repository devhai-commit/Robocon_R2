R2 Mission

[||] Root  (SuccessOnSelected: Main_Mission)
│
├── [→] Topics2BB
│   └── (A) GuiSignal2BB  ← /gui_start_signal → BB["gui_start_raw"]
│
├── [→] Main_Mission
│   │
│   ├── ─────────────── PHASE 0: STANDBY ───────────────
│   │
│   ├── [→] Phase_0_Standby
│   │   ├── (A) Tool_Arm_Home        ← ESP32: home_pose_{side}
│   │   ├── (A) Box_Arm_Home         ← MoveIt: [325,0,0,0]
│   │   └── (A) Wait_For_GUI_Strategy ← parse JSON → BB[priority_col, target_boxes_list]
│   │
│   ├── ─────────────── PHASE 1: INIT & TOOL PICKUP ────
│   │
│   ├── [→] Khoi_Dong_Va_Lay_Dung_Cu
│   │   ├── (A) Tool_Arm_Approach    ← ESP32: approach_j4_{side}
│   │   ├── (A) Tien_Lay_Dung_Cu     ← navigate: dx=0.42, dy=lat×0.89
│   │   ├── [→] Quy_Trinh_Lay_Dung_Cu  (build_tool_assembly_sequence)
│   │   │   ├── (A) Approach_J3      ← ESP32: approach_j3_{side}  (2.0s)
│   │   │   ├── (A) Grasp_Tool       ← ESP32: grasp_{side}         (1.0s)
│   │   │   ├── (A) Move_Back_J4     ← ESP32: move_back_J4_{side}  (1.0s)
│   │   │   ├── (A) Move_Back_J1     ← ESP32: move_back_J1_{side}  (1.0s)
│   │   │   └── (A) Assemble_Action  ← ESP32: assemble_sequence    (20.0s)
│   │   ├── (A) Tien_Ra_Cua_Cell     ← navigate: dx=1.6, dy=-lat×2.5
│   │   └── (BB) Set_Target_2_1      ← BB[target_cell] = (2,1)
│   │
│   ├── ─────────────── PHASE 2: GRID MISSION LOOP ─────
│   │
│   ├── [→] Vao_Luoi_Va_Lap
│   │   ├── [→] Process_And_Enter_Cell  ← (lần đầu vào ô 2,1)
│   │   │   └── ... (xem build_move_subtree bên dưới)
│   │   │
│   │   └── [KRS] Loop_Explore_Exit
│   │       └── [?] Mode_Selector
│   │           │
│   │           ├── [→] Mode_EXIT_Goal
│   │           │   ├── (C) Real_GE_2?      BB[real_box_count] >= 2
│   │           │   ├── (C) At_Exit_Cell?   BB[robot_pos] ∈ {(1,4),(3,4)}
│   │           │   ├── [→] Chuoi_Thoat_Khoi_San  ← EXIT SEQUENCE (xem bên dưới)
│   │           │   └── (A) Mission_Complete  → SUCCESS → kết thúc loop
│   │           │
│   │           ├── [→] Mode_FIND_Exit
│   │           │   ├── (C) Real_GE_2_Find? BB[real_box_count] >= 2
│   │           │   ├── (A) Route_Exit      DecideNextGridCell(mode=EXIT)
│   │           │   ├── [→] Process_And_Enter_Cell  (build_move_subtree)
│   │           │   └── (A) Loop_Find_Exit  → FAILURE → loop lại
│   │           │
│   │           └── [→] Mode_HUNT
│   │               ├── (A) Route_Explore   DecideNextGridCell(mode=EXPLORE)
│   │               ├── [→] Process_And_Enter_Cell  (build_move_subtree)
│   │               └── (A) Loop_Hunt       → FAILURE → loop lại
│   │
│   └── (A) IDLE_MODE_ACTIVATED  ← Running (không bao giờ tới)
│
└── (A) BB_Logger  ← log blackboard mỗi 30 tick




BUILD MOVE SUBTREE

[→] Process_And_Enter_Cell
│
├── (A) Turn_To_Face_Cell      ← navigate: xoay về hướng target_cell
│
├── [FIS] Ignore_Align_1
│   └── [TO:5s] Align_1_Timeout
│       └── (A) Align_Before_Vision  ← LiDAR wall-align, dist=0.4m, window=20°
│
├── [FIS] Run_AI
│   └── (A) Run_AI_ID1         ← YOLO FollowTarget(ai_target_id, dist=315mm)
│                                  → "real"/"fake"/"empty" ghi BB[latest_box_detection]
│
├── [?] Box_Logic
│   ├── [→] Hunt_REAL
│   │   ├── (C) Is_Real?       BB[latest_box_detection] == "REAL"
│   │   ├── [?] Pick_Dynamic_Mode  (build_pick_and_place_sequence_dynamic)
│   │   │   ├── [→] Pick_If_High
│   │   │   │   ├── (C) Is_Target_Higher   BB: h(target) > h(robot_pos)
│   │   │   │   └── [?] Place_HIGH
│   │   │   │       ├── [→] Place_high_Count2
│   │   │   │       │   ├── (C) Real_GE_2?  real_box_count >= 1
│   │   │   │       │   └── [→] ArmSeq_high_2  
|   |   |   |       |            (home→pre_grasp→close→lift@510→retract→hold→home)
│   │   │   │       └── [→] ArmSeq_high_1      (home→pre_grasp→close→lift@250→retract→home)
│   │   │   └── [?] Place_LOW
│   │   │       ├── [→] Place_low_Count2
│   │   │       │   ├── (C) Real_GE_2?  real_box_count >= 1
│   │   │       │   └── [→] ArmSeq_low_2   (home→pre_grasp→close→lift@510→retract→hold→home)
│   │   │       └── [→] ArmSeq_low_1        (home→pre_grasp→close→lift@250→retract→home)
│   │   ├── (A) Add_Score      BB[real_box_count]++
│   │   └── (A) Mark_Accessible BB[grid_status][cell]['visited'] = True
│   │
│   ├── [→] Hunt_FAKE
│   │   ├── (C) Is_Fake?       BB[latest_box_detection] == "FAKE"
│   │   ├── (A) Mark_Obstacle  BB[grid_status][cell]['state'] = 'obstacle'
│   │   └── (A) Block_Route    → FAILURE (chặn route)
│   │
│   └── [→] Hunt_EMPTY
│       └── (A) Mark_Accessible_Empty  BB['visited'] = True
│
├── [FIS] Ignore_Align_2
│   └── [→] Align_If_Elevation_Change
│       ├── (C) Check_Elevation  h(target) - h(robot_pos) > ngưỡng?
│       └── (A) Align_Before_Climb  ← LiDAR wall-align, dist=0.3m
│
├── (A) Auto_Climb       ← DynamicClimbStep: velocity lên=0.3 m/s, xuống=0.1 m/s
└── (A) Move_Into_Cell   ← navigate: climb_dist=0.3m hoặc flat_dist=1.2m




EXIT SEQUENCE 

[→] Chuoi_Thoat_Khoi_San
├── (A) Thoat_Tien_025       navigate: dx=0.25
├── (A) Thoat_Xuong_Bac      ClimbStep: velocity=0.1 (xuống bậc)
├── (A) Thoat_Tien_025_Sau   navigate: dx=0.25
├── [FIS] Ignore_Align_Exit
│   └── (A) Thoat_Align_1    wall-align, window=60°, dist=0.4m
├── [?] Chon_Huong_Thoat
│   ├── [→] Thoat_Tu_3_4
│   │   ├── (C) La_o_3_4?    BB[robot_pos] == (3,4)
│   │   └── (A) Di_Ve_Doc_1_3m  navigate: dy=lat×1.3
│   └── [→] Thoat_Tu_1_4
│       ├── (C) La_o_1_4?    BB[robot_pos] == (1,4)
│       └── (A) Di_Ve_Doc_3_7m  navigate: dy=lat×3.7
├── [FIS] Ignore_Align_Ramp
│   └── (A) Thoat_Align_2    wall-align, window=60°, dist=0.4m
├── (A) Thoat_Len_Doc        ClimbStep: velocity=0.5 (lên dốc)
├── (A) Quay_Ve_Arena        navigate: yaw=post_ramp_yaw
└── (A) Tien_4m              navigate: dx=4.0



Luồng dữ liệu Blackboard chính
    Key	            |                    Ghi bởi	            |             Đọc bởi
gui_start_raw	    |                GuiSignal2BB	            |  WaitForStartSignal
                    |                                           |
priority_col,       |                WaitForStartSignal	        |  DecideNextGridCell
                    |                                           |
target_boxes_list	|                                           |
robot_pos	        |              DecideNextGridCell	        |TurnToCell, Check_Elevation
                    |                                           |     IsAtExitCell
                    |                                           |
target_cell	        |        DecideNextGridCell                 | TurnToCell, 
                                                        Check_Elevation,Pick_Dynamic_Mode
                    |                                           |
latest_box_detection|	        FollowTargetBehavior	        |    Is_Real?, Is_Fake?
                    |                                           |
real_box_count	    |            IncrementRealCount	            |Real_GE_2?, ArmSeq branch 
                    |                                           |         selection
                    |                                           |
grid_status	        |        MarkAsVisited, MarkAsObstacle	    |DecideNextGridCell
                    |                                           |
just_climbed	    |        DynamicClimbStep	                |MoveRelativeOdom
                    |                                           |
current_arm1_z	    |            MoveArmBehavior	            |Pick_Dynamic_Mode  
                                                                (high/low preset)