#!/usr/bin/env python3

GRID_CELL_SIZE = 1.2
GRID_ORIGIN_X  = 0.0
GRID_ORIGIN_Y  = 0.0

GRID_MIN_COL, GRID_MAX_COL = 1, 3
GRID_MIN_ROW, GRID_MAX_ROW = 1, 4

# ─────────────────────────────────────────────────────────────────────────────
# ABU Robocon 2026 – "Kung Fu Quest" – Meihua Forest field configs
#
# Height map from meihua_forest.txt. Red and Blue are horizontal mirrors.
# Only cells with height != default (400mm) are stored in map_cols/rows/heights.
#
# lat           – sign multiplied onto every lateral (dy) move in mission.py
#   'red' : +1.0  (positive dy heads toward the Arena ramp)
#   'blue': -1.0  (field is mirrored → same physical direction, opposite sign)
#
# post_ramp_yaw – target yaw (deg) after ascending the Arena ramp
#   'red' : -90.0  (turn right to face Tic-Tac-Toe rack)
#   'blue': +90.0  (turn left  to face Tic-Tac-Toe rack)
# ─────────────────────────────────────────────────────────────────────────────

# Height map source: meihua_forest.txt (measured from actual field)
# Full 3×4 grid (col 1-3, row 1-4). Only cells != 400mm are listed.
# RED layout:                  BLUE layout (mirror of red, col1↔col3):
#   col1 col2 col3               col1 col2 col3
#   400  200  400  row1           400  200  400  row1
#   200  400  600  row2           600  400  200  row2
#   400  600  400  row3           400  600  400  row3
#   200  400  200  row4           200  400  200  row4
FIELD_CONFIGS = {
    'red': {
        'map_cols':          [1,     1,     1,  1,   2,     2,     2  , 2,     3,     3,     3, 3    ],
        'map_rows':          [1,     2,     3,  4,   1,     2,     3  , 4,     1,     2,     3, 4    ],
        'map_heights':       [400.0, 200.0, 400.0, 200.0, 200.0, 400.0, 600.0, 400.0, 400.0, 600.0, 400.0, 200.0],
        'default_elevation': 0.0,
        'lat':               1.0,
        'post_ramp_yaw':    -90.0,
        'tool_arm_side':    'right',
        'ai_target_id':     'class4',  # YOLO label của hộp R2REAL phía red
    },
    'blue': {
        'map_cols':          [1,     1,     1,  1,   2,     2,     2  , 2,     3,     3,     3, 3    ],
        'map_rows':          [1,     2,     3,  4,   1,     2,     3  , 4,     1,     2,     3, 4    ],
        'map_heights':       [400.0, 600.0, 400.0, 200.0, 200.0, 400.0, 600.0, 400.0, 400.0, 200.0, 400.0, 200.0],
        'default_elevation': 0.0,
        'lat':              -1.0,
        'post_ramp_yaw':    90.0,
        'tool_arm_side':    'left',
        'ai_target_id':     'class5',  # YOLO label của hộp R2REAL phía blue
    },
}


def manhattan_distance(c1, r1, c2, r2):
    return abs(c1 - c2) + abs(r1 - r2)


# ─────────────────────────────────────────────────────────────────────────────
# KFS class labels — phai khop voi label_class.txt cua node perception (YOLO).
# id → ten class. Dung de bridge giua output detector (so) va logic BT (chuoi).
# ─────────────────────────────────────────────────────────────────────────────
CLASS_LABELS = {
    0: 'KFS-unknown',
    1: 'R1-BLUE',
    2: 'R1-RED',
    3: 'R2FAKE-BLUE',
    4: 'R2FAKE-RED',
    5: 'R2REAL-BLUE',
    6: 'R2REAL-RED',
}

# Hanh dong tuong ung voi tung class:
#   'pick'  → gap hop (R1, R2REAL — phia mau team se loc them o BT)
#   'avoid' → danh dau o lam vat can, khong gap
#   'skip'  → bo qua (chua xac dinh nhan)
CLASS_PICK_INTENT = {
    'KFS-unknown': 'skip',
    'R1-BLUE':     'pick',
    'R1-RED':      'pick',
    'R2FAKE-BLUE': 'avoid',
    'R2FAKE-RED':  'avoid',
    'class5': 'pick',
    'class4':  'pick',
}


def is_own_team_class(class_name, team_color):
    """team_color: 'red' hoac 'blue'. Tra ve True neu class thuoc team minh."""
    if not class_name:
        return False
    suffix = team_color.upper()
    return class_name.endswith(f'-{suffix}')



# ─────────────────────────────────────────────────────────────────────────────
# Tuning parameters — tập trung tại đây để dễ chỉnh mà không cần mở từng file
# ─────────────────────────────────────────────────────────────────────────────

CLIMB_PARAMS = {
    'up_velocity':      0.3,    # m/s — vận tốc khi leo LÊN bậc
    'down_velocity':    0.1,    # m/s — vận tốc khi XUỐNG bậc
    'pitch_threshold':  5.0,    # deg — ngưỡng phát hiện bắt đầu leo/xuống
    'flat_threshold':   1.5,    # deg — ngưỡng xác nhận robot đã phẳng
    'timeout_sec':      15.0,   # s   — timeout toàn bộ hành động leo bậc
    'yaw_kp':           1.5,    # P-gain giữ thẳng hướng khi leo/xuống
    'yaw_max_ang':      0.25,   # rad/s — clamp angular.z yaw lock
}

WALL_ALIGN_PARAMS = {
    'kp_angular':           2.5,    # P-gain xoay phase 1
    'kp_linear':            1.5,    # P-gain tiến phase 3
    'ang_clamp':            0.1,    # rad/s — clamp angular.z phase 1
    'lin_clamp':            0.2,    # m/s  — clamp linear.x phase 3
    'tolerance_deg':        1.5,    # deg  — ngưỡng dừng phase 1 (vuông góc)
    'tolerance_m':          0.01,   # m    — ngưỡng dừng phase 3 (khoảng cách)
    'lidar_yaw_offset_deg': 90.0,   # deg  — offset góc LiDAR so với base_link
}

NAV_PARAMS = {
    'flat_dist':            1.2,    # m   — khoảng cách di chuyển vào ô phẳng
    'climb_dist':           0.3,    # m   — khoảng cách di chuyển sau khi leo bậc
    'wall_window_deg':      20.0,   # deg — góc quét LiDAR khi wall align
    'wall_dist_vision':     0.4,    # m   — goal_distance wall align trước khi bật AI
    'wall_dist_climb':      0.3,    # m   — goal_distance wall align trước khi leo
    'follow_dist_mm':       300.0,  # mm  — khoảng cách dừng khi FollowTarget
    'align_timeout_sec':    5.0,    # s   — timeout WallAlignmentBehavior trong BT
}

TRACKING_PARAMS = {
    'kp_linear_mm':     0.003,      # P-gain tiến (PHASE_APPROACH)
    'kp_lateral':       0.0005,     # P-gain strafe (PHASE_ALIGN_X)
}


# ─────────────────────────────────────────────────────────────────────────────
# Pick presets — dùng cho build_pick_and_place_sequence (đặt hộp lên giá TTT).
# Chọn theo mode ('low' / 'high') VÀ số hộp đã ghi điểm (real_box_count):
#   _1 → hộp đầu tiên  (arm1 nâng 300mm)
#   _2 → hộp thứ hai   (arm1 nâng 500mm để tránh hộp đã đặt trước)
# Pose: [arm1, arm2, arm3, gripper]
# ─────────────────────────────────────────────────────────────────────────────
PICK_PRESETS = {
    'low_1': {
        "home":      [   0.0,  -5.0,   0.0,  70.0],
        "pre_grasp": [   0.0,  80.0,   0.0,  70.0],
        "close":     [   0.0,  80.0,   0.0, -50.0],
        "lift":      [ 250.0,  15.0,  90.0, -50.0],
        "retract":   [ 250.0,   0.0,   0.0,  40.0],
    },
    'low_2': {
        "home":      [   0.0,  -5.0,   0.0,  70.0],
        "pre_grasp": [   0.0,  80.0,   0.0,  70.0],
        "close":     [   0.0,  80.0,   0.0, -50.0],
        "lift":      [ 500.0,  15.0,  90.0, -50.0],
        "retract":   [ 500.0,   0.0,   0.0,  40.0],
        "hold":      [ 500.0,   0.0,   0.0,   0.0],
    },
    'high_1': {
        "home":      [ 300.0,  -5.0,   0.0,  70.0],
        "pre_grasp": [ 300.0,  80.0,   0.0,  70.0],
        "close":     [ 300.0,  80.0,   0.0, -50.0],
        "lift":      [ 250.0,  15.0,  90.0, -50.0],
        "retract":   [ 250.0,   0.0,   0.0,  40.0],
    },
    'high_2': {
        "home":      [ 300.0,  -5.0,   0.0,  70.0],
        "pre_grasp": [ 300.0,  80.0,   0.0,  70.0],
        "close":     [ 300.0,  80.0,   0.0, -50.0],
        "lift":      [ 500.0,  15.0,  90.0, -50.0],
        "retract":   [ 500.0,   0.0,   0.0,  40.0],
        "hold":      [ 500.0,   0.0,   0.0,   0.0],
    },
}

# (step_name, preset_key, wait_sec) — hộp thứ nhất, không có bước hold
PICK_STEPS_COUNT_1 = [
    ("Home_1",    "home",      1.0),
    ("Pre_Grasp", "pre_grasp", 1.0),
    ("Close",     "close",     1.0),
    ("Lift",      "lift",      1.0),
    ("Retract",   "retract",   1.0),
    ("Home_2",    "home",      0.0),
]

# Hộp thứ hai — thêm bước hold ở arm1=500mm để tránh va hộp đã đặt trước
PICK_STEPS_COUNT_2 = [
    ("Home_1",    "home",      1.0),
    ("Pre_Grasp", "pre_grasp", 1.0),
    ("Close",     "close",     1.0),
    ("Lift",      "lift",      1.0),
    ("Retract",   "retract",   1.0),
    ("Hold",      "hold",      2.0),
    ("Home_2",    "home",      0.0),
]
