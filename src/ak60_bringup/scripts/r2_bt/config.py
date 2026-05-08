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
    },
    'blue': {
        'map_cols':          [1,     1,     1,  1,   2,     2,     2  , 2,     3,     3,     3, 3    ],
        'map_rows':          [1,     2,     3,  4,   1,     2,     3  , 4,     1,     2,     3, 4    ],
        'map_heights':       [400.0, 600.0, 400.0, 200.0, 200.0, 400.0, 600.0, 400.0, 400.0, 200.0, 400.0, 200.0],
        'default_elevation': 0.0,
        'lat':              -1.0,
        'post_ramp_yaw':    90.0,
        'tool_arm_side':    'left',  
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
# Pick presets theo do chenh cao delta_h = h_target - h_curr (mm).
#   delta > 0 : bac chua KFS CAO hon bac robot → voi LEN gap
#   delta < 0 : bac chua KFS THAP hon bac robot → voi XUONG gap
#   delta = 0 : cung bac → voi THANG
#
# Field stage heights = {200, 400, 600} mm → cac delta kha thi:
#   -400, -200, 0, +200, +400
#
# Pose: [arm1, arm2, arm3, gripper] (4-DOF, dung MoveArmBehavior).
# TODO: tune cac gia tri nay bang teleop_test_arm.py — hien tai ke thua tu
# cap 'low'/'high' cu trong builders.py, can do thuc te tren tung bac.
# ─────────────────────────────────────────────────────────────────────────────
PICK_PRESETS_BY_DELTA = {
    +400: {  # 200→600: voi len cao nhat
        "home":      [   0.0,  -5.0,   0.0,  70.0],
        "pre_grasp": [   0.0,  85.0,  -1.5,  70.0],
        "extend":    [   0.0,  85.0,  -1.5,  70.0],
        "close":     [   0.0,  85.0,  -1.5, -28.0],
        "lift":      [   0.0,  10.0,   0.0, -28.0],
        "retract":   [   0.0,  10.0,   0.0,  70.0],
    },
    +200: {  # 200→400 hoac 400→600: voi len trung binh (≈ preset 'low' cu)
        "home":      [ 250.0,  -5.0,   0.0,  70.0],
        "pre_grasp": [ 250.0,  75.0,   0.0,  70.0],
        "extend":    [ 250.0,  75.0,   0.0,  70.0],
        "close":     [ 250.0,  75.0,   0.0, -50.0],
        "lift":      [ 300.0,   0.0,   80.0, -50.0],
        "retract":   [ 300.0,   0.0,   0.0,  70.0],
    },
    0: {     # cung bac: voi thang
        "home":      [ 250.0,  -5.0,   0.0,  70.0],
        "pre_grasp": [ 250.0,  75.0,   0.0,  70.0],
        "extend":    [ 250.0,  75.0,   0.0,  70.0],
        "close":     [ 250.0,  75.0,   0.0, -50.0],
        "lift":      [ 300.0,   0.0,   80.0, -50.0],
        "retract":   [ 300.0,   0.0,   0.0,  70.0],
    },
    -200: {  # 600→400 hoac 400→200: voi xuong trung binh (≈ preset 'high' cu)
        "home":      [ 300.0,  -5.0,   0.0,  70.0],
        "pre_grasp": [ 300.0,  55.0,   0.0,  70.0],
        "extend":    [ 300.0,  55.0,   0.0,  70.0],
        "close":     [ 300.0,  55.0,   0.0, -50.0],
        "lift":      [ 300.0,   0.0,   0.0, -50.0],
        "retract":   [ 300.0,   0.0,   0.0,  70.0],
    },
    -400: {  # 600→200: voi xuong sau nhat
        "home":      [ 300.0,  -5.0,   0.0,  70.0],
        "pre_grasp": [ 300.0,  75.0,   0.0,  70.0],
        "extend":    [ 300.0,  75.0,   0.0,  70.0],
        "close":     [ 300.0,  75.0,   0.0, -50.0],
        "lift":      [ 300.0,   0.0,   0.0, -50.0],
        "retract":   [ 300.0,   0.0,   0.0,  70.0],
    },
}


def closest_pick_preset(delta_h):
    """Snap delta_h tuy y ve preset key gan nhat trong PICK_PRESETS_BY_DELTA.
    Tra ve (key, preset_dict)."""
    keys = sorted(PICK_PRESETS_BY_DELTA.keys())
    nearest = min(keys, key=lambda k: abs(k - float(delta_h)))
    return nearest, PICK_PRESETS_BY_DELTA[nearest]
