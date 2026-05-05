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
        'map_cols':          [2,     1,     3,     2,     1,     3    ],
        'map_rows':          [1,     2,     2,     3,     4,     4    ],
        'map_heights':       [200.0, 200.0, 600.0, 600.0, 200.0, 200.0],
        'default_elevation': 400.0,
        'lat':               1.0,
        'post_ramp_yaw':    -90.0,
        'tool_arm_side':    'right',   
    },
    'blue': {
        'map_cols':          [2,     1,     3,     2,     1,     3    ],
        'map_rows':          [1,     2,     2,     3,     4,     4    ],
        'map_heights':       [200.0, 600.0, 200.0, 600.0, 200.0, 200.0],
        'default_elevation': 400.0,
        'lat':              -1.0,
        'post_ramp_yaw':    90.0,
        'tool_arm_side':    'left',  
    },
}


def manhattan_distance(c1, r1, c2, r2):
    return abs(c1 - c2) + abs(r1 - r2)
