# Phân Tích Thời Gian & Điểm Chậm — mission.py

> File phân tích: `src/ak60_bringup/scripts/r2_bt/trees/mission.py`  
> Các file liên quan: `builders.py`, `navigation.py`, `hardware.py`, `config.py`

---

## Tổng Quan Luồng Mission

```
Root (Parallel)
├── Topics2BB          ← subscribe ROS liên tục (nền)
├── BB_Logger          ← log blackboard mỗi 30 tick (nền)
└── Main_Mission (Sequence)
    ├── Phase 0: Phase_0_Standby
    ├── Phase 1: Khoi_Dong_Va_Lay_Dung_Cu
    ├── Phase 2: Vao_Luoi_Va_Lap  (loop lưới)
    └── Phase 4: Dat_Hop_Vao_Gia
```

---

## Phase 0 — Standby (`Phase_0_Standby`)

| Bước | Behavior | Thời gian ước tính |
|------|----------|--------------------|
| 1 | `MoveArmBehavior("Box_Arm_Home")` — gập tay về `[500, 0, 0, 0]` | ~1–2s (chờ arm server) |
| 2 | `WaitForStartSignalBehavior` — chờ tín hiệu `/gui_start_signal` | **Không giới hạn** |

**Điểm chú ý:** Phase 0 hoàn toàn phụ thuộc người vận hành. Không có timeout.

---

## Phase 1 — Lấy Dụng Cụ & Tiến Ra Cửa Lưới (`Khoi_Dong_Va_Lay_Dung_Cu`)

### 1.1 Di chuyển đến khu dụng cụ

| Bước | Behavior | Thời gian ước tính |
|------|----------|--------------------|
| 1 | `ArmSequenceBTNode("Tool_Arm_Approach", duration=1.0)` | **1.0s** cứng |
| 2 | `GoToRelativePoseBehavior("Tien_Lay_Dung_Cu", dx=0.4, dy=±0.6)` | ~3–5s (Nav2) |

### 1.2 Quy trình lấy dụng cụ (`build_tool_assembly_sequence`)

| Bước | Behavior | Thời gian ước tính |
|------|----------|--------------------|
| 1 | `ArmSequenceBTNode("Approach_J3", duration=2.0)` | **2.0s** cứng |
| 2 | `ArmSequenceBTNode("Grasp_Tool", duration=1.0)` | **1.0s** cứng |
| 3 | `ArmSequenceBTNode("Move_Back_J4", duration=1.0)` | **1.0s** cứng |
| 4 | `ArmSequenceBTNode("Move_Back_J1", duration=1.0)` | **1.0s** cứng |

### 1.3 Lắp vũ khí — ĐIỂM CHẬM NẶNG NHẤT

| Bước | Behavior | Thời gian ước tính |
|------|----------|--------------------|
| 1 | `ArmSequenceBTNode("Lap_Vu_Khi", "assemble_sequence", duration=20.0)` | **🔴 20.0s cứng** |

> Đây là bước chậm nhất toàn mission. 20s được đặt cứng trong tham số `duration`.
> Nếu cơ cấu lắp ráp hoàn thành sớm hơn, robot vẫn phải chờ đủ 20s.

### 1.4 Di chuyển đến cửa ô lưới

| Bước | Behavior | Thời gian ước tính |
|------|----------|--------------------|
| 1 | `GoToEntranceBoxBehavior("Di_Den_Entrance_Box")` | ~5–8s/entrance box (Nav2) |
| 2 | `WallAlignmentBehavior("Align_Cua_Cell", timeout_sec=5.0)` | **0–5s** |

### 1.5 Xử lý entrance box col1/col3 (nếu có)

| Bước | Behavior | Thời gian ước tính |
|------|----------|--------------------|
| 1 | `MoveArmBehavior("Box_Arm_Home")` | ~0.5s |
| 2 | `FollowTargetBehavior("Follow_Cua_Cell", dist=325mm, timeout=15s)` | **🟡 0–15s** (AI tracking) |
| 3 | `_build_arm_seq(preset='400', 6 bước × 0.5s wait)` | **3.0s** chờ cứng + arm move |
| 4 | `GoToRelativePoseBehavior("Lui_Adj")` + `("Adj_Col")` | ~3–5s |

**Tổng Phase 1 ước tính: 35–55s** (phần lớn do `assemble_sequence` 20s)

---

## Phase 2 — Loop Lưới (`Vao_Luoi_Va_Lap`)

### Mỗi iteration — `build_move_subtree`

```
Process_And_Enter_Cell (Sequence)
├── TurnToTargetCellBehavior           ← xoay hướng về ô
├── WallAlignmentBehavior (timeout 5s) ← căn góc trước AI
├── FollowTargetBehavior (timeout 15s) ← AI quét hộp
├── Box_Logic (Selector)
│   ├── Hunt_REAL → build_pick_and_place_sequence_dynamic
│   ├── Hunt_FAKE → MarkAsObstacle
│   └── Hunt_EMPTY → MarkAsVisited
├── DynamicClimbStepBehavior           ← leo/xuống bậc nếu cần
└── MoveRelativeOdomBehavior           ← tiến vào tâm ô
```

| Bước | Behavior | Thời gian ước tính |
|------|----------|--------------------|
| 1 | `TurnToTargetCellBehavior` | ~2–4s (Nav2 rotate in place) |
| 2 | `WallAlignmentBehavior("Align_Before_Vision", timeout=5.0)` | **0–5s** |
| 3 | `FollowTargetBehavior("Run_AI_ID1", dist=0mm, timeout=15s)` | **🟡 0–15s** |
| 4a (REAL) | `FollowTargetBehavior` trong pick (timeout=15s) | **🟡 0–15s** thêm |
| 4b (REAL) | `_build_arm_seq` 6 bước × 0.5s | **3.0s** cứng |
| 4c (REAL thấp) | `WallAlignmentBehavior("Align_Truoc_Gap_Thap", goal=0.65m)` | **0–5s** |
| 5 | `DynamicClimbStepBehavior` | **0–15s** (timeout climb) |
| 6 | `MoveRelativeOdomBehavior(flat_dist=1.2m)` | ~3–5s (Nav2) |

- **Mỗi ô trống:** ~10–15s  
- **Mỗi ô REAL:** ~25–45s  
- **Tổng 12 ô (3×4):** 120–540s tùy route và số ô REAL/FAKE

### Điều kiện thoát loop

Loop dừng khi:
1. `target_boxes_list == []` (AI xác nhận hết hộp cần lấy) **VÀ**
2. `real_box_count >= 2` (đã gắp đủ 2 hộp) **VÀ**
3. `IsAtExitCellCondition` (đang ở `(1,4)` hoặc `(3,4)`)

Nếu điều kiện 3 chưa đạt → `Mode_FIND_Exit` tiếp tục di chuyển đến ô thoát.

---

## Phase 3 — Thoát Khỏi Sàn (`Chuoi_Thoat_Khoi_San`)

| Bước | Behavior | Thời gian ước tính |
|------|----------|--------------------|
| 1 | `GoToRelativePoseBehavior("Quay_Huong_Thoat", yaw=0°)` | ~1–3s (xoay tại chỗ) |
| 2 | `WallAlignmentBehavior("Align_Truoc_Khi_Thoat", timeout=5s)` | **0–5s** |
| 3 | `ClimbStepBehavior("Thoat_Xuong_Bac", velocity=0.1)` | **🔴 ~10–15s** — v=0.1 m/s rất chậm |
| 4 | `GoToRelativePoseBehavior("Thoat_Tien_025_Sau", dx=0.25)` | ~1–2s |
| 5 | `WallAlignmentBehavior("Thoat_Align_1", timeout=5s)` | **0–5s** |
| 6 | Di chuyển ngang về hướng dốc (1.1–3.7m tùy cột) | ~5–12s (Nav2) |
| 7 | `WallAlignmentBehavior("Thoat_Align_2", timeout=5s)` | **0–5s** |
| 8 | `ClimbStepBehavior("Thoat_Len_Doc", velocity=0.6)` | ~3–6s (nhanh hơn 6×) |
| 9 | `GoToRelativePoseBehavior("Quay_Ve_Arena", yaw=±90°)` | ~2–4s |
| 10 | `GoToRelativePoseBehavior("Tien_vao_tic_tac_toe", dx=3.5, dy=±1.0)` | **🟡 ~10–15s** — quãng đường dài nhất |
| 11 | `WallAlignmentBehavior("Align_Sau_Ramp", timeout=5s)` | **0–5s** |

**Điểm chú ý:**
- `Thoat_Xuong_Bac` dùng `velocity=0.1` — chậm hơn `Thoat_Len_Doc` (0.6) tới 6 lần.
- `Tien_vao_tic_tac_toe` có `dx=3.5m` — quãng đường dài nhất trong toàn mission.

**Tổng Phase 3 ước tính: 25–55s**

---

## Phase 4 — Đặt Hộp Lên Giá (`Dat_Hop_Vao_Gia`)

| Bước | Behavior | Thời gian ước tính |
|------|----------|--------------------|
| 1 | `FollowTargetBehavior("Run_AI_Exit", target=R1, dist=300mm, timeout=15s)` | **🟡 0–15s** |
| 2–5 | `MoveArmBehavior` × 4 (pose 11→14) | ~2–4s |
| 6 | `WallAlignmentBehavior("Align_Exit_Cell", goal=0.35m, timeout=10.0)` | **🟡 0–10s** |
| 7–8 | `MoveArmBehavior` × 2 (pose 15–16) + `GoToRelativePoseBehavior(-0.5m)` | ~3s |
| 9 | `GoToRelativePoseBehavior("Tien_0.5m_Exit_2", dy=0.54)` | ~2–3s |
| 10–12 | `MoveArmBehavior` × 3 (pose 21–23) | ~2s |
| 13 | `WallAlignmentBehavior("Align_Exit_Cell", goal=0.4m, timeout=10.0)` | **🟡 0–10s** |
| 14–16 | `MoveArmBehavior` × 3 (pose 24–25) + `GoToRelativePoseBehavior(-0.5m)` | ~3s |

**Tổng Phase 4 ước tính: 20–50s**

---

## Bảng Tổng Hợp Điểm Gây Chậm

| Mức | Behavior | Thời gian tối đa | Nguyên nhân |
|-----|----------|-----------------|-------------|
| 🔴 | `ArmSequenceBTNode("Lap_Vu_Khi", duration=20.0)` | **20.0s cứng** | Duration hardcode, không dừng sớm |
| 🔴 | `ClimbStepBehavior("Thoat_Xuong_Bac", velocity=0.1)` | **~15s** | v=0.1 m/s — 6× chậm hơn leo lên |
| 🟡 | `FollowTargetBehavior` (×4 lần trong mission) | **15s/lần** | AI không thấy target → chạy hết timeout |
| 🟡 | `GoToRelativePoseBehavior("Tien_vao_tic_tac_toe", dx=3.5)` | **~15s** | Quãng đường dài nhất mission |
| 🟡 | `WallAlignmentBehavior` (×8+ lần) | **5–10s/lần** | Tích lũy: 8 × 5s = 40s worst case |
| 🟢 | `_build_arm_seq` 6 steps × 0.5s (gọi nhiều lần) | **3.0s/lần** | Wait cứng sau mỗi bước arm |
| 🟢 | `TurnToTargetCellBehavior` (mỗi ô lưới) | ~4s/ô | Tích lũy qua 12 ô: ~48s |
| 🟢 | `DynamicClimbStepBehavior` (ô có bậc) | ~8s/ô | Chỉ khi có leo bậc |

---

## Ước Tính Tổng Thời Gian Mission

| Phase | Best case | Worst case |
|-------|-----------|------------|
| Phase 1 (lấy dụng cụ) | ~35s | ~55s |
| Phase 2 (loop 12 ô) | ~80s | ~360s |
| Phase 3 (thoát sàn) | ~25s | ~55s |
| Phase 4 (đặt hộp) | ~20s | ~50s |
| **Tổng** | **~160s (~2.7 phút)** | **~520s (~8.7 phút)** |

---

## Khuyến Nghị Tối Ưu

### Ưu tiên cao

1. **`assemble_sequence` duration 20s → dynamic feedback**  
   Thêm phản hồi từ ESP32 arm server để dừng ngay khi lắp xong, thay vì chờ cứng 20s.

2. **Tăng `Thoat_Xuong_Bac` velocity từ 0.1 → 0.25**  
   Sửa `CLIMB_PARAMS['down_velocity']` trong `config.py` nếu cơ khí cho phép.

3. **Giảm `FollowTargetBehavior` timeout từ 15s → 8s**  
   Nếu sau 8s không thấy target, tiếp tục mission thay vì chờ thêm 7s vô ích.

### Ưu tiên trung bình

4. **`RosWaitBehavior` trong `_build_arm_seq`: 0.5s → 0.3s/bước**  
   Tiết kiệm 1.2s/lần gọi arm sequence, gọi nhiều lần trong loop.

5. **Tăng `tolerance_deg` WallAlignment từ 1.5° → 2.0°** trong `config.py`  
   Align xong nhanh hơn, chấp nhận sai số góc nhỏ hơn.

6. **Confirm `TurnToTargetCellBehavior` không cần backup phase**  
   Hiện backup (`phase=0`) bị skip trong `update()`. Nếu không cần, xóa code dead để tránh nhầm lẫn.
