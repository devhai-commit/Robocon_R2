#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
import json


class GuiEntranceNode(Node):
    def __init__(self):
        super().__init__('gui_entrance_panel_node')
        self.pub = self.create_publisher(String, '/gui_start_signal', 10)

    def send_config(self, priority_col, target_boxes, entrance_boxes):
        data = {
            "command": "START",
            "priority_col": int(priority_col),
            "target_boxes": target_boxes,
            "entrance_boxes": entrance_boxes,  # [[col, 1], ...] — ô hàng đầu robot vào trước
        }
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)
        self.get_logger().info(f"🚀 ĐÃ PHÁT LỆNH CHIẾN THUẬT: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    ros_node = GuiEntranceNode()

    root = tk.Tk()
    root.title("Robocon Strategy Control - Entrance Box Selector")
    root.geometry("900x520")
    root.configure(bg='#1e1e1e')

    selected_col = tk.IntVar(value=2)
    target_boxes_list = []
    entrance_boxes = []          # ← biến lưu các ô hàng đầu được chọn
    entrance_buttons = {}        # {col: tk.Button}

    # --- XỬ LÝ CHỌN / BỎ CHỌN Ô HÀNG 1 ---
    def toggle_entrance(c):
        btn = entrance_buttons[c]
        cell = [c, 1]
        if cell in entrance_boxes:
            entrance_boxes.remove(cell)
            btn.config(bg="#333333", fg="white", relief=tk.RAISED)
        else:
            entrance_boxes.append(cell)
            btn.config(bg="#ff8800", fg="black", relief=tk.SUNKEN)
        update_label()

    def update_label():
        if not entrance_boxes:
            lbl_entrance.config(text="Chưa chọn ô vào nào", fg="#aaaaaa")
        else:
            txt = " | ".join([f"C{x[0]}-H1" for x in sorted(entrance_boxes, key=lambda x: x[0])])
            lbl_entrance.config(text=f"Ô hàng đầu vào trước: {txt}", fg="#ff8800")

    def clear_all():
        entrance_boxes.clear()
        for btn in entrance_buttons.values():
            btn.config(bg="#333333", fg="white", relief=tk.RAISED)
        update_label()

    def on_start():
        ros_node.send_config(selected_col.get(), target_boxes_list, entrance_boxes)
        btn_start.config(text="🚀 ROBOT ĐANG THI ĐẤU...", bg='#555555', fg="white", state=tk.DISABLED)

    # ================= LAYOUT =================
    left_frame = tk.Frame(root, bg='#1e1e1e')
    left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=20, pady=20)

    right_frame = tk.Frame(root, bg='#2b2b2b', bd=2, relief=tk.GROOVE)
    right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=20, pady=20)

    # --- TRÁI: CỘT ƯU TIÊN & TRẠNG THÁI ---
    tk.Label(left_frame, text="1. CHỌN CỘT ƯU TIÊN DI CHUYỂN",
             fg="#00ffcc", bg="#1e1e1e", font=("Arial", 16, "bold")).pack(pady=10)

    col_frame = tk.Frame(left_frame, bg='#1e1e1e')
    col_frame.pack(pady=5)
    for i in range(1, 4):
        tk.Radiobutton(col_frame, text=f"CỘT {i}", variable=selected_col, value=i,
                       font=("Arial", 18, "bold"), indicatoron=0, width=10,
                       bg="#444", fg="white", selectcolor="#0066ff").pack(side=tk.LEFT, padx=10)

    tk.Label(left_frame, text="2. TRẠNG THÁI Ô VÀO (HÀNG ĐẦU)",
             fg="#ff8800", bg="#1e1e1e", font=("Arial", 16, "bold")).pack(pady=(30, 10))

    lbl_entrance = tk.Label(left_frame, text="Chưa chọn ô vào nào",
                            fg="#aaaaaa", bg="#1e1e1e", font=("Arial", 16, "italic"))
    lbl_entrance.pack(pady=10)

    tk.Button(left_frame, text="🗑 XÓA HẾT", font=("Arial", 14, "bold"),
              bg="#ff4444", fg="white", command=clear_all).pack(pady=10)

    btn_start = tk.Button(left_frame, text="START MISSION",
                          font=("Arial", 35, "bold"), bg="#ff1111", fg="white",
                          activebackground="#ff5555", command=on_start)
    btn_start.pack(side=tk.BOTTOM, fill=tk.X, pady=20)

    # --- PHẢI: 3 Ô HÀNG 1 ---
    tk.Label(right_frame, text="CHỌN Ô HÀNG ĐẦU\n(Entrance Boxes — Hàng 1)",
             fg="white", bg="#2b2b2b", font=("Arial", 14, "bold")).pack(pady=10)

    cell_frame = tk.Frame(right_frame, bg='#2b2b2b')
    cell_frame.pack(expand=True, pady=20)

    # Hiển thị cùng hướng với gui_start_panel: Cột 3 trái → Cột 1 phải
    for c in range(3, 0, -1):
        btn = tk.Button(cell_frame, text=f"C{c}-H1",
                        font=("Arial", 22, "bold"),
                        bg="#333333", fg="white",
                        width=8, height=5, relief=tk.RAISED,
                        command=lambda col=c: toggle_entrance(col))
        btn.pack(side=tk.LEFT, padx=10)
        entrance_buttons[c] = btn

    # ================= ROS + TKINTER LOOP =================
    def ros_spin():
        rclpy.spin_once(ros_node, timeout_sec=0.01)
        root.after(50, ros_spin)

    ros_spin()
    root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
