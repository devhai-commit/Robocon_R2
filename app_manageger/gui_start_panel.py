#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
import json

class GuiStartNode(Node):
    def __init__(self):
        super().__init__('gui_start_panel_node')
        self.pub = self.create_publisher(String, '/gui_start_signal', 10)

    def send_config(self, priority_col, target_boxes):
        data = {
            "command": "START",
            "priority_col": int(priority_col),
            "target_boxes": target_boxes # Format: [[1,3], [2,4]]
        }
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)
        self.get_logger().info(f"🚀 ĐÃ PHÁT LỆNH CHIẾN THUẬT: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    ros_node = GuiStartNode()
    
    # --- KHỞI TẠO CỬA SỔ ---
    root = tk.Tk()
    root.title("Robocon Strategy Control - Grid Map")
    root.geometry("1024x600")
    root.configure(bg='#1e1e1e')

    selected_col = tk.IntVar(value=2)
    target_boxes_list = []
    grid_buttons = {} # Lưu trữ các nút trên sa bàn để đổi màu

    # --- HÀM XỬ LÝ CLICK Ô SA BÀN ---
    def toggle_cell(c, r):
        btn = grid_buttons[(c, r)]
        if [c, r] in target_boxes_list:
            target_boxes_list.remove([c, r])
            btn.config(bg="#333333", fg="white", relief=tk.RAISED) # Trạng thái Tắt
        else:
            target_boxes_list.append([c, r])
            btn.config(bg="#00cc44", fg="black", relief=tk.SUNKEN) # Trạng thái Bật (Xanh lá)
        update_target_label()

    def update_target_label():
        if not target_boxes_list: 
            lbl_targets.config(text="Chưa chọn ô mục tiêu nào", fg="#aaaaaa")
        else:
            txt = " | ".join([f"({x[0]},{x[1]})" for x in target_boxes_list])
            lbl_targets.config(text=f"Mục tiêu chờ gắp: {txt}", fg="#ffcc00")

    def clear_all():
        target_boxes_list.clear()
        for (c, r), btn in grid_buttons.items():
            btn.config(bg="#333333", fg="white", relief=tk.RAISED)
        update_target_label()

    def on_start():
        ros_node.send_config(selected_col.get(), target_boxes_list)
        btn_start.config(text="🚀 ROBOT ĐANG THI ĐẤU...", bg='#555555', fg="white", state=tk.DISABLED)

    # ================= LAYOUT GIAO DIỆN =================
    left_frame = tk.Frame(root, bg='#1e1e1e')
    left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=20, pady=20)

    right_frame = tk.Frame(root, bg='#2b2b2b', bd=2, relief=tk.GROOVE)
    right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=20, pady=20)

    # --- TRÁI: ĐIỀU KHIỂN & CHỌN CỘT ---
    tk.Label(left_frame, text="1. CHỌN CỘT ƯU TIÊN DI CHUYỂN", fg="#00ffcc", bg="#1e1e1e", font=("Arial", 16, "bold")).pack(pady=10)
    col_frame = tk.Frame(left_frame, bg='#1e1e1e')
    col_frame.pack(pady=5)
    for i in range(1, 4):
        tk.Radiobutton(col_frame, text=f"CỘT {i}", variable=selected_col, value=i, 
                       font=("Arial", 18, "bold"), indicatoron=0, width=10, 
                       bg="#444", fg="white", selectcolor="#0066ff").pack(side=tk.LEFT, padx=10)

    tk.Label(left_frame, text="2. TRẠNG THÁI MỤC TIÊU", fg="#ffcc00", bg="#1e1e1e", font=("Arial", 16, "bold")).pack(pady=(30, 10))
    lbl_targets = tk.Label(left_frame, text="Chưa chọn ô mục tiêu nào", fg="#aaaaaa", bg="#1e1e1e", font=("Arial", 16, "italic"))
    lbl_targets.pack(pady=10)

    tk.Button(left_frame, text="🗑 XÓA HẾT MỤC TIÊU", font=("Arial", 14, "bold"), bg="#ff4444", fg="white", command=clear_all).pack(pady=10)

    btn_start = tk.Button(left_frame, text="START MISSION", font=("Arial", 35, "bold"), bg="#ff1111", fg="white", activebackground="#ff5555", command=on_start)
    btn_start.pack(side=tk.BOTTOM, fill=tk.X, pady=20)

    # --- PHẢI: BẢN ĐỒ LƯỚI (GRID MAP) ---
    tk.Label(right_frame, text="BẢN ĐỒ CHỌN HỘP NHANH\n(Cột 1, Hàng 1 ở Dưới-Phải)", fg="white", bg="#2b2b2b", font=("Arial", 14, "bold")).pack(pady=10)
    
    grid_frame = tk.Frame(right_frame, bg="#2b2b2b")
    grid_frame.pack(expand=True)

    # Tạo ma trận 4 Hàng x 3 Cột
    # Hàng 1 ở dưới cùng (r = 1), Hàng 4 ở trên cùng (r = 4)
    # Cột 1 ở bên phải (c = 1), Cột 3 ở bên trái (c = 3)
    for r in range(4, 0, -1):      # Chạy từ Hàng 4 lùi về Hàng 1
        for c in range(3, 0, -1):  # Chạy từ Cột 3 lùi về Cột 1
            tk_row = 4 - r         # Map tọa độ R sang hệ của Tkinter grid (0->3)
            tk_col = 3 - c         # Map tọa độ C sang hệ của Tkinter grid (0->2)
            
            btn = tk.Button(grid_frame, text=f"C{c}-H{r}", font=("Arial", 18, "bold"),
                            bg="#333333", fg="white", width=8, height=3, relief=tk.RAISED)
            
            # Sử dụng tham số mặc định trong lambda để tránh lỗi binding biến trong vòng lặp
            btn.config(command=lambda col=c, row=r: toggle_cell(col, row))
            btn.grid(row=tk_row, column=tk_col, padx=5, pady=5)
            
            grid_buttons[(c, r)] = btn # Lưu vào dict để đổi màu

    # ================= VÒNG LẶP ROS & TKINTER =================
    def ros_spin():
        rclpy.spin_once(ros_node, timeout_sec=0.01)
        root.after(50, ros_spin)

    ros_spin()
    root.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    