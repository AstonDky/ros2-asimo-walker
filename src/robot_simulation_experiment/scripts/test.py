#!/usr/bin/python3
import math
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


JOINTS = [
    ("hip_yaw", "大腿左右拧 / 脚尖内外转"),
    ("hip_roll", "大腿左右侧摆 / 调左右重心"),
    ("hip_pitch", "大腿前后摆 / 迈腿"),
    ("knee_pitch", "膝盖弯曲 / 伸直"),
    ("ankle_roll", "脚掌左右翻 / 抗左右倒"),
    ("ankle_pitch", "脚尖上下 / 抗前后倒"),
]

DEFAULT_LEFT_DEG = [0.0, 0.0, 8.0, 20.0, 0.0, -12.0]
DEFAULT_RIGHT_DEG = [0.0, 0.0, 8.0, 20.0, 0.0, -12.0]


def deg_to_rad(x: float) -> float:
    return x * math.pi / 180.0


class JointPlaygroundGui(Node):
    def __init__(self):
        super().__init__("joint_playground_gui")

        self.pub = self.create_publisher(
            Float64MultiArray,
            "/legTargetJoints",
            10,
        )

        self.publish_hz = 20.0
        self.left_deg = list(DEFAULT_LEFT_DEG)
        self.right_deg = list(DEFAULT_RIGHT_DEG)
        self.last_published_text = ""

        self.root = tk.Tk()
        self.root.title("Asti Leg Joint Playground - /legTargetJoints")
        self.root.geometry("920x430")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.left_vars = []
        self.right_vars = []
        self.status_var = tk.StringVar(
            value="修改任意数据框后按 Enter 应用。单位：degree。程序会持续发布到 /legTargetJoints。"
        )

        self.build_ui()
        self.apply_values(show_popup=False)
        self.schedule_publish()

    def build_ui(self):
        title = ttk.Label(
            self.root,
            text="Asti 腿部关节实时控制面板",
            font=("Arial", 18, "bold"),
        )
        title.pack(pady=(12, 4))

        hint = ttk.Label(
            self.root,
            text="不要同时运行 main.py；这里直接发布 12 个腿部关节目标角。输入单位是 degree，内部自动转 radian。",
        )
        hint.pack(pady=(0, 10))

        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=16, pady=8)

        left_frame = ttk.LabelFrame(main_frame, text="左腿 Left Leg")
        right_frame = ttk.LabelFrame(main_frame, text="右腿 Right Leg")
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 8))
        right_frame.pack(side="left", fill="both", expand=True, padx=(8, 0))

        self.build_leg_panel(left_frame, self.left_vars, DEFAULT_LEFT_DEG)
        self.build_leg_panel(right_frame, self.right_vars, DEFAULT_RIGHT_DEG)

        status = ttk.Label(self.root, textvariable=self.status_var, anchor="w")
        status.pack(fill="x", padx=16, pady=(4, 12))

    def build_leg_panel(self, parent, var_list, defaults):
        header = ttk.Frame(parent)
        header.pack(fill="x", padx=8, pady=(8, 4))

        ttk.Label(header, text="参数名称", width=14, font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(header, text="改变的功能", width=30, font=("Arial", 10, "bold")).grid(row=0, column=1, sticky="w")
        ttk.Label(header, text="数据框 deg", width=14, font=("Arial", 10, "bold")).grid(row=0, column=2, sticky="w")

        for name, desc in JOINTS:
            index = len(var_list)
            row = ttk.Frame(parent)
            row.pack(fill="x", padx=8, pady=6)

            var = tk.StringVar(value=str(defaults[index]))
            var_list.append(var)

            ttk.Label(row, text=name, width=14).grid(row=0, column=0, sticky="w")
            ttk.Label(row, text=desc, width=30).grid(row=0, column=1, sticky="w")

            entry = ttk.Entry(row, textvariable=var, width=14)
            entry.grid(row=0, column=2, sticky="w")
            entry.bind("<Return>", lambda event: self.apply_values())

    def read_leg_values(self, var_list, side_name):
        values = []
        for i, var in enumerate(var_list):
            joint_name = JOINTS[i][0]
            text = var.get().strip()
            try:
                value = float(text)
            except ValueError:
                raise ValueError(f"{side_name} {joint_name} 不是数字: {text}")
            values.append(value)
        return values

    def apply_values(self, show_popup=True):
        try:
            self.left_deg = self.read_leg_values(self.left_vars, "左腿")
            self.right_deg = self.read_leg_values(self.right_vars, "右腿")
        except ValueError as exc:
            self.status_var.set(f"输入错误：{exc}")
            if show_popup:
                messagebox.showerror("输入错误", str(exc))
            return

        self.publish_once()
        self.status_var.set(
            "已应用："
            f"L={self.left_deg}, R={self.right_deg}"
        )

    def publish_once(self):
        left_rad = [deg_to_rad(x) for x in self.left_deg]
        right_rad = [deg_to_rad(x) for x in self.right_deg]

        msg = Float64MultiArray()
        msg.data = left_rad + right_rad
        self.pub.publish(msg)

        text = f"L={self.left_deg}, R={self.right_deg}"
        if text != self.last_published_text:
            self.get_logger().info(f"published leg target deg: {text}")
            self.last_published_text = text

    def schedule_publish(self):
        self.publish_once()
        delay_ms = int(1000.0 / self.publish_hz)
        self.root.after(delay_ms, self.schedule_publish)

    def on_close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = JointPlaygroundGui()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
