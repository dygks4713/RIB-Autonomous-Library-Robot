import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk, messagebox
import threading

class Ros2RobotGUI(Node):
    def __init__(self):
        super().__init__('gui_teleop_node')
        self.wheel_pub = self.create_publisher(Twist, '/lib_cmd_vel', 10)
        self.arm_pub = self.create_publisher(String, '/arm_command', 10)
        
        self.root = tk.Tk()
        self.root.title("Robot Control Center (Fixed Speed 30)")
        
        self.motor_names = ["Base", "Shoulder", "Elbow", "Wrist R", "Wrist P", "Gripper"]
        self.neutrals = [90, 120, 180, 60, 100, 90] # 아두이노 초기 설정과 일치시킴
        self.entries = []
        
        self.create_widgets()
        self.setup_key_bindings()

    def create_widgets(self):
        # 로봇팔 섹션
        arm_frame = ttk.LabelFrame(self.root, text="[ 로봇팔 제어 ]")
        arm_frame.pack(padx=20, pady=10, fill="x")
        
        for i in range(6):
            frame = ttk.Frame(arm_frame)
            frame.pack(padx=10, pady=2, fill="x")
            ttk.Label(frame, text=f"{self.motor_names[i]} (ID:{i+1})", width=12).pack(side="left")
            
            entry = ttk.Entry(frame, width=8)
            entry.insert(0, str(self.neutrals[i]))
            entry.pack(side="left", padx=5)
            self.entries.append(entry)
            
            ttk.Button(frame, text="전송", command=lambda idx=i: self.send_arm_topic(idx)).pack(side="left")

        # 주행 섹션
        wheel_frame = ttk.LabelFrame(self.root, text="[ 주행 제어 (WASD) ]")
        wheel_frame.pack(padx=20, pady=10, fill="x")
        
        self.status_lbl = ttk.Label(wheel_frame, text="상태: 정지", font=("Arial", 11, "bold"))
        self.status_lbl.pack(pady=5)

        # 조종 버튼 배치
        btn_grid = ttk.Frame(wheel_frame)
        btn_grid.pack(pady=10)
        
        ttk.Button(btn_grid, text="W", command=lambda: self.pub_cmd(0.5, 0.0)).grid(row=0, column=1)
        ttk.Button(btn_grid, text="A", command=lambda: self.pub_cmd(0.0, 1.0)).grid(row=1, column=0)
        ttk.Button(btn_grid, text="S", command=lambda: self.pub_cmd(-0.5, 0.0)).grid(row=1, column=1)
        ttk.Button(btn_grid, text="D", command=lambda: self.pub_cmd(0.0, -1.0)).grid(row=1, column=2)
        ttk.Button(btn_grid, text="STOP (Space)", command=lambda: self.pub_cmd(0.0, 0.0)).grid(row=2, column=0, columnspan=3, sticky="we", pady=5)

    def pub_cmd(self, lx, az):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(lx), float(az)
        self.wheel_pub.publish(msg)
        
        # 상태 업데이트
        if lx > 0: txt = "전진"
        elif lx < 0: txt = "후진"
        elif az > 0: txt = "좌회전"
        elif az < -0: txt = "우회전"
        else: txt = "정지"
        self.status_lbl.config(text=f"상태: {txt}")

    def send_arm_topic(self, idx):
        try:
            val = int(self.entries[idx].get())
            msg = String(data=f"{idx+1}:{val}")
            self.arm_pub.publish(msg)
        except:
            messagebox.showerror("Error", "숫자만 입력하세요.")

    def setup_key_bindings(self):
        # 소문자/대문자 모두 대응
        self.root.bind('<w>', lambda e: self.pub_cmd(0.5, 0.0))
        self.root.bind('<s>', lambda e: self.pub_cmd(-0.5, 0.0))
        self.root.bind('<a>', lambda e: self.pub_cmd(0.0, 1.0))
        self.root.bind('<d>', lambda e: self.pub_cmd(0.0, -1.0))
        self.root.bind('<space>', lambda e: self.pub_cmd(0.0, 0.0))

    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    node = Ros2RobotGUI()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()