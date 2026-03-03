import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2, cv2.aruco as aruco
import tkinter as tk
from tkinter import ttk
import threading, time, numpy as np
import os

class IntegratedLibraryController(Node):
    def __init__(self):
        super().__init__('integrated_control_node')
        
        # [명세 1] Network: 제어 토픽명 고정
        self.pub_vel = self.create_publisher(Twist, '/rib_cmd_vel', 10)
        self.pub_arm = self.create_publisher(String, '/arm_command', 10)
        self.pub_notify = self.create_publisher(String, '/stop_notification', 10)
        
        # [설정] 하드웨어 조향 방향 (1: 정방향, -1: 역방향)
        self.STEERING_SIGN = 1 
        
        # ArUco 사전 설정
        self.dict_base = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.dict_shelf = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.parameters = aruco.DetectorParameters_create()
        
        # [명세 11] Thread Safe Stop용 ID
        self.run_id = 0 
        self.reset_internal_states()
        
        # [명세 9] Sequence: 로봇팔 포즈 정의
        self.POSE_START = [180, 90, 180, 60, 120, 90]
        self.POSE_PICK  = [180, 30, 180, 67, 178, 90] 
        
        # [명세 2] GUI Thread Safety용 프레임 버퍼
        self.latest_floor_frame = None
        self.latest_arm_frame = None
        
        self.setup_gui()
        
        # 구독 설정
        self.create_subscription(CompressedImage, '/camera_base/image_raw/compressed', self.base_image_callback, 10)
        self.create_subscription(CompressedImage, '/camera_arm/image_raw/compressed', self.arm_image_callback, 10)
        self.create_subscription(String, '/scan_command', self.scan_command_callback, 10)
        
        self.get_logger().info(f"🚀 RIB Master Node Started (Steering Sign: {self.STEERING_SIGN})")

    def reset_internal_states(self):
        """내부 상태 초기화"""
        self.current_step = "IDLE"; self.msg_count = 1
        # stop_threshold는 이제 콜백에서 동적으로 결정되므로 초기값은 의미상 둡니다.
        self.align_deadzone = 10.0; self.book_offset = 70.0; self.is_aligned = False
        self.align_stable_start_time = None; self.VERIFICATION_DURATION = 3.0; self.last_pulse_time = 0.0
        self.current_arm_base_angle = 90  

    def setup_gui(self):
        """[명세 2] GUI 구성"""
        self.root = tk.Tk(); self.root.title("RIB Robot Control Console")
        frame = ttk.LabelFrame(self.root, text="[ RIB Project Control ]"); frame.pack(padx=20, pady=20)
        self.status_label = ttk.Label(frame, text="상태: 대기 중", font=("Arial", 12, "bold")); self.status_label.pack(pady=10)
        ttk.Button(frame, text="프로세스 시작 (s)", command=self.start_process).pack(fill="x", pady=5)
        ttk.Button(frame, text="긴급 정지 (Space)", command=self.stop_all).pack(fill="x", pady=5)
        
        self.root.bind('<s>', lambda e: self.start_process()); self.root.bind('<space>', lambda e: self.stop_all())
        self.root.after(30, self.update_cv_windows)

    def update_cv_windows(self):
        """[명세 2] GUI Thread Safety"""
        if self.latest_floor_frame is not None:
            cv2.imshow("Floor Vision (Base)", self.latest_floor_frame)
        if self.latest_arm_frame is not None:
            cv2.imshow("Arm Vision (Shelf)", self.latest_arm_frame)
        cv2.waitKey(1)
        self.root.after(30, self.update_cv_windows)

    def base_image_callback(self, msg):
        """[명세 3, 4, 12] 바닥 주행: 멀티 앵커 & 동적 Threshold"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            h, w = frame.shape[:2]; cx = w / 2
            marker_status = "Searching Marker..."

            if "FWD" in self.current_step or "BWD" in self.current_step:
                is_fwd = "FWD" in self.current_step
                
                # [수정] 전진 시 90.0, 후진 시 95.0 설정
                current_threshold = 85.0 if is_fwd else 95.0
                
                vx = 0.55 if is_fwd else -0.55
                az = 0.0; arrival_detected = False
                
                dest_id, anchors = self.get_navigation_targets() 
                corners, ids, _ = aruco.detectMarkers(frame, self.dict_base, parameters=self.parameters)
                
                if ids is not None:
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    f_ids = ids.flatten()
                    
                    # 1. 앵커 탐색
                    active_anchor = -1
                    for anc in anchors:
                        if anc in f_ids:
                            active_anchor = anc
                            break 
                    
                    if active_anchor != -1:
                        idx = np.where(f_ids == active_anchor)[0][0]
                        marker_x = np.mean(corners[idx][0][:, 0])
                        dist = np.linalg.norm(corners[idx][0][0] - corners[idx][0][1])
                        ex = marker_x - cx
                        
                        # 화면에 현재 적용된 Threshold(90 or 95) 표시
                        marker_status = f"Tracking ID:{active_anchor} Size:{int(dist)}/{int(current_threshold)}"

                        if abs(ex) > 15: 
                            steering_gain = 200.0 
                            raw_az = float(ex / steering_gain)
                            if is_fwd:
                                az = -(raw_az) * self.STEERING_SIGN
                            else:
                                az = (raw_az) * self.STEERING_SIGN
                    
                    # 2. 목적지 도착 판정
                    if dest_id in f_ids:
                        idx = np.where(f_ids == dest_id)[0][0]
                        dist = np.linalg.norm(corners[idx][0][0] - corners[idx][0][1])
                        
                        # [ID 1 패스스루 - 전진 시]
                        if dest_id == 1 and is_fwd:
                            self.get_logger().info("ID 1 Passed (FWD) - Switching to ID 2 Sequence")
                            self.fwd_to_id2(); return 

                        # [수정] 도착 판정 로직에 current_threshold 적용
                        if dest_id == 0 and self.current_step == "BWD_TO_ID0":
                             # ID 0은 후진의 마지막이므로 정밀하게 95.0 체크
                             if dist > current_threshold:
                                self.stop_robot_only(); arrival_detected = True
                        elif is_fwd:
                            # 전진 (90.0)
                            if dist > current_threshold:
                                self.handle_base_arrival(dest_id); arrival_detected = True
                        else:
                            # 후진 (95.0) - 즉시 정지가 아니라 크기 체크 후 정지
                            if dist > current_threshold:
                                self.handle_base_arrival(dest_id); arrival_detected = True
                            
                if not arrival_detected: self.send_velocity(vx, az)
            
            cv2.line(frame, (int(cx), 0), (int(cx), h), (255, 0, 0), 1)
            cv2.putText(frame, f"Step: {self.current_step}", (10, 30), 1, 1.2, (255, 255, 255), 2)
            cv2.putText(frame, marker_status, (10, 60), 1, 1.2, (0, 255, 0), 2)
            self.latest_floor_frame = frame
        except: pass

    def arm_image_callback(self, msg):
        """[명세 5, 6, 7, 8, 10] 책장 정렬 로직"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            h, w = frame.shape[:2]; cx = w / 2
            target_info = f"Step: {self.current_step}"
            
            if self.current_step == "ALIGNING":
                curr_time = time.time()
                corners, ids, _ = aruco.detectMarkers(frame, self.dict_shelf, parameters=self.parameters)
                target_info = "Searching Target..."; tx = None
                
                if ids is not None:
                    f_ids = ids.flatten()
                    rows = [400, 100, 200, 300] if self.msg_count > 3 else [100, 200, 300, 400]
                    for r in rows:
                        x01, x02, x03, x04 = r+1, r+2, r+3, r+4
                        if x02 in f_ids and x03 in f_ids:
                            tx = (np.mean(corners[np.where(f_ids == x02)[0][0]][0], axis=0)[0] + 
                                  np.mean(corners[np.where(f_ids == x03)[0][0]][0], axis=0)[0]) / 2
                            target_info = f"Row {r}: Center Pair"; break
                        elif x01 in f_ids and x04 in f_ids:
                            tx = (np.mean(corners[np.where(f_ids == x01)[0][0]][0], axis=0)[0] + 
                                  np.mean(corners[np.where(f_ids == x04)[0][0]][0], axis=0)[0]) / 2
                            target_info = f"Row {r}: Aux Pair"; break
                        elif x03 in f_ids:
                            tx = np.mean(corners[np.where(f_ids == x03)[0][0]][0], axis=0)[0] - self.book_offset
                            target_info = f"Row {r}: L-Offset"; break
                        elif x02 in f_ids:
                            tx = np.mean(corners[np.where(f_ids == x02)[0][0]][0], axis=0)[0] + self.book_offset
                            target_info = f"Row {r}: R-Offset"; break

                if tx is not None:
                    ex = tx - cx
                    cv2.line(frame, (int(tx), 0), (int(tx), h), (0, 255, 0), 2)
                    if abs(ex) > self.align_deadzone:
                        if curr_time - self.last_pulse_time > 1.0: 
                            self.pulse_move(0.12 if ex > 0 else -0.12)
                            self.last_pulse_time = curr_time
                        target_info = f"Pulsing (Err:{int(ex)})" 
                        self.align_stable_start_time = None
                    else:
                        if self.align_stable_start_time is None: self.align_stable_start_time = time.time()
                        stable_dur = time.time() - self.align_stable_start_time
                        target_info = f"STABLE: {round(stable_dur, 1)}s" 
                        if stable_dur >= self.VERIFICATION_DURATION: self.is_aligned = True
                else: self.align_stable_start_time = None
                
            cv2.line(frame, (int(cx), 0), (int(cx), h), (255, 0, 0), 1)
            cv2.putText(frame, target_info, (10, 30), 1, 1.5, (0, 255, 255), 2)
            self.latest_arm_frame = frame
        except: pass

    def send_velocity(self, lx, az):
        """[명세 4] Dynamic Steering Trim"""
        msg = Twist()
        msg.linear.x = float(lx)
        steering_trim = -0.0145 if (lx != 0 and self.current_arm_base_angle >= 170) else 0.0
        msg.angular.z = float(az) + steering_trim
        self.pub_vel.publish(msg)

    def pulse_move(self, speed):
        """[명세 5] 헌팅 방지형 0.15초 펄스 이동"""
        def task(sid):
            if self.run_id != sid: return 
            self.send_velocity(speed, 0.0); time.sleep(0.15); self.send_velocity(0.0, 0.0)
        threading.Thread(target=task, args=(self.run_id,), daemon=True).start()

    def handle_base_arrival(self, m_id):
        self.send_velocity(0.0, 0.0)
        
        # [명세 9 반영] 전진 시 ID 4 도착 판정 -> 스캔 준비
        if self.current_step == "FWD_TO_ID4" and m_id == 4:
            self.current_step = "ARM_CHANGING_FOR_SHELF"; self.reached_end(); return
        
        # [수정: 명세 9 반영] 복귀 시 ID 1 도착 판정 -> 정렬 없이 즉시 ID 0으로 이동
        if self.current_step == "BWD_TO_ID1" and m_id == 1:
            self.get_logger().info("ID 1 Passed (BWD) - Switching to ID 0 Sequence")
            self.bwd_to_id0(); return

        # 그 외(ID 2, 3)는 정렬 수행
        self.is_aligned = False; self.align_stable_start_time = None
        prev_step = self.current_step; self.current_step = "ALIGNING"
        self.update_status(f"ID {m_id} 정렬 중...", "blue")
        
        next_f = self.get_next_action(prev_step)
        threading.Thread(target=self.wait_for_align, args=(m_id, next_f, self.run_id), daemon=True).start()

    def wait_for_align(self, m_id, next_f, sid):
        while self.current_step == "ALIGNING":
            if self.run_id != sid: return 
            if self.is_aligned: break 
            time.sleep(0.1)
        if self.current_step == "ALIGNING":
            threading.Timer(1.0, self.notify_and_wait, [next_f, sid]).start()

    def notify_and_wait(self, next_f, sid):
        if self.run_id != sid: return
        self.pub_notify.publish(String(data=str(self.msg_count)))
        self.msg_count += 1
        time.sleep(3.0) 
        if self.run_id == sid: next_f(sid)

    def get_navigation_targets(self):
        """[명세 12] 멀티 앵커 맵"""
        nav_map = {
            "FWD_TO_ID1": (1, [1]),       
            "FWD_TO_ID2": (2, [2, 1]),    
            "FWD_TO_ID3": (3, [3, 2]),    
            "FWD_TO_ID4": (4, [4, 3]),
            "BWD_TO_ID3": (3, [4, 3]),    
            "BWD_TO_ID2": (2, [3, 2]),
            "BWD_TO_ID1": (1, [2, 1]),
            "BWD_TO_ID0": (0, [1])
        }
        return nav_map.get(self.current_step, (-1, []))

    def get_next_action(self, prev_step):
        actions = {
            "FWD_TO_ID1": self.fwd_to_id2, "FWD_TO_ID2": self.fwd_to_id3, "FWD_TO_ID3": self.fwd_to_id4,
            "BWD_TO_ID3": self.bwd_to_id2, "BWD_TO_ID2": self.bwd_to_id1, "BWD_TO_ID1": self.bwd_to_id0
        }
        return actions.get(prev_step, self.stop_robot_only)

    def start_process(self):
        if self.current_step not in ["IDLE", "COMPLETED"]: return
        self.run_id += 1; self.reset_internal_states()
        self.current_step = "ARM_START"; self.update_status("미션 시작", "green")
        self.send_arm_pose_sequentially(self.POSE_START, self.begin_fwd, self.run_id)

    def stop_all(self):
        self.run_id += 1; self.send_velocity(0.0, 0.0); self.reset_internal_states()
        self.update_status("긴급 정지", "red")

    def stop_robot_only(self): self.send_velocity(0.0, 0.0); self.current_step = "COMPLETED"

    def begin_fwd(self, sid=None): self.current_step = "FWD_TO_ID1"
    def fwd_to_id2(self, sid=None): self.current_step = "FWD_TO_ID2"
    def fwd_to_id3(self, sid=None): self.current_step = "FWD_TO_ID3"
    def fwd_to_id4(self, sid=None): self.current_step = "FWD_TO_ID4"
    def reached_end(self): self.send_arm_pose_sequentially(self.POSE_PICK, self.begin_bwd, self.run_id)
    def begin_bwd(self, sid=None): self.current_step = "BWD_TO_ID3" 
    def bwd_to_id2(self, sid=None): self.current_step = "BWD_TO_ID2"
    def bwd_to_id1(self, sid=None): self.current_step = "BWD_TO_ID1"
    def bwd_to_id0(self, sid=None): self.current_step = "BWD_TO_ID0"

    def send_arm_pose_sequentially(self, angles, next_f, sid):
        self.current_arm_base_angle = angles[0] 
        def task():
            for i, a in enumerate(angles):
                if self.run_id != sid: return 
                self.pub_arm.publish(String(data=f"{i+1}:{a}")); time.sleep(0.25)
            time.sleep(2.0)
            if self.run_id == sid: next_f(sid) 
        threading.Thread(target=task, daemon=True).start()

    def scan_command_callback(self, msg):
        cmd = msg.data.lower().strip()
        if "start" in cmd: self.start_process()
        elif "stop" in cmd: self.stop_all()

    def update_status(self, text, color):
        self.root.after(0, lambda: self.status_label.config(text=text, foreground=color))

    def run(self):
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()
        self.root.mainloop()

if __name__ == '__main__':
    rclpy.init(); IntegratedLibraryController().run(); rclpy.shutdown()