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
        
        # ---------------------------------------------------------
        # [1. 통신 설정] ROS2 퍼블리셔 초기화
        # ---------------------------------------------------------
        # 로봇 바퀴 속도 제어 (Twist 메시지: 선속도, 각속도)
        self.pub_vel = self.create_publisher(Twist, '/lib_cmd_vel', 10)
        # 로봇 팔 제어 명령 (String 메시지: "관절번호:각도" 형식)
        self.pub_arm = self.create_publisher(String, '/arm_command', 10)
        # 단계별 정지 알림 (스캔 완료 등을 알림)
        self.pub_notify = self.create_publisher(String, '/stop_notification', 10)
        
        # ---------------------------------------------------------
        # [2. 하드웨어 및 주행 파라미터 설정]
        # ---------------------------------------------------------
        # 조향 방향 부호 (1: 정방향, -1: 역방향, 하드웨어 배선에 따라 다름)
        self.STEERING_SIGN = 1 
        
        # [후진 보정] 후진 시 왼쪽 바퀴 속도 미세 조정 (직진성 보정)
        # 음수 값: 왼쪽 바퀴 감속 / 양수 값: 왼쪽 바퀴 가속
        self.backward_left_trim = -0.01

        # [주행 튜닝]
        # floor_deadzone: 라인 트레이싱 시 중앙 오차 허용 범위 (픽셀 단위)
        self.floor_deadzone = 1
        # steering_divisor: 조향 감도 (값이 클수록 핸들을 부드럽게/적게 꺾음)
        self.steering_divisor = 300.0 
        
        # ---------------------------------------------------------
        # [3. 비전(ArUco) 설정]
        # ---------------------------------------------------------
        # 바닥 주행용 마커 사전 (5x5, 250)
        self.dict_base = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        # 책장 인식용 마커 사전 (5x5, 1000)
        self.dict_shelf = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.parameters = aruco.DetectorParameters_create()
        
        # ---------------------------------------------------------
        # [4. 상태 관리 변수]
        # ---------------------------------------------------------
        self.run_id = 0  # 스레드 안전성을 위한 실행 ID (긴급 정지 시 ID 변경으로 기존 작업 무효화)
        self.reset_internal_states() # 내부 변수 초기화 함수 호출
        
        # ---------------------------------------------------------
        # [5. 로봇 팔 포즈 정의] (각도: [Base, Shoulder, Elbow, Wrist-Roll, Wrist-Pitch, Gripper])
        # ---------------------------------------------------------
        self.POSE_START = [180, 90, 180, 60, 120, 90] # 시작 자세
        self.POSE_PICK  = [180, 30, 160, 67, 148, 90] # 책 꺼내기/스캔 자세
        self.POSE_FINAL = [90, 120, 180, 55, 100, 90] # 복귀 후 정리 자세
        
        # ---------------------------------------------------------
        # [6. GUI 및 구독 설정]
        # ---------------------------------------------------------
        # GUI 이미지 표시용 버퍼
        self.latest_floor_frame = None
        self.latest_arm_frame = None
        
        self.setup_gui() # Tkinter GUI 구성
        
        # 토픽 구독
        # 바닥 카메라 이미지 구독
        self.create_subscription(CompressedImage, '/camera_base/image_raw/compressed', self.base_image_callback, 10)
        # 팔(책장) 카메라 이미지 구독
        self.create_subscription(CompressedImage, '/camera_arm/image_raw/compressed', self.arm_image_callback, 10)
        # 외부 제어 명령(start/stop) 구독
        self.create_subscription(String, '/scan_command', self.scan_command_callback, 10)
        
        self.get_logger().info(f"🚀 RIB Master Node Started (Steering Sign: {self.STEERING_SIGN})")

    def reset_internal_states(self):
        """로봇의 주행 및 작업 상태를 초기화하는 함수"""
        self.current_step = "IDLE"      # 현재 상태 (대기 중)
        self.msg_count = 1              # 메시지 카운터
        self.align_deadzone = 10.0      # 정렬 허용 오차
        self.book_offset = 70.0         # 책장 마커로부터의 오프셋
        self.is_aligned = False         # 정렬 완료 여부 플래그
        self.align_stable_start_time = None # 정렬 안정화 시간 측정용
        self.VERIFICATION_DURATION = 3.0    # 정렬 완료 판단 기준 시간 (초)
        self.last_pulse_time = 0.0      # 펄스 이동 시간 기록용
        self.current_arm_base_angle = 90 # 현재 팔 베이스 각도

    def setup_gui(self):
        """Tkinter를 이용한 제어 GUI 구성"""
        self.root = tk.Tk(); self.root.title("RIB Robot Control Console")
        frame = ttk.LabelFrame(self.root, text="[ RIB Project Control ]"); frame.pack(padx=20, pady=20)
        
        # 상태 표시 라벨
        self.status_label = ttk.Label(frame, text="상태: 대기 중", font=("Arial", 12, "bold")); self.status_label.pack(pady=10)
        
        # 버튼 구성 (시작, 정지)
        ttk.Button(frame, text="프로세스 시작 (s)", command=self.start_process).pack(fill="x", pady=5)
        ttk.Button(frame, text="긴급 정지 (Space)", command=self.stop_all).pack(fill="x", pady=5)
        
        # 키보드 단축키 바인딩
        self.root.bind('<s>', lambda e: self.start_process()); self.root.bind('<space>', lambda e: self.stop_all())
        
        # OpenCV 이미지 갱신 루프 시작
        self.root.after(30, self.update_cv_windows)

    def update_cv_windows(self):
        """OpenCV 이미지를 GUI 스레드에서 안전하게 표시"""
        if self.latest_floor_frame is not None:
            cv2.imshow("Floor Vision (Base)", self.latest_floor_frame)
        if self.latest_arm_frame is not None:
            cv2.imshow("Arm Vision (Shelf)", self.latest_arm_frame)
        cv2.waitKey(1)
        self.root.after(30, self.update_cv_windows)

    def base_image_callback(self, msg):
        """
        [핵심 로직] 바닥 카메라 콜백 함수 (주행 제어)
        - 역할: 라인 트레이싱, 마커 인식, 조향 제어, 목적지 도착 판단
        - 특징: 마커 중심점 추적, 원거리 마커 우선 순위 적용
        """
        try:
            # 이미지 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            h, w = frame.shape[:2]; cx = w / 2
            marker_status = "Searching Marker..."

            # 주행 중일 때만 로직 수행 (FWD: 전진, BWD: 후진)
            if "FWD" in self.current_step or "BWD" in self.current_step:
                is_fwd = "FWD" in self.current_step
                is_bwd = "BWD" in self.current_step

                # 방향에 따른 임계값 및 속도 설정
                current_threshold = 45.0 if is_fwd else 50.0 # 마커 크기 기준 도착 판정값
                vx = 0.55 if is_fwd else -0.55             # 전진/후진 속도
                az = 0.0; arrival_detected = False
                
                # 현재 목표 ID와 추적할 앵커 리스트 가져오기
                dest_id, anchors = self.get_navigation_targets() 
                
                # ArUco 마커 검출
                corners, ids, _ = aruco.detectMarkers(frame, self.dict_base, parameters=self.parameters)
                
                if ids is not None:
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    f_ids = ids.flatten()
                    
                    # [앵커 탐색 순서 결정]
                    # 후진 시에는 도착지(dest_id)보다 멀리 있는 마커를 먼저 보게 하여 조향 안정성 확보
                    search_order = anchors
                    if is_bwd:
                        search_order = sorted(anchors, key=lambda x: x == dest_id) 

                    # 지정된 순서대로 마커 탐색
                    active_anchor = -1
                    for anc in search_order:
                        if anc in f_ids:
                            active_anchor = anc
                            break 
                    
                    if active_anchor != -1:
                        # 추적할 마커 찾음
                        idx = np.where(f_ids == active_anchor)[0][0]
                        
                        # [마커 중심점 계산] 4개 모서리의 평균값 사용 (정확도 향상)
                        c = corners[idx][0]
                        marker_x = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4
                        marker_y = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4
                        
                        # [시각화] 인식된 중심점 빨간색 원 표시
                        cv2.circle(frame, (int(marker_x), int(marker_y)), 8, (0, 0, 255), -1)
                        # 화면 중앙과 마커 중심을 잇는 노란 선 표시 (조향 오차)
                        cv2.line(frame, (int(cx), int(h/2)), (int(marker_x), int(marker_y)), (0, 255, 255), 2)

                        # 거리 계산 (마커 크기 측정용, 상단 모서리 길이)
                        dist = np.linalg.norm(c[0] - c[1])
                        # 오차 계산 (마커 X좌표 - 화면 중앙)
                        ex = marker_x - cx
                        
                        marker_status = f"Tracking ID:{active_anchor} Size:{int(dist)}/{int(current_threshold)}"
                        
                        # [조향 제어] 데드존을 벗어난 경우 조향값 계산
                        if abs(ex) > self.floor_deadzone: 
                            raw_az = float(ex / self.steering_divisor) # 오차를 감도로 나누어 각속도 산출
                            if is_fwd:
                                az = -(raw_az) * self.STEERING_SIGN # 전진 시 반대 방향 보정
                            else:
                                az = (raw_az) * self.STEERING_SIGN  # 후진 시 정방향 보정
                    
                    # [목적지 도착 판정]
                    if dest_id in f_ids:
                        idx = np.where(f_ids == dest_id)[0][0]
                        dist = np.linalg.norm(corners[idx][0][0] - corners[idx][0][1])
                        
                        # ID 1번(코너) 도착 시 ID 2번 시퀀스로 즉시 전환 (멈추지 않음)
                        if dest_id == 1 and is_fwd:
                            self.get_logger().info("ID 1 Passed (FWD) - Switching to ID 2 Sequence")
                            self.fwd_to_id2(); return 

                        # ID 0번(최종 복귀) 도착 시
                        if dest_id == 0 and self.current_step == "BWD_TO_ID0":
                            self.get_logger().info(f"ID 0 Detected (Size: {int(dist)}) - Final Sequence Initiated")
                            self.handle_base_arrival(dest_id); arrival_detected = True
                        
                        # 일반적인 도착 (마커 크기가 임계값보다 커지면 도착으로 간주)
                        elif is_fwd:
                            if dist > current_threshold:
                                self.handle_base_arrival(dest_id); arrival_detected = True
                        else:
                            if dist > current_threshold:
                                self.handle_base_arrival(dest_id); arrival_detected = True
                
                # 도착하지 않았으면 계산된 속도로 주행 명령 전송
                if not arrival_detected: self.send_velocity(vx, az)
            
            # 디버깅용 텍스트 및 라인 표시
            cv2.line(frame, (int(cx), 0), (int(cx), h), (255, 0, 0), 1)
            cv2.putText(frame, f"Step: {self.current_step}", (10, 30), 1, 1.2, (255, 255, 255), 2)
            cv2.putText(frame, marker_status, (10, 60), 1, 1.2, (0, 255, 0), 2)
            self.latest_floor_frame = frame
        except: pass

    def arm_image_callback(self, msg):
        """
        [책장 정렬 로직] 팔 카메라 콜백 함수
        - 역할: 책장 마커를 인식하여 로봇의 미세 위치 조정 (Aligning)
        """
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
                    # 책장 층별 마커 ID 세트 정의 (4층->1층 순서 등 상황에 따라 다름)
                    rows = [400, 100, 200, 300] if self.msg_count > 3 else [100, 200, 300, 400]
                    
                    # 마커 조합에 따라 목표 중심점(tx) 계산
                    for r in rows:
                        x01, x02, x03, x04 = r+1, r+2, r+3, r+4
                        if x02 in f_ids and x03 in f_ids: # 중앙 2개 마커가 보일 때
                            tx = (np.mean(corners[np.where(f_ids == x02)[0][0]][0], axis=0)[0] + 
                                  np.mean(corners[np.where(f_ids == x03)[0][0]][0], axis=0)[0]) / 2
                            target_info = f"Row {r}: Center Pair"; break
                        # ... (기타 마커 조합 로직 생략: 좌/우 오프셋 등) ...
                        elif x02 in f_ids:
                             tx = np.mean(corners[np.where(f_ids == x02)[0][0]][0], axis=0)[0] + self.book_offset
                             target_info = f"Row {r}: R-Offset"; break

                if tx is not None:
                    ex = tx - cx # 중심 오차
                    cv2.line(frame, (int(tx), 0), (int(tx), h), (0, 255, 0), 2)
                    
                    # [펄스 이동] 오차가 크면 조금씩 움직여서 맞춤 (헌팅 방지)
                    if abs(ex) > self.align_deadzone:
                        if curr_time - self.last_pulse_time > 1.0: 
                            self.pulse_move(0.12 if ex > 0 else -0.12)
                            self.last_pulse_time = curr_time
                        target_info = f"Pulsing (Err:{int(ex)})" 
                        self.align_stable_start_time = None
                    else:
                        # [정렬 완료 판단] 오차 범위 내에서 일정 시간(VERIFICATION_DURATION) 유지 시 완료
                        if self.align_stable_start_time is None: self.align_stable_start_time = time.time()
                        stable_dur = time.time() - self.align_stable_start_time
                        target_info = f"STABLE: {round(stable_dur, 1)}s" 
                        if stable_dur >= self.VERIFICATION_DURATION: self.is_aligned = True
                else: self.align_stable_start_time = None
                
            # 디버깅 화면 갱신
            cv2.line(frame, (int(cx), 0), (int(cx), h), (255, 0, 0), 1)
            cv2.putText(frame, target_info, (10, 30), 1, 1.5, (0, 255, 255), 2)
            self.latest_arm_frame = frame
        except: pass

    def send_velocity(self, lx, az):
        """[속도 명령 전송] 선속도(lx)와 각속도(az)를 Twist 메시지로 발행"""
        msg = Twist()
        msg.linear.x = float(lx)

        # 1. 팔 무게중심 보정 (팔이 펴져 있을 때 직진성 보정값 적용)
        steering_trim = -0.0145 if (lx != 0 and self.current_arm_base_angle >= 170) else 0.0

        # 2. [후진 보정] 후진 시 왼쪽 바퀴 속도 미세 조정
        if lx < 0:
            steering_trim += self.backward_left_trim

        msg.angular.z = float(az) + steering_trim
        self.pub_vel.publish(msg)

    def pulse_move(self, speed):
        """짧게 움직이고 멈추는 펄스 구동 (미세 조정용)"""
        def task(sid):
            if self.run_id != sid: return 
            self.send_velocity(speed, 0.0); time.sleep(0.15); self.send_velocity(0.0, 0.0)
        threading.Thread(target=task, args=(self.run_id,), daemon=True).start()

    def handle_base_arrival(self, m_id):
        """목적지 도착 시 처리 로직"""
        self.send_velocity(0.0, 0.0) # 정지
        
        # 전진 중 ID 4 도착 -> 팔 펴기 준비
        if self.current_step == "FWD_TO_ID4" and m_id == 4:
            self.current_step = "ARM_CHANGING_FOR_SHELF"; self.reached_end(); return
        
        # 후진 중 ID 1 도착 -> 정렬 없이 ID 0으로 이동
        if self.current_step == "BWD_TO_ID1" and m_id == 1:
            self.get_logger().info("ID 1 Passed (BWD) - Switching to ID 0 Sequence")
            self.bwd_to_id0(); return

        # 후진 중 ID 0 도착 (최종 복귀) -> 팔 정리 후 종료
        if self.current_step == "BWD_TO_ID0" and m_id == 0:
            self.get_logger().info("ID 0 Arrival. Folding Arm for Final State.")
            self.current_step = "ARM_FOLDING"
            self.send_arm_pose_sequentially(self.POSE_FINAL, self.finish_mission, self.run_id)
            return

        # 그 외(중간 책장) 도착 -> 정렬(Aligning) 시퀀스 시작
        self.is_aligned = False; self.align_stable_start_time = None
        prev_step = self.current_step; self.current_step = "ALIGNING"
        self.update_status(f"ID {m_id} 정렬 중...", "blue")
        
        next_f = self.get_next_action(prev_step) # 다음 단계 함수 가져오기
        threading.Thread(target=self.wait_for_align, args=(m_id, next_f, self.run_id), daemon=True).start()

    def wait_for_align(self, m_id, next_f, sid):
        """정렬이 완료될 때까지 대기하는 스레드 함수"""
        while self.current_step == "ALIGNING":
            if self.run_id != sid: return 
            if self.is_aligned: break 
            time.sleep(0.1)
        # 정렬 완료 후 알림 전송 및 다음 동작 수행
        if self.current_step == "ALIGNING":
            threading.Timer(1.0, self.notify_and_wait, [next_f, sid]).start()

    def notify_and_wait(self, next_f, sid):
        """정렬 완료 알림을 보내고 잠시 대기 후 다음 동작 수행"""
        if self.run_id != sid: return
        self.pub_notify.publish(String(data=str(self.msg_count)))
        self.msg_count += 1
        time.sleep(3.0) # 스캔 등을 위한 대기 시간
        if self.run_id == sid: next_f(sid)

    def get_navigation_targets(self):
        """현재 단계에 따른 목표 ID와 참조 앵커 ID 리스트 반환"""
        nav_map = {
            # 단계명 : (목표 ID, [참조 앵커 리스트])
            "FWD_TO_ID1": (1, [1, 0]),    # 출발 시 0, 1번 모두 참조
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
        """현재 단계 완료 후 실행할 다음 단계 함수 반환"""
        actions = {
            "FWD_TO_ID1": self.fwd_to_id2, "FWD_TO_ID2": self.fwd_to_id3, "FWD_TO_ID3": self.fwd_to_id4,
            "BWD_TO_ID3": self.bwd_to_id2, "BWD_TO_ID2": self.bwd_to_id1, "BWD_TO_ID1": self.bwd_to_id0
        }
        return actions.get(prev_step, self.stop_robot_only)

    def start_process(self):
        """미션 시작 함수"""
        if self.current_step not in ["IDLE", "COMPLETED"]: return
        self.run_id += 1; self.reset_internal_states()
        self.current_step = "ARM_START"; self.update_status("미션 시작", "green")
        # 시작 팔 자세 잡고 -> 전진 시작(begin_fwd)
        self.send_arm_pose_sequentially(self.POSE_START, self.begin_fwd, self.run_id)

    def stop_all(self):
        """긴급 정지 함수"""
        self.run_id += 1; self.send_velocity(0.0, 0.0); self.reset_internal_states()
        self.update_status("긴급 정지", "red")

    def finish_mission(self, sid):
        """최종 종료 함수 (팔 정리 완료 후 호출)"""
        if self.run_id != sid: return
        self.send_velocity(0.0, 0.0)
        self.current_step = "COMPLETED"
        self.update_status("미션 완료 (복귀 및 팔 정리 끝)", "blue")

    def stop_robot_only(self): 
        """단순 정지 함수"""
        self.send_velocity(0.0, 0.0); 
        self.current_step = "COMPLETED"
        self.update_status("미션 완료 (복귀 성공)", "blue")

    # --- 상태 전이 함수들 ---
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
        """로봇 팔 각도를 순차적으로 전송하는 스레드 함수"""
        self.current_arm_base_angle = angles[0] 
        def task():
            for i, a in enumerate(angles):
                if self.run_id != sid: return 
                self.pub_arm.publish(String(data=f"{i+1}:{a}")); time.sleep(0.25)
            time.sleep(2.0)
            if self.run_id == sid: next_f(sid) 
        threading.Thread(target=task, daemon=True).start()

    def scan_command_callback(self, msg):
        """외부 제어 명령 처리 콜백"""
        cmd = msg.data.lower().strip()
        if "start" in cmd: self.start_process()
        elif "stop" in cmd: self.stop_all()

    def update_status(self, text, color):
        """GUI 상태 텍스트 업데이트"""
        self.root.after(0, lambda: self.status_label.config(text=text, foreground=color))

    def run(self):
        """노드 실행 및 GUI 루프 시작"""
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()
        self.root.mainloop()

if __name__ == '__main__':
    rclpy.init(); IntegratedLibraryController().run(); rclpy.shutdown()