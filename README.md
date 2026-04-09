# 📚 LIB-Buddy: Autonomous Library Assistant Robot
An intelligent library assistant robot 'LIB-Buddy' designed for autonomous shelf-tracking, book scanning, and automated navigation using ROS 2 Humble.

![ROS2](https://img.shields.io/badge/ROS2-Workspace-22314E?style=flat-square&logo=ros)
![Python](https://img.shields.io/badge/Python-3.8+-3776AB?style=flat-square&logo=python)
![OpenCV](https://img.shields.io/badge/OpenCV-Vision-5C3EE8?style=flat-square&logo=opencv)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-Edge%20Computing-A22846?style=flat-square&logo=raspberrypi)
![Arduino](https://img.shields.io/badge/Arduino-Mega%202560-00979D?style=flat-square&logo=arduino)

**LIB-Buddy**은 PC, Raspberry Pi, Arduino Mega 간의 유기적인 분산 제어(Distributed System)를 통해 도서관 서가를 자율 주행하며 책을 인식하고 관리하는 지능형 로봇 프로젝트입니다.

<br>

## ⚙️ 로봇 회로도
<img width="1739" height="1414" alt="image" src="https://github.com/user-attachments/assets/11dec041-ff6d-449f-8fe6-da358a776a9c" />

## 🌟 System Architecture & Features
<img width="1600" height="789" alt="image" src="https://github.com/user-attachments/assets/504f83eb-4eee-4d5a-b3e1-5893f20bbe37" />

<br>

## 📂 Repository Structure

```text
LIB-Autonomous-Library-Robot/
├── lib_arduino_mega/                   # 하위 하드웨어 제어용 아두이노 펌웨어
│   └── robot_final.pt/                 # 아두이노 스케치 디렉토리
│       └── robot_final.pt.ino          # DC 모터 및 하드웨어 제어 메인 소스코드
│
├── lib_pc_ws/                          # PC용 ROS2 워크스페이스 (Master)
│   └── src/
│       ├── integrated_control/         # 로봇 통합 제어 및 시퀀스 관리 패키지
│       └── robot_gui/                  # 관리자용 관제 대시보드 UI 패키지
│
├── lib_rasp_ws/                        # 라즈베리파이용 ROS2 워크스페이스 (Edge)
│   └── src/
│       ├── robot_bridge/               # ROS2 <-> Arduino 시리얼 브릿지 통신 패키지
│       └── robot_vision/               # ArUco 마커 인식 및 이미지 처리 패키지
│
├── requirements.txt                    # 파이썬 의존성 패키지 목록
└── README.md                           # 프로젝트 설명서
```

## 🎥작동 시연 영상
> ![1](https://github.com/user-attachments/assets/ed27c503-5c1a-44fe-87ec-40ff51337b2a)
> ![1_2](https://github.com/user-attachments/assets/0d01e76c-b943-46f7-a257-1eea98ed3044)
> ![1_3](https://github.com/user-attachments/assets/38217423-21a3-4e31-989e-cf9e8eb3001f)
> ![1_4](https://github.com/user-attachments/assets/1ecca0a9-ce9c-4fe2-848f-2b56fdee9a4a)
> ![1_6](https://github.com/user-attachments/assets/e7eda8ea-adec-41e6-9068-3da021190153)
> ![1_7](https://github.com/user-attachments/assets/d394bcbc-2132-4254-8422-b7d53a8489a8)


