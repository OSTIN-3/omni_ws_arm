
https://github.com/user-attachments/assets/8d668d40-0851-4a6f-91bb-210488451d53


OSTIN-3 Omni-Wheel Mobile Manipulator
OSTIN-3는 ROS 2 기반의 3륜 옴니휠(Omni-wheel) 이동 로봇 플랫폼입니다. LiDAR를 활용한 자율 주행(SLAM/Navigation) 기능과 6축(추정) 로봇팔 제어 기능을 통합하여, 이동과 작업이 모두 가능한 모바일 매니퓰레이터 시스템을 구축하는 것을 목표로 합니다.

📦 패키지 구성 (Packages)
이 워크스페이스는 다음과 같은 주요 패키지들로 구성되어 있습니다.

1. omniwheel (Core Package)
설명: 로봇의 핵심 제어 및 구동을 담당하는 패키지입니다.


주요 노드:

omnirun: 하드웨어(ESP32/STM32)와 통신하여 3개의 옴니휠 모터와 IMU 데이터를 처리합니다.

arm_controller: STS3215 시리얼 버스 서보 모터를 사용한 로봇팔을 제어합니다.

주요 기능:

Robot State Publisher (ostin_3.urdf) 실행

Robot Localization (EKF)를 통한 오도메트리(Odometry) 융합

전체 시스템 Bringup

2. agv_mission_control
설명: Nav2(Navigation2) 스택을 활용하여 로봇의 상위 레벨 미션 및 경로 추종을 관리합니다.

의존성: nav2_simple_commander, geometry_msgs

3. ldlidar_sl_ros2
설명: LDRobot의 LD14 LiDAR 센서 드라이버입니다.

기능: 주변 환경을 스캔하여 SLAM 및 내비게이션에 필요한 레이저 스캔 데이터를 발행합니다.

4. embedded_src
설명: 로봇의 하위 제어기를 위한 펌웨어 소스 코드입니다.

하드웨어: ESP32 / STM32

라이브러리: SCServo (로봇팔), MPU6050_light (IMU), AdaptiveGripper 등

⚙️ 하드웨어 사양 (Hardware Specification)
Platform: 3-Wheeled Omnidirectional Robot (Holonomic Drive)

Lidar: LDRobot LD14 (연결: /dev/ttyUSB0 or similar)

Arm Actuators: Feetech STS3215 Serial Bus Servos

IMU: MPU6050

Controller: ROS 2 (Host PC) <-> ESP32/STM32 (Microcontroller)

🛠️ 설치 및 빌드 (Installation & Build)
요구 사항 (Prerequisites)

ROS 2 (Humble 또는 Foxy 권장)

Python 3

pyserial 패키지 (pip install pyserial)

빌드 (Build)

Bash

# 워크스페이스 루트 디렉토리에서
colcon build --symlink-install
source install/setup.bash
🚀 실행 방법 (Usage)
1. 로봇 구동 (Bringup)
로봇의 센서, 모터 드라이버, TF 변환, EKF 등을 한 번에 실행합니다.

Bash

ros2 launch omniwheel omni_bringup.launch.py
실행되는 노드들:

robot_state_publisher: URDF 모델 로드 (ostin_3.urdf)

omnirun_node: 옴니휠 주행 제어

arm_controller_node: 로봇팔 제어

ld14.launch.py: LiDAR 드라이버 실행

ekf_filter_node: 센서 퓨전 (IMU + Wheel Odometry)

2. 내비게이션 및 지도 (Navigation)
(추가 설정 필요 시 기술) 지도를 로드하고 내비게이션을 수행하려면 다음 명령어를 사용합니다. (예시)

Bash

ros2 launch omniwheel map_load.launch.py
📝 라이선스 (License)
omniwheel: MIT License

agv_mission_control: TODO
