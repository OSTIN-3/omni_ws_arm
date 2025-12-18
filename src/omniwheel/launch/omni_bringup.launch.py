import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 설정
    pkg_omniwheel = get_package_share_directory('omniwheel')
    
    # [주석처리] 라이다 패키지 경로는 지금 필요 없으므로 주석
    # pkg_ldlidar = get_package_share_directory('ldlidar_sl_ros2')

    # 2. URDF 파일 설정
    urdf_file_name = 'ostin_3.urdf'
    urdf_path = os.path.join(pkg_omniwheel, 'urdf', urdf_file_name)

    # URDF 파일 읽기 (없으면 에러 나니 파일은 있어야 함)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. Robot State Publisher (로봇 모델링 정보 발행)
    # RViz에서 로봇 모양을 보기 위해 이건 켜두는 게 좋습니다.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 4. ⭐ OmniRun 노드 (메인 드라이버)
    # STM32와 통신하며 Odom, TF, Sensor 데이터를 처리합니다.
    omnirun_node = Node(
        package='omniwheel',
        executable='omnirun',
        name='omnirun_node',
        output='screen'
    )

    # 5. [주석처리] Arm Controller 노드 (로봇팔)
    # arm_node = Node(
    #     package='omniwheel',
    #     executable='arm_controller',
    #     name='arm_controller',
    #     output='screen'
    # )

    # 6. [주석처리] LiDAR 드라이버
    # lidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ldlidar, 'launch', 'ld19.launch.py')
    #     )
    # )

    return LaunchDescription([
        rsp_node,       # 로봇 모델
        omnirun_node,   # 하체 제어 (STM32)
        # arm_node,     # [주석처리] 상체 제어 제외
        # lidar_launch, # [주석처리] 라이다 센서 제외
    ])