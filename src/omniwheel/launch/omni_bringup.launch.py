import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
# os 모듈이 누락되어 있었으므로 추가합니다.
# from launch_ros.actions import PushRosNamespace # 필요 없음

def generate_launch_description():
    # ----------------------------------------------------
    # 1. 패키지 경로 및 설정 파일 경로 설정
    # ----------------------------------------------------
    pkg_omniwheel = get_package_share_directory('omniwheel')
    pkg_ldlidar = get_package_share_directory('ldlidar_sl_ros2')
    
    # URDF 파일이 있는 패키지 경로를 'omniwheel'로 지정
    pkg_description = pkg_omniwheel 

    # 2. URDF 파일 경로 설정 (경로 중복 제거)
    urdf_file_name = 'ostin_3.urdf'
    urdf_path = os.path.join(
        pkg_description, # omniwheel/share/omniwheel/
        'urdf',          # urdf/
        urdf_file_name   # ostin_3.urdf
        # 이 경로가 최종적으로 /home/jdamr/OSTIN-3-SLAM/install/omniwheel/share/omniwheel/urdf/ostin_3.urdf 를 가리킵니다.
    )
    
    # 3. 로봇 상태 퍼블리셔 (RSP) 설정
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 4. Odometry/IMU 브리지 노드 설정
    # omnirun.py 내부에서 파라미터를 선언하지 않았으므로, 파라미터 전달을 제거합니다.
    bridge_node = Node(
        package='omniwheel',
        executable='omnirun',
        name='omnibridge_node',
        output='screen'
        # 파라미터 제거: omnirun.py 파일에서 하드코딩된 시리얼 포트를 사용합니다.
        # parameters=[{'publish_tf': False}, ...]
    )

    # 5. 로봇팔 제어 노드 (Arm Controller)
    # 로봇팔 노드도 omnirun과 유사하게 파라미터 전달 대신 내부 하드코딩을 사용할 가능성이 높습니다.
    arm_node = Node(
        package='omniwheel',
        executable='arm_controller',
        name='arm_controller',
        output='screen'
        # parameters 제거
    )

    # 6. LiDAR 드라이버 Launch 파일 포함
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ldlidar, 'launch', 'ld14.launch.py')
        )
    )

    # LaunchDescription에 액션 추가
    return LaunchDescription([
        rsp_node,
        bridge_node,
        arm_node,  
        lidar_launch,
    ])
