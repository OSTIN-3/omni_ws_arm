import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 1. 패키지 경로 설정
    pkg_omniwheel = get_package_share_directory('omniwheel')
    pkg_ldlidar = get_package_share_directory('ldlidar_sl_ros2') 
    
    # 🌟🌟 경로 수정 반영: URDF 파일이 있는 패키지 경로를 가져옵니다. 🌟🌟
    pkg_description = get_package_share_directory('jdamr200_description')

    # 2. URDF 파일 경로 설정
    urdf_file_name = 'jdamr200.urdf'
    urdf_path = os.path.join(
        pkg_description, # 🌟 'jdamr200_description' 패키지 경로 사용
        'urdf',
        urdf_file_name
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
    bridge_node = Node(
        package='omniwheel',
        executable='omnirun',
        name='omnibridge_node',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyUSB1'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'imu_frame': 'imu_link'}, 
            {'publish_tf': False} 
        ]
    )

    # 5. LiDAR 드라이버 Launch 파일 포함
    # LiDAR Launch 파일이 ldlidar_sl_ros2 패키지에 있는 것을 확인했습니다.
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ldlidar, 'launch', 'ld14.launch.py')
        )
    )

    # LaunchDescription에 액션 추가
    return LaunchDescription([
        rsp_node,
        bridge_node,
        lidar_launch,
    ])
