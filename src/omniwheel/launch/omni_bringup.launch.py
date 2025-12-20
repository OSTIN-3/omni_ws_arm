import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_omniwheel = get_package_share_directory('omniwheel')
    pkg_ldlidar = get_package_share_directory('ldlidar_sl_ros2')

    # 1. URDF 설정
    urdf_file_name = 'ostin_3.urdf'
    urdf_path = os.path.join(pkg_omniwheel, 'urdf', urdf_file_name)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 2. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. OmniRun (바퀴/IMU 하드웨어 제어)
    omnirun_node = Node(
        package='omniwheel',
        executable='omnirun',
        name='omnirun_node',
        output='screen'
    )

    # 4. ⭐ [추가됨] Arm Controller (로봇팔 하드웨어 제어)
    arm_node = Node(
        package='omniwheel',
        executable='arm_controller',
        name='arm_controller_node',
        output='screen'
    )

    # 5. LiDAR 실행
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ldlidar, 'launch', 'ld14.launch.py')
        )
    )

    # 6. EKF 노드 실행 (Robot Localization)
    ekf_config_path = os.path.join(pkg_omniwheel, 'config', 'ekf.yaml')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
    )

    return LaunchDescription([
        rsp_node,
        omnirun_node,
        arm_node,      # 로봇팔 노드 추가
        lidar_launch,
        ekf_node,
    ])
