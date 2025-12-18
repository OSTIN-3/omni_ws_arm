import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 찾기
    pkg_omniwheel = get_package_share_directory('omniwheel')
    pkg_my_nav = get_package_share_directory('agv_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 2. [하드웨어] Omniwheel Bringup (모터 + 라이다 + 로봇팔 제어)
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_omniwheel, 'launch', 'omni_bringup.launch.py')
        )
    )

    # 3. [지능] Navigation2 실행
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_nav, 'launch', 'navigation.launch.py')
        )
    )

    # 4. [시각화] Rviz2 (Nav2용 설정 파일로 열기)
    rviz_config_file = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        driver_launch,
        nav_launch,
        rviz_node
    ])
