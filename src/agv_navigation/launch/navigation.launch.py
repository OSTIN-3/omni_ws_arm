import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_my_nav = get_package_share_directory('agv_navigation')

    # 파일 경로 설정
    nav2_params_file = os.path.join(pkg_my_nav, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_my_nav, 'maps', 'map_q.yaml')

    # Nav2 Bringup 실행 (Map Server + AMCL + Planner + Controller 포함)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'false', # 실제 로봇이므로 false
            'autostart': 'true'      # 실행 시 자동 시작
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
