#!/usr/bin/env python3
"""
간단한 통합 Navigation2 런치 파일
Nav2의 bringup_launch.py를 사용하여 map_server + AMCL + navigation을 모두 실행
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 파라미터
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    map_yaml_file = LaunchConfiguration('map', 
                                        default='/home/jdamr/omni_ws_edit/src/navigation2/maps/map_q.yaml')
    params_file = LaunchConfiguration('params_file',
                                      default='/home/jdamr/omni_ws_edit/src/navigation2/config/nav2_params.yaml')

    # Nav2 bringup 디렉토리
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Nav2 전체 실행 (localization + navigation 모두 포함)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'automni_ws_editart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='/home/jdamr/omni_ws_edit/src/navigation2/maps/map_q.yaml',
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/jdamr/omni_ws_edit/src/navigation2/config/nav2_params.yaml',
            description='Full path to nav2 parameters file'
        ),
        
        nav2_bringup_launch
    ])