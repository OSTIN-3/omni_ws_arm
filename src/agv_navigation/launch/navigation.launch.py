import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 내 패키지 경로 찾기 (agv_navigation)
    pkg_agv_navigation = get_package_share_directory('agv_navigation')
    
    # 2. 파일 경로 설정 (사용자 구조에 맞춤)
    # 맵 파일: maps/map_q.yaml
    default_map_path = os.path.join(pkg_agv_navigation, 'maps', 'map_q.yaml')
    
    # 파라미터 파일: config/nav2_params.yaml
    default_params_path = os.path.join(pkg_agv_navigation, 'config', 'nav2_params.yaml')

    # Nav2 패키지 경로
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 3. Launch Configuration 변수 설정
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # 4. Nav2 실행 (Include)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': 'true'  # 로드되면 바로 시작
        }.items()
    )

    return LaunchDescription([
        # 인자 선언 (터미널에서 변경 가능하도록)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',  # 실물 로봇이므로 False
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_path,
            description='Full path to nav2 parameters file'
        ),
        
        nav2_bringup_launch
    ])