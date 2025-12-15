import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 맵 파일이 설치된 경로를 찾습니다.
    map_data_pkg_dir = get_package_share_directory('omniwheel')
    map_file_path = os.path.join(map_data_pkg_dir, 'maps', 'map_q.yaml')

    # 2. nav2_map_server 노드 설정
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}],
    )

    # 3. map_server 노드의 lifecycle_manager 설정
    # 맵 서버 노드를 활성화 상태로 전환해주는 필수 노드입니다.
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['map_server']}]
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager_node
    ])
