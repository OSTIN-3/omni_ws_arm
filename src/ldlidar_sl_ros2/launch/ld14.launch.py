#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
        package='ldlidar_sl_ros2',
        executable='ldlidar_sl_ros2_node',
        name='ldlidar_publisher_ld14',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD14'},
            {'laser_scan_topic_name': 'scan'},
            {'point_cloud_2d_topic_name': 'pointcloud2d'},
            # 🌟 수정 완료: URDF의 laser_frame과 일치시킴
            {'frame_id': 'laser_link'}, 
            {'port_name': '/dev/ttyACM0'},
            {'serial_baudrate' : 115200},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    
    # 1. 라이다 노드만 추가
    ld.add_action(ldlidar_node)

    return ld
