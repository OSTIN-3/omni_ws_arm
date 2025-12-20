from setuptools import setup
import os
from glob import glob

package_name = 'omniwheel'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
        # ⭐ [추가됨] Config 파일 (ekf.yaml) 복사
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omniwheel User',
    maintainer_email='user@example.com',
    description='Omni-Wheel Robot ROS2 Control Package',
    license='MIT',
    tests_require=['pytest', 'pyserial'],
    entry_points={
        'console_scripts': [
            # 1. 메인 구동 노드
            'omnirun = omniwheel.omnirun:main',
            
            # 2. 키보드 텔레옵 노드
            'teleop = omniwheel.teleop:main',
            
            # ⭐ [추가됨] 로봇팔 컨트롤러 등록
            'arm_controller = omniwheel.arm_controller:main',
        ],
    },
)
