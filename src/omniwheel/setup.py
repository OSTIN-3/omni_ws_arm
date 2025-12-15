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
        
        # Launch 파일을 설치 폴더로 복사합니다.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
        
        # URDF 파일을 설치 폴더로 복사합니다.
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),

        # ⭐⭐ 추가된 부분: 맵 파일 (map_q.pgm, map_q.yaml)을 설치 폴더로 복사합니다. ⭐⭐
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
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
            'omnirun = omniwheel.omnirun:main',
            'teleop = omniwheel.teleop:main',
            'omnibridge = omniwheel.omnibridge:main',
        ],
    },
)
