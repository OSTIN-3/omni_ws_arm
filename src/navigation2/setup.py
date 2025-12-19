from setuptools import setup
import os
from glob import glob

package_name = 'navigation2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Launch 파일 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Config 파일 설치
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # 3. Map 파일 설치
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='My Navigation Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)