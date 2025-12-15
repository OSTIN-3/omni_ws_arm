from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'jdamr200_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'urdf'),glob(os.path.join('urdf','*.*'))),
        (os.path.join('share',package_name,'meshes'),glob(os.path.join('meshes','*.*'))),
        (os.path.join('share',package_name,'rviz'),glob(os.path.join('rviz','*.rviz*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slam',
    maintainer_email='jdedu.kr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
