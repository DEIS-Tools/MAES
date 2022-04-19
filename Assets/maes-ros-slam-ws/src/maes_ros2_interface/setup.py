import glob
import os

from setuptools import setup

package_name = 'maes_ros2_interface'

setup(
    name=package_name,
    version='0.0.2',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), [
            'rviz/maes_nav2.rviz',
            'launch/maes_ros2_multi_robot_launch.py',
            'launch/rviz_launch.py',
            'launch/maes_bringup_launch.py',
            'launch/maes_slam_launch.py',
            'params/maes_nav2_multirobot_params_robotn.yaml',
            'maes_parameters.yaml'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maes',
    maintainer_email='pholle17@student.aau.dk',
    description='Maes Ros2 interface',
    license='Apache 2.0',
    tests_require=['pytest']
)
