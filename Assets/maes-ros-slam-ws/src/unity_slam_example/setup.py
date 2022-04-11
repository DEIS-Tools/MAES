import glob
import os

from setuptools import setup

package_name = 'unity_slam_example'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), [
                                                'rviz/nav2_namespaced_view.rviz',
                                                'rviz/nav2_unity.rviz',
                                                'launch/unity_slam_example.py',
                                                'launch/rviz_launch.py',
                                                'launch/maes_bringup_launch.py',
                                                'launch/maes_slam_launch.py',
                                                'params/maes_nav2_multirobot_params_robot0.yaml',
                                                'params/maes_nav2_multirobot_params_robot1.yaml',
                                                'params/maes_nav2_multirobot_params_robot2.yaml',
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
