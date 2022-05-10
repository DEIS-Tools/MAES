import glob
import os

from setuptools import setup

package_name = 'maes_ros2_interface'

setup(
    name=package_name,
    version='1.0.0',
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
            'maes_config.yaml',
            'behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml',
            'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml',
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michele Albano',
    maintainer_email='michele.albano@gmail.com',
    description='Main entry point for MAES robot bring-up of multiple robots.',
    license='Apache 2.0',
    tests_require=['pytest']
)
