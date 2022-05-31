# Copyright 2022 MAES
# 
# This file is part of MAES
# 
# MAES is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your option)
# any later version.
# 
# MAES is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with MAES. If not, see http://www.gnu.org/licenses/.
# 
# Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
# 
# Original repository: https://github.com/MalteZA/MAES
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
            'maes_config_ros_system_test.yaml',
            'behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml',
            'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml',
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michele Albano',
    maintainer_email='michele.albano@gmail.com',
    description='Main entry point for MAES robot bring-up of multiple robots.',
    license='GPL 3.0',
    tests_require=['pytest']
)
