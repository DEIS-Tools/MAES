import os

from setuptools import setup

package_name = 'maes_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), [
            'behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml',
            'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='philipholler',
    maintainer_email='philipholler94@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maes_controller = maes_robot_controller.maes_robot_controller:main',
        ],
    },
)
