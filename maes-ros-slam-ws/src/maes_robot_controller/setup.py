import os

from setuptools import setup

package_name = 'maes_robot_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michele Albano',
    maintainer_email='michele.albano@gmail.com',
    description='The controller with a logic loop for controlling the behavior of the MAES robots',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maes_controller = maes_robot_controller.maes_robot_controller:main',
        ],
    },
)
