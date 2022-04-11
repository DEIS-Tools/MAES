import os
import sys
import string

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, TextSubstitution
from nav2_common import launch
from rclpy.parameter import Parameter


def include_swarm_launch_descriptions(context):
    package_name = 'unity_slam_example'
    package_dir = get_package_share_directory(package_name)
    rviz_config_file = context.launch_configurations['rviz_config']
    map_yaml_file = context.launch_configurations['map'] # Not used???
    default_bt_xml_filename = context.launch_configurations["default_bt_xml_filename"]

    number_of_robots = int(context.launch_configurations['number_of_robots'])

    swarm_descriptions = []
    for n in range(number_of_robots):
        robot_name = "robot{0}".format(n)

        robot_n_description = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_dir, 'rviz_launch.py')),
                launch_arguments={
                    'namespace': TextSubstitution(text=robot_name),
                    'use_namespace': 'True',
                    'rviz_config': rviz_config_file}.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package_dir, "maes_bringup_launch.py")),
                launch_arguments={
                    "namespace": TextSubstitution(text=robot_name),
                    "use_namespace": 'True',
                    "slam": 'True',
                    "map": map_yaml_file,
                    "use_sim_time": 'True',
                    "params_file":  os.path.join(package_dir, 'maes_nav2_multirobot_params_robot{0}.yaml'.format(n)),
                    "default_bt_xml_filename": default_bt_xml_filename,
                    "autostart": 'True',
                }.items())
        ])
        swarm_descriptions += [robot_n_description]

    return swarm_descriptions

def generate_launch_description():
    package_name = 'unity_slam_example'
    package_dir = get_package_share_directory(package_name)
    params_dir = os.path.join(package_dir, 'params')
    launch_dir = os.path.join(package_dir, 'launch')
    bringup_dir = get_package_share_directory('nav2_bringup')


    rviz_config_file = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml_file = LaunchConfiguration('map')
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")



    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'nav2_unity.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map yaml file to load')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"), "behavior_trees", "navigate_w_replanning_and_recovery.xml"
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_number_of_robots_cmd = DeclareLaunchArgument(
        "number_of_robots",
        default_value='3',
        description="Number of robot instances to spawn. Including Nav2 stack and rviz",
    )

    ros_tcp_endpoint_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_tcp_endpoint'), 'launch', 'endpoint.py')
        ),
    )


    launch_description = LaunchDescription()
    launch_description.add_action(ros_tcp_endpoint_launch)
    launch_description.add_action(declare_rviz_config_file_cmd)
    launch_description.add_action(declare_use_rviz_cmd)
    launch_description.add_action(declare_map_cmd)
    launch_description.add_action(declare_bt_xml_cmd)
    launch_description.add_action(declare_number_of_robots_cmd)

    launch_description.add_action(OpaqueFunction(function=include_swarm_launch_descriptions))

    return launch_description
