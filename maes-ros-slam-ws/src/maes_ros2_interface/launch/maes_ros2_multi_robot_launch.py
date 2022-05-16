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
import yaml

def get_maes_config_from_yaml(package_name):
    package_dir = get_package_share_directory(package_name)
    config_file_path = os.path.join(package_dir, "maes_config.yaml")
    with open(config_file_path, 'r') as file:
        maes_config = yaml.safe_load(file)

        return maes_config


def include_swarm_launch_descriptions(context):
    package_name = 'maes_ros2_interface'
    package_dir = get_package_share_directory(package_name)
    rviz_config_file = context.launch_configurations['rviz_config']
    use_rviz = context.launch_configurations['use_rviz']
    default_bt_xml_filename = context.launch_configurations["default_bt_xml_filename"]

    maes_config = get_maes_config_from_yaml(package_name)

    number_of_robots = maes_config['number_of_robots']

    swarm_descriptions = []
    for n in range(number_of_robots):
        robot_name = "robot{0}".format(n)

        robot_n_description = GroupAction([
            LaunchDescription([
                Node(
                    package='maes_robot_controller',
                    executable='maes_controller',
                    name='maes_robot_controller',
                    output='screen',
                    namespace=robot_name,
                    emulate_tty=True,
                    parameters=[
                        {'robot_id': robot_name}
                    ]
                )
            ]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_dir, 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
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
                    "use_sim_time": 'True',
                    "params_file":  os.path.join(package_dir, 'maes_nav2_multirobot_params_robotn.yaml'),
                    "autostart": 'True',
                    "raytrace_range": str(maes_config['robot_constraints']['slam_raytrace_range']),
                    "robot_radius": str(float(maes_config['robot_constraints']['agent_relative_size']) / 2),
                    "global_costmap_width": str(maes_config['map']['width_in_tiles']),
                    "global_costmap_height": str(maes_config['map']['height_in_tiles']),
                    "global_costmap_origin_x": str(-float(maes_config['map']['width_in_tiles'] / 2)),
                    "global_costmap_origin_y": str(-float(maes_config['map']['height_in_tiles'] / 2))
                }.items()),
        ])
        swarm_descriptions += [robot_n_description]

    return swarm_descriptions

def generate_launch_description():
    package_name = 'maes_ros2_interface'
    package_dir = get_package_share_directory(package_name)

    rviz_config_file = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")


    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'maes_nav2.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"), "behavior_trees", "navigate_w_replanning_time.xml"
        ),
        description="Full path to the behavior tree xml file to use",
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
    launch_description.add_action(declare_bt_xml_cmd)

    launch_description.add_action(OpaqueFunction(function=include_swarm_launch_descriptions))

    return launch_description
