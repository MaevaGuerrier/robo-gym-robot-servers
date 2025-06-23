#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



def get_dof_4_controllers(context):
    return [
        'joint_state_controller', 'waist_controller', 'shoulder_controller',
        'elbow_controller', 'wrist_angle_controller', 'left_finger_controller',
        'right_finger_controller'
    ]

def get_dof_5_controllers(context):
    return [
        'joint_state_controller', 'waist_controller', 'shoulder_controller',
        'elbow_controller', 'wrist_angle_controller', 'wrist_rotate_controller',
        'left_finger_controller', 'right_finger_controller'
    ]

def get_dof_6_controllers(context):
    return [
        'joint_state_controller', 'waist_controller', 'shoulder_controller',
        'elbow_controller', 'forearm_roll_controller', 'wrist_angle_controller',
        'wrist_rotate_controller', 'left_finger_controller', 'right_finger_controller'
    ]

def launch_setup(context, *args, **kwargs):
    use_trajectory_controllers = LaunchConfiguration('use_trajectory_controllers')
    use_position_controllers = LaunchConfiguration('use_position_controllers')

    js_broadcaster = Node(
                    package='controller_manager',
                    executable='spawner',
                    name='controller_spawner',
                    namespace=LaunchConfiguration('robot_name'),
                    arguments=['arm_controller', 'gripper_controller', 'joint_state_controller'],
                    condition=IfCondition(use_trajectory_controllers),
                    parameters=[
                        ParameterFile(PathJoinSubstitution([
                            FindPackageShare('interbotix_xsarm_gazebo'),
                            'config',
                            'trajectory_controllers',
                            [LaunchConfiguration('robot_model'), '_trajectory_controllers.yaml']
                        ]))
                    ],
                    output='screen'
                )
    
    dof4_and_position = PythonExpression([
        "'true' if '", use_position_controllers, "' == 'true' and '",
        LaunchConfiguration('dof'), "' == '4' else 'false'"
    ])
    dof5_and_position = PythonExpression([
        "'true' if '", LaunchConfiguration('use_position_controllers'), "' == 'true' and '",
        LaunchConfiguration('dof'), "' == '5' else 'false'"
    ])
    dof6_and_position = PythonExpression([
        "'true' if '", LaunchConfiguration('use_position_controllers'), "' == 'true' and '",
        LaunchConfiguration('dof'), "' == '6' else 'false'"
    ])
    js_broadcaster_4dof = Node(
                    package='controller_manager',
                    executable='spawner',
                    name='controller_spawner',
                    namespace=LaunchConfiguration('robot_name'),
                    arguments=get_dof_4_controllers,
                    condition=IfCondition(dof4_and_position),
                    parameters=[
                        ParameterFile(PathJoinSubstitution([
                            FindPackageShare('interbotix_xsarm_gazebo'),
                            'config',
                            'position_controllers',
                            [LaunchConfiguration('robot_model'), '_position_controllers.yaml']
                        ]))
                    ],
                    output='screen'
                ),

    js_broadcaster_5dof = Node(
                    package='controller_manager',
                    executable='spawner',
                    name='controller_spawner',
                    namespace=LaunchConfiguration('robot_name'),
                    arguments=get_dof_5_controllers,
                    condition=IfCondition(dof5_and_position),
                    parameters=[
                        ParameterFile(PathJoinSubstitution([
                            FindPackageShare('interbotix_xsarm_gazebo'),
                            'config',
                            'position_controllers',
                            [LaunchConfiguration('robot_model'), '_position_controllers.yaml']
                        ]))
                    ],
                    output='screen'
                ),

    js_broadcaster_6dof = Node(
                    package='controller_manager',
                    executable='spawner',
                    name='controller_spawner',
                    namespace=LaunchConfiguration('robot_name'),
                    arguments=get_dof_6_controllers,
                    condition=IfCondition(dof6_and_position),
                    parameters=[
                        ParameterFile(PathJoinSubstitution([
                            FindPackageShare('interbotix_xsarm_gazebo'),
                            'config',
                            'position_controllers',
                            [LaunchConfiguration('robot_model'), '_position_controllers.yaml']
                        ]))
                    ],
                    output='screen'
                )
    
    nodes_to_start = [
        js_broadcaster,
        js_broadcaster_4dof,
        js_broadcaster_5dof,
        js_broadcaster_6dof
    ]
    return nodes_to_start

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('robot_model', default_value='', description='Robot model name'),
        DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model'), description='Robot name'),
        DeclareLaunchArgument('show_ar_tag', default_value='false', description='Show AR tag'),
        DeclareLaunchArgument('show_gripper_bar', default_value='true', description='Show gripper bar'),
        DeclareLaunchArgument('show_gripper_fingers', default_value='true', description='Show gripper fingers'),
        DeclareLaunchArgument('use_world_frame', default_value='true', description='Use world frame'),
        DeclareLaunchArgument('external_urdf_loc', default_value='', description='External URDF location'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Use RViz'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo GUI'),
        DeclareLaunchArgument('gazebo_world', default_value=PathJoinSubstitution([
            FindPackageShare('interbotix_xsarm_gazebo'), 'worlds', 'xsarm_gazebo.world']),
                             description='Gazebo world file'),
        DeclareLaunchArgument('debug', default_value='false', description='Debug mode'),
        DeclareLaunchArgument('paused', default_value='true', description='Start paused'),
        DeclareLaunchArgument('recording', default_value='false', description='Record simulation'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('use_position_controllers', default_value='false', description='Use position controllers'),
        DeclareLaunchArgument('use_trajectory_controllers', default_value='true', description='Use trajectory controllers'),
        DeclareLaunchArgument('dof', default_value='5', description='Degrees of freedom'),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])