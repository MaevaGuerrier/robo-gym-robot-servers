#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition



def launch_setup(context, *args, **kwargs):

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('interbotix_arm_robot_server'),
            'urdf',
            [LaunchConfiguration('robot_model'), '.urdf.xacro']
        ]),
        ' robot_name:=', LaunchConfiguration('robot_name'),
        ' base_link_frame:=', LaunchConfiguration('base_link_frame'),
        ' show_ar_tag:=', LaunchConfiguration('show_ar_tag'),
        ' show_gripper_bar:=', LaunchConfiguration('show_gripper_bar'),
        ' show_gripper_fingers:=', LaunchConfiguration('show_gripper_fingers'),
        ' use_world_frame:=', LaunchConfiguration('use_world_frame'),
        ' external_urdf_loc:=', LaunchConfiguration('external_urdf_loc'),
        ' load_gazebo_configs:=', LaunchConfiguration('load_gazebo_configs'),
        ' x:=', LaunchConfiguration('x'),
        ' y:=', LaunchConfiguration('y'),
        ' z:=', LaunchConfiguration('z'),
        ' roll:=', LaunchConfiguration('roll'),
        ' pitch:=', LaunchConfiguration('pitch'),
        ' yaw:=', LaunchConfiguration('yaw'),
        ' camera1_gazebo:=', LaunchConfiguration('camera1_gazebo'),
        ' camera1_link_x:=', LaunchConfiguration('camera1_link_x'),
        ' camera1_link_y:=', LaunchConfiguration('camera1_link_y'),
        ' camera1_link_z:=', LaunchConfiguration('camera1_link_z'),
        ' camera1_link_roll:=', LaunchConfiguration('camera1_link_roll'),
        ' camera1_link_pitch:=', LaunchConfiguration('camera1_link_pitch'),
        ' camera1_link_yaw:=', LaunchConfiguration('camera1_link_yaw')
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
        ],
    )
    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", LaunchConfiguration('gazebo_world')]}.items(),
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
    )

    gz_launch_description_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -s -r -v 4 ", LaunchConfiguration('gazebo_world')]}.items(),
        condition=UnlessCondition(LaunchConfiguration('gazebo_gui')),
    )

    # Make the /clock topic available in ROS
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )
    
    nodes_to_start = [
        robot_state_publisher_node,
        gz_spawn_entity,
        gz_launch_description_with_gui,
        gz_launch_description_without_gui,
        gz_sim_bridge
    ]
    
    return nodes_to_start

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('robot_model', default_value='', description='Robot model name'),
        DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model'), description='Robot name'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link', description='Base link frame name'),
        DeclareLaunchArgument('show_ar_tag', default_value='false', description='Show AR tag'),
        DeclareLaunchArgument('show_gripper_bar', default_value='true', description='Show gripper bar'),
        DeclareLaunchArgument('show_gripper_fingers', default_value='true', description='Show gripper fingers'),
        DeclareLaunchArgument('use_world_frame', default_value='true', description='Use world frame'),
        DeclareLaunchArgument('external_urdf_loc', default_value='', description='External URDF location'),
        DeclareLaunchArgument('load_gazebo_configs', default_value='false', description='Load Gazebo configurations'),

        DeclareLaunchArgument('x', default_value='0.0', description='base_link x coordinate with respect to the world frame'),
        DeclareLaunchArgument('y', default_value='0.0', description='base_link y coordinate with respect to the world frame'),
        DeclareLaunchArgument('z', default_value='0.1', description='base_link z coordinate with respect to the world frame'),
        DeclareLaunchArgument('roll', default_value='0.0', description='base_link roll with respect to the world frame'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='base_link pitch with respect to the world frame'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='base_link yaw with respect to the world frame'),

        DeclareLaunchArgument('camera1_gazebo', default_value='False', description='use camera1 gazebo simulated sensor'),
        DeclareLaunchArgument('camera1_link_x', default_value='0.0', description='camera1_link x coordinate with respect to the world frame'),
        DeclareLaunchArgument('camera1_link_y', default_value='0.0', description='camera1_link y coordinate with respect to the world frame'),
        DeclareLaunchArgument('camera1_link_z', default_value='0.1', description='camera1_link z coordinate with respect to the world frame'),
        DeclareLaunchArgument('camera1_link_roll', default_value='0.0', description='camera1_link roll with respect to the world frame'),
        DeclareLaunchArgument('camera1_link_pitch', default_value='0.0', description='camera1_link pitch with respect to the world frame'),
        DeclareLaunchArgument('camera1_link_yaw', default_value='0.0', description='camera1_link yaw with respect to the world frame'),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])