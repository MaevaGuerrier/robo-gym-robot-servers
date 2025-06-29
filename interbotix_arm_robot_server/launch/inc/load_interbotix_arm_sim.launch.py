#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name")
    robot_model = LaunchConfiguration("robot_model")
    base_link_frame = LaunchConfiguration("base_link_frame")
    show_ar_tag = LaunchConfiguration("show_ar_tag")
    show_gripper_bar = LaunchConfiguration("show_gripper_bar")
    show_gripper_fingers = LaunchConfiguration("show_gripper_fingers")
    use_world_frame = LaunchConfiguration("use_world_frame")
    external_urdf_loc = LaunchConfiguration("external_urdf_loc")
    load_gazebo_configs = LaunchConfiguration("load_gazebo_configs")
    gazebo_world = LaunchConfiguration("gazebo_world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    camera_gazebo = LaunchConfiguration("camera_gazebo")
    camera_link_x = LaunchConfiguration("camera_link_x")
    camera_link_y = LaunchConfiguration("camera_link_y")
    camera_link_z = LaunchConfiguration("camera_link_z")
    camera_link_roll = LaunchConfiguration("camera_link_roll")
    camera_link_pitch = LaunchConfiguration("camera_link_pitch")
    camera_link_yaw = LaunchConfiguration("camera_link_yaw")
    robot_description_filename = LaunchConfiguration("robot_description_filename")
    controllers_file = LaunchConfiguration("controllers_file")
    hardware_type = LaunchConfiguration("hardware_type")

    mesh_path = FindPackageShare('interbotix_xsarm_descriptions').perform(context)

    share_path = os.path.dirname(mesh_path)
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare('interbotix_arm_robot_server'), "config", controllers_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare('interbotix_arm_robot_server'), 'urdf', robot_description_filename]
        ),
        ' robot_name:=', robot_name,
        ' base_link_frame:=', base_link_frame,
        ' show_ar_tag:=', show_ar_tag,
        ' show_gripper_bar:=', show_gripper_bar,
        ' show_gripper_fingers:=', show_gripper_fingers,
        ' use_world_frame:=', use_world_frame,
        ' external_urdf_loc:=', external_urdf_loc,
        ' load_gazebo_configs:=', load_gazebo_configs,
        ' x:=', x,
        ' y:=', y,
        ' z:=', z,
        ' roll:=', roll,
        ' pitch:=', pitch,
        ' yaw:=', yaw,
        ' camera1_gazebo:=', camera_gazebo,
        ' camera1_link_x:=', camera_link_x,
        ' camera1_link_y:=', camera_link_y,
        ' camera1_link_z:=', camera_link_z,
        ' camera1_link_roll:=', camera_link_roll,
        ' camera1_link_pitch:=', camera_link_pitch,
        ' camera1_link_yaw:=', camera_link_yaw,
        ' simulation_controllers:=', initial_joint_controllers,
        ' hardware_type:=', hardware_type,
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "gripper_controller", "-c", "controller_manager"],
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "interbotix_arm",
            "-allow_renaming",
            "true",
        ],
    )
    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", gazebo_world]}.items(),
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
    )

    gz_launch_description_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -s -r -v 4 ", gazebo_world]}.items(),
        condition=UnlessCondition(LaunchConfiguration('gazebo_gui')),
    )

    world_name_split = PythonExpression([
        "'", LaunchConfiguration('gazebo_world'), "'.replace('.world', '')"
    ])

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            f"/world/{world_name_split.perform(context)}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        output="screen",
    )

    nodes_to_start = [
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=share_path),
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        robot_state_publisher_node,
        gz_spawn_entity,
        gz_launch_description_with_gui,
        gz_launch_description_without_gui,
        gz_sim_bridge,
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
        DeclareLaunchArgument('rviz_gui', default_value='false', description='Use RViz'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo GUI'),
        DeclareLaunchArgument('gazebo_world', default_value='empty.world'),
        DeclareLaunchArgument('debug', default_value='false', description='Debug mode'),
        DeclareLaunchArgument('paused', default_value='true', description='Start paused'),
        DeclareLaunchArgument('recording', default_value='false', description='Record simulation'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('dof', default_value='5', description='Degrees of freedom'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link', description='Base link frame name'),
        DeclareLaunchArgument('load_gazebo_configs', default_value='false', description='Load Gazebo configurations'),
        DeclareLaunchArgument('x', default_value='0.0', description='base_link x coordinate with respect to the world frame'),
        DeclareLaunchArgument('y', default_value='0.0', description='base_link y coordinate with respect to the world frame'),
        DeclareLaunchArgument('z', default_value='0.1', description='base_link z coordinate with respect to the world frame'),
        DeclareLaunchArgument('roll', default_value='0.0', description='base_link roll with respect to the world frame'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='base_link pitch with respect to the world frame'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='base_link yaw with respect to the world frame'),
        DeclareLaunchArgument('robot_description_file', default_value=''),
        DeclareLaunchArgument('camera_gazebo', default_value='False', description='use camera gazebo simulated sensor'),
        DeclareLaunchArgument('camera_link_x', default_value='0.0', description='camera_link x coordinate with respect to the world frame'),
        DeclareLaunchArgument('camera_link_y', default_value='0.0', description='camera_link y coordinate with respect to the world frame'),
        DeclareLaunchArgument('camera_link_z', default_value='0.1', description='camera_link z coordinate with respect to the world frame'),
        DeclareLaunchArgument('camera_link_roll', default_value='0.0', description='camera_link roll with respect to the world frame'),
        DeclareLaunchArgument('camera_link_pitch', default_value='0.0', description='camera_link pitch with respect to the world frame'),
        DeclareLaunchArgument('camera_link_yaw', default_value='0.0', description='camera_link yaw with respect to the world frame'),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="",
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "hardware_type",
            default_value="gz_ign",
            description="Hardware type to use",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])