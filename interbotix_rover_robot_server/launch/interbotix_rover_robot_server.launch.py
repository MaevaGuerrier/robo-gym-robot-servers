from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('robot_model', default_value='locobot_wx250s'),
        DeclareLaunchArgument('robot_name', default_value='locobot'),
        DeclareLaunchArgument('dof', default_value='6'),
        DeclareLaunchArgument('real_robot', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz_gui', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('world_name', default_value=os.path.join(get_package_share_directory('interbotix_xslocobot_gazebo'), 'worlds/xslocobot_gazebo.world')),
        DeclareLaunchArgument('server_port', default_value='50051'),
        DeclareLaunchArgument('action_cycle_rate', default_value='25'),
        DeclareLaunchArgument('reference_frame', default_value=[LaunchConfiguration('robot_model'), '/base_link']),
        DeclareLaunchArgument('arm_model', default_value=["mobile_", LaunchConfiguration('robot_model')[LaunchConfiguration('robot_model').find('_')+1:]]),
        DeclareLaunchArgument('show_lidar', default_value='false'),
        DeclareLaunchArgument('show_gripper_bar', default_value='true'),
        DeclareLaunchArgument('show_gripper_fingers', default_value='true'),
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        DeclareLaunchArgument('rviz_frame', default_value=[LaunchConfiguration('robot_name'), '/base_footprint']),
        DeclareLaunchArgument('use_position_controllers', default_value='false'),
        DeclareLaunchArgument('use_trajectory_controllers', default_value='true'),
        DeclareLaunchArgument('base_type', default_value='create3'),
        DeclareLaunchArgument('rviz_config_path', default_value=os.path.join(get_package_share_directory('interbotix_rover_robot_server'), 'rviz')),
        DeclareLaunchArgument('rviz_config_file', default_value='interbotix_rl.rviz'),
    ]

    rviz_config = PathJoinSubstitution([
        LaunchConfiguration('rviz_config_path'),
        LaunchConfiguration('rviz_config_file')
    ])

    robot_description_content = Command([
        'xacro ', os.path.join(
            get_package_share_directory('interbotix_rover_robot_server'),
            'urdf', 'locobot.urdf.xacro'
        ),
        ' robot_name:=', LaunchConfiguration('robot_name'),
        ' robot_model:=', LaunchConfiguration('robot_model'),
        ' arm_model:=', LaunchConfiguration('arm_model'),
        ' base_model:=', LaunchConfiguration('base_type'),
        ' rviz_frame_frame:=', LaunchConfiguration('rviz_frame'),
        ' show_lidar:=', LaunchConfiguration('show_lidar'),
        ' show_gripper_bar:=', LaunchConfiguration('show_gripper_bar'),
        ' show_gripper_fingers:=', LaunchConfiguration('show_gripper_fingers'),
        ' external_urdf_loc:=', LaunchConfiguration('external_urdf_loc'),
        ' camera1_gazebo:=False camera1_link_x:=0 camera1_link_y:=0 camera1_link_z:=0 ',
        'camera1_link_roll:=0 camera1_link_pitch:=0 camera1_link_yaw:=0'
    ])

    robot_description = {'robot_description': robot_description_content}

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('robot_name'),
            name='robot_state_publisher',
            parameters=[robot_description, {'publish_frequency': 125.0}]
        ),

        Node(
            package='interbotix_rover_robot_server',
            executable='joint_trajectory_command_handler.py',
            name='joint_trajectory_command_handler',
            parameters=[
                {'real_robot': LaunchConfiguration('real_robot')},
                {'action_cycle_rate': LaunchConfiguration('action_cycle_rate')}
            ],
            output='screen'
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('rviz_gui')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        Node(
            package='interbotix_rover_robot_server',
            executable='robot_server.py',
            name='robot_server',
            parameters=[
                {'server_port': LaunchConfiguration('server_port')},
                {'real_robot': LaunchConfiguration('real_robot')},
                {'robot_model': LaunchConfiguration('robot_model')},
                {'action_cycle_rate': LaunchConfiguration('action_cycle_rate')},
                {'reference_frame': LaunchConfiguration('reference_frame')}
            ],
            output='screen',
            remappings=[
                ('/locobot/odom', '/mobile_base/odom'),
                ('/locobot/cmd_vel', '/mobile_base/cmd_vel')
            ]
        )
    ]

    return LaunchDescription(declared_args + nodes)
