from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')
    arm_model = LaunchConfiguration('arm_model')
    show_gripper_bar = LaunchConfiguration('show_gripper_bar')
    show_gripper_fingers = LaunchConfiguration('show_gripper_fingers')
    use_world_frame = LaunchConfiguration('use_world_frame')
    external_urdf_loc = LaunchConfiguration('external_urdf_loc')
    use_rviz = LaunchConfiguration('use_rviz')
    gui = LaunchConfiguration('gui')
    gazebo_world = LaunchConfiguration('gazebo_world')
    debug = LaunchConfiguration('debug')
    paused = LaunchConfiguration('paused')
    recording = LaunchConfiguration('recording')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_position_controllers = LaunchConfiguration('use_position_controllers')
    use_trajectory_controllers = LaunchConfiguration('use_trajectory_controllers')
    dof = LaunchConfiguration('dof')

    interbotix_gazebo_dir = get_package_share_directory('interbotix_xslocobot_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value=''),
        DeclareLaunchArgument('robot_name', default_value=robot_model),
        DeclareLaunchArgument('arm_model', default_value=''),
        DeclareLaunchArgument('show_gripper_bar', default_value='true'),
        DeclareLaunchArgument('show_gripper_fingers', default_value='true'),
        DeclareLaunchArgument('use_world_frame', default_value='true'),
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('gazebo_world', default_value=os.path.join(interbotix_gazebo_dir, 'worlds/xslocobot_gazebo.world')),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('recording', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_position_controllers', default_value='false'),
        DeclareLaunchArgument('use_trajectory_controllers', default_value='true'),
        DeclareLaunchArgument('dof', default_value='5'),

        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', interbotix_gazebo_dir),

        # Load controllers config
        Node(
            package='ros2param',
            executable='ros2param',
            name='load_gazebo_controllers',
            arguments=['load_from_yaml', robot_name, os.path.join(interbotix_gazebo_dir, 'config', 'locobot_gazebo_controllers.yaml')],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': gazebo_world,
                'gui': gui,
                'debug': debug,
                'paused': paused,
                'use_sim_time': use_sim_time
            }.items(),
        ),

        # Controller spawning logic
        GroupAction([
            GroupAction([
                Node(
                    package='ros2param',
                    executable='ros2param',
                    name='load_arm_trajectory_controllers',
                    arguments=['load_from_yaml', robot_name, os.path.join(interbotix_gazebo_dir, 'config/trajectory_controllers', f'{LaunchConfiguration("arm_model")}_trajectory_controllers.yaml')],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner.py',
                    arguments=['arm_controller', 'gripper_controller', 'pan_controller', 'tilt_controller', 'joint_state_controller'],
                    namespace=robot_name,
                    output='screen'
                )
            ], condition=IfCondition(use_trajectory_controllers)),

            GroupAction([
                Node(
                    package='ros2param',
                    executable='ros2param',
                    name='load_arm_position_controllers',
                    arguments=['load_from_yaml', robot_name, os.path.join(interbotix_gazebo_dir, 'config/position_controllers', f'{LaunchConfiguration("arm_model")}_position_controllers.yaml')],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner.py',
                    arguments=[
                        'joint_state_controller',
                        'waist_controller',
                        'shoulder_controller',
                        'elbow_controller',
                        'wrist_angle_controller',
                        'wrist_rotate_controller',
                        'left_finger_controller',
                        'right_finger_controller',
                        'pan_controller',
                        'tilt_controller'
                    ],
                    namespace=robot_name,
                    output='screen'
                )
            ], condition=IfCondition(use_position_controllers))
        ], condition=IfCondition("robot_model != 'locobot_base'")),

        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['pan_controller', 'tilt_controller', 'joint_state_controller'],
            namespace=robot_name,
            output='screen',
            condition=UnlessCondition("robot_model != 'locobot_base'")
        )
    ])
