from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('controller_config_file', description='Config file used for defining the ROS-Control controllers'),
        DeclareLaunchArgument('controllers', default_value='joint_state_controller eff_joint_traj_controller', description='Controllers to start'),
        DeclareLaunchArgument('stopped_controllers', default_value='joint_group_eff_controller', description='Controllers to load but not start'),
        DeclareLaunchArgument("activate_joint_controller", default_value="true", description="Enable headless mode for robot control"),
        DeclareLaunchArgument("initial_joint_controller", default_value="scaled_joint_trajectory_controller", description="Robot controller to start."),

        DeclareLaunchArgument('gazebo_model_name', default_value='robot', description='The name to give to the model in Gazebo (after spawning it)'),
        DeclareLaunchArgument('gazebo_world', default_value='empty.world', description='The .world file to load in Gazebo'),
        DeclareLaunchArgument('gui', default_value='true', description='If true, Gazebo UI is started. If false, only start Gazebo server'),
        DeclareLaunchArgument('paused', default_value='false', description='If true, start Gazebo in paused mode'),
        DeclareLaunchArgument('robot_description_param_name', default_value='robot_description', description='Name of the parameter which contains the robot description'),
        DeclareLaunchArgument('spawn_z', default_value='0.0', description='At which height the model should be spawned'),
        DeclareLaunchArgument('start_gazebo', default_value='true', description='If true, Gazebo will be started'),
        
        DeclareLaunchArgument('spawn_shoulder_pan_joint', default_value='0.0', description='Initial position for shoulder_pan_joint'),
        DeclareLaunchArgument('spawn_shoulder_lift_joint', default_value='0.0', description='Initial position for shoulder_lift_joint'),
        DeclareLaunchArgument('spawn_elbow_joint', default_value='0.0', description='Initial position for elbow_joint'),
        DeclareLaunchArgument('spawn_wrist_1_joint', default_value='0.0', description='Initial position for wrist_1_joint'),
        DeclareLaunchArgument('spawn_wrist_2_joint', default_value='0.0', description='Initial position for wrist_2_joint'),
        DeclareLaunchArgument('spawn_wrist_3_joint', default_value='0.0', description='Initial position for wrist_3_joint'),
    ]
    
    paused_tag = PythonExpression([
        "'' if '", LaunchConfiguration('paused'), "' == 'true' else '-unpause'"
    ])
    
    def launch_setup(context, *args, **kwargs):
        def split_controllers(controller_arg):
            controllers_str = LaunchConfiguration(controller_arg).perform(context)
            return controllers_str.split()
        
        # Get the controller arguments
        active_controllers = split_controllers('controllers')
        stopped_controllers = ['--stopped'] + split_controllers('stopped_controllers')
        ros_gz_sim = get_package_share_directory('ros_gz_sim')

        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={
                    'gz_args': ['-r -v4 ', 'empty.sdf'], 'on_exit_shutdown': 'true'}.items(),
                condition=IfCondition(LaunchConfiguration('start_gazebo'))
            )
        ]

        nodes = [
            Node(
                package='ros_gz_sim',
                executable='create',
                condition=IfCondition(LaunchConfiguration('start_gazebo')),
                arguments=[
                    '-topic', LaunchConfiguration('robot_description_param_name'),
                    '-z', '0.2',
                    '-J', 'shoulder_pan_joint', LaunchConfiguration('spawn_shoulder_pan_joint'),
                    '-J', 'shoulder_lift_joint', LaunchConfiguration('spawn_shoulder_lift_joint'),
                    '-J', 'elbow_joint', LaunchConfiguration('spawn_elbow_joint'),
                    '-J', 'wrist_1_joint', LaunchConfiguration('spawn_wrist_1_joint'),
                    '-J', 'wrist_2_joint', LaunchConfiguration('spawn_wrist_2_joint'),
                    '-J', 'wrist_3_joint', LaunchConfiguration('spawn_wrist_3_joint'),
                    paused_tag
                ],
                output='screen',
            ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[initial_joint_controller, "-c", "/controller_manager"],
            condition=IfCondition(activate_joint_controller),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
            condition=UnlessCondition(activate_joint_controller),
        )
        ]

        
        return actions + nodes
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])