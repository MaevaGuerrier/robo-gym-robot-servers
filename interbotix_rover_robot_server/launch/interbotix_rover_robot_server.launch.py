from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')
    dof = LaunchConfiguration('dof')
    real_robot = LaunchConfiguration('real_robot')
    gui = LaunchConfiguration('gui')
    rviz_gui = LaunchConfiguration('rviz_gui')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    world_name = LaunchConfiguration('world_name')
    server_port = LaunchConfiguration('server_port')
    action_cycle_rate = LaunchConfiguration('action_cycle_rate')
    reference_frame = LaunchConfiguration('reference_frame')
    arm_model = LaunchConfiguration('arm_model')
    show_lidar = LaunchConfiguration('show_lidar')
    show_gripper_bar = LaunchConfiguration('show_gripper_bar')
    show_gripper_fingers = LaunchConfiguration('show_gripper_fingers')
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    camera_gazebo = LaunchConfiguration("camera_gazebo")
    camera_link_x = LaunchConfiguration("camera_link_x")
    camera_link_y = LaunchConfiguration("camera_link_y")
    camera_link_z = LaunchConfiguration("camera_link_z")
    camera_link_roll = LaunchConfiguration("camera_link_roll")
    camera_link_pitch = LaunchConfiguration("camera_link_pitch")
    camera_link_yaw = LaunchConfiguration("camera_link_yaw")
    show_ar_tag = LaunchConfiguration("show_ar_tag")
    n_objects = LaunchConfiguration("n_objects")
    object_trajectory_file_name = LaunchConfiguration("object_trajectory_file_name")
    object_0_model_name = LaunchConfiguration("object_0_model_name")
    object_1_model_name = LaunchConfiguration("object_1_model_name")
    object_0_frame = LaunchConfiguration("object_0_frame")
    object_1_frame = LaunchConfiguration("object_1_frame")
    objects_controller = LaunchConfiguration("objects_controller")
    controllers_file = LaunchConfiguration("controllers_file")
    context_size = LaunchConfiguration("context_size")

    rviz_config = PathJoinSubstitution([
        LaunchConfiguration('rviz_config_path'),
        LaunchConfiguration('rviz_config_file')
    ])

    gui_and_gazebo_gui = PythonExpression([
    "'true' if '", LaunchConfiguration('gui'), "' == 'true' and '",
    gazebo_gui, "' == 'true' else 'false'"
    ])
    gui_and_rviz_gui = PythonExpression([
        "'true' if '", gui, "' == 'true' and '",
        rviz_gui, "' == 'true' else 'false'"
    ])

    world_name_split= PythonExpression([
        "'", world_name, "'.replace('.world', '')"
    ])

    robot_sim = IncludeLaunchDescription(
        condition=UnlessCondition(LaunchConfiguration('real_robot')),
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('interbotix_rover_robot_server'),
                'launch',
                'inc',
                'load_interbotix_rover_sim.launch.py'
            ])
        ),
        launch_arguments={
            'robot_model': robot_model,
            'robot_name': robot_name,
            'dof': dof,
            'gui':gui,
            'gazebo_gui': gui_and_gazebo_gui,
            'rviz_gui': gui_and_rviz_gui,
            'world_name': world_name,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'camera_gazebo': camera_gazebo,
            'camera_link_x': camera_link_x,
            'camera_link_y': camera_link_y,
            'camera_link_z': camera_link_z,
            'camera_link_roll': camera_link_roll,
            'camera_link_pitch': camera_link_pitch,
            'camera_link_yaw': camera_link_yaw,
            'show_ar_tag': show_ar_tag,
            'show_gripper_bar': show_gripper_bar,
            'show_gripper_fingers': show_gripper_fingers,
            'gazebo_gui': gui_and_gazebo_gui,
            'rviz_gui': gui_and_rviz_gui,
            'arm_model': arm_model,
            'show_lidar': show_lidar,
            'controllers_file': controllers_file
        }.items()
        )

    nodes = [
        Node(
            package='interbotix_rover_robot_server',
            executable='joint_trajectory_command_handler.py',
            name='joint_trajectory_command_handler',
            parameters=[{
                'real_robot': real_robot,
                'action_cycle_rate': action_cycle_rate,
                'robot_model': robot_model
            }],
            output='screen'
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config],
            condition=IfCondition(gui_and_rviz_gui),
        ),
        Node(
            package='interbotix_rover_robot_server',
            executable='robot_server.py',
            name='robot_server',
            parameters=[{
                'server_port': server_port,
                'real_robot': real_robot,
                'robot_model': robot_model,
                'action_cycle_rate': action_cycle_rate,
                'reference_frame': reference_frame,
                'context_size': context_size,
            }],
            output='screen',
            remappings=[
                ('/locobot/odom', '/locobot/mobile_base/odom'),
                ('/locobot/cmd_vel', '/locobot/mobile_base/cmd_vel'),
                ('/joint_states', '/locobot/joint_states')
            ]
        ),
        Node(
            condition=IfCondition(objects_controller),
            package='simulation_objects',
            executable='objects_controller.py',
            name='objects_controller',
            parameters=[{
                'real_robot': real_robot,
                'reference_frame': reference_frame,
                'object_trajectory_file_name': object_trajectory_file_name,
                'n_objects': n_objects,
                'object_1_model_name': object_1_model_name,
                'object_1_frame': object_1_frame,
                'object_0_model_name': object_0_model_name,
                'object_0_frame': object_0_frame,
                'world_name': world_name_split,
            }],
            output='screen'
        ),
    ]

    return [robot_sim] + nodes

def generate_launch_description():

    arm_model = PythonExpression([
        "'mobile_' + '", LaunchConfiguration('robot_model'), "'.split('_')[1]"
    ])

    robot_description_filename = "locobot.urdf.xacro"

    robot_description_file = PathJoinSubstitution([
        FindPackageShare('interbotix_rover_robot_server'),
        'urdf',
        robot_description_filename
    ])
    robot_controller_filename = PythonExpression([
        "'", arm_model, "_trajectory_controllers.yaml'"
    ])

    declared_arguments = [
        DeclareLaunchArgument('robot_model', default_value='locobot_wx250s'),
        DeclareLaunchArgument('robot_name', default_value='locobot'),
        DeclareLaunchArgument('dof', default_value='6'),
        DeclareLaunchArgument('real_robot', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz_gui', default_value='false'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='empty.world'),
        DeclareLaunchArgument('server_port', default_value='50051'),
        DeclareLaunchArgument('action_cycle_rate', default_value='25'),
        DeclareLaunchArgument('reference_frame', default_value=[LaunchConfiguration('robot_model'), '/base_link']),
        DeclareLaunchArgument('arm_model', default_value=arm_model),
        DeclareLaunchArgument('show_lidar', default_value='false'),
        DeclareLaunchArgument('show_ar_tag', default_value='true'),
        DeclareLaunchArgument('show_gripper_bar', default_value='true'),
        DeclareLaunchArgument('show_gripper_fingers', default_value='true'),
        DeclareLaunchArgument('rviz_frame', default_value=[LaunchConfiguration('robot_name'), '/base_footprint']),
        DeclareLaunchArgument('base_type', default_value='create3'),
        DeclareLaunchArgument('rviz_config_path', default_value=PathJoinSubstitution([
            FindPackageShare('interbotix_rover_robot_server'), 'rviz'])),
        DeclareLaunchArgument('rviz_config_file', default_value='interbotix_rl.rviz'),
        DeclareLaunchArgument('objects_controller', default_value='false'),
        DeclareLaunchArgument('n_objects', default_value='0'),
        DeclareLaunchArgument('object_trajectory_file_name', default_value='no_file'),
        DeclareLaunchArgument('object_0_model_name', default_value=''),
        DeclareLaunchArgument('object_0_frame', default_value=''),
        DeclareLaunchArgument('object_1_model_name', default_value=''),
        DeclareLaunchArgument('object_1_frame', default_value=''),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('camera_gazebo', default_value='False'),
        DeclareLaunchArgument('camera_link_x', default_value='0.0'),
        DeclareLaunchArgument('camera_link_y', default_value='0.0'),
        DeclareLaunchArgument('camera_link_z', default_value='0.0'),
        DeclareLaunchArgument('camera_link_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_link_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_link_yaw', default_value='0.0'),
        DeclareLaunchArgument('context_size', default_value='1.0'),
        DeclareLaunchArgument('robot_description_file', default_value=robot_description_file),
        DeclareLaunchArgument('robot_description_filename', default_value=robot_description_filename),
        DeclareLaunchArgument('controllers_file', default_value=PathJoinSubstitution([
            FindPackageShare('interbotix_rover_robot_server'),
            'config',
            robot_controller_filename
        ])),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
