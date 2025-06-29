from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    rviz_config = PathJoinSubstitution([
        LaunchConfiguration('rviz_config_path'),
        LaunchConfiguration('rviz_config_file')
    ])

    gui_and_gazebo_gui = PythonExpression([
    "'true' if '", LaunchConfiguration('gui'), "' == 'true' and '", 
    LaunchConfiguration('gazebo_gui'), "' == 'true' else 'false'"
    ])
    gui_and_rviz_gui = PythonExpression([
        "'true' if '", LaunchConfiguration('gui'), "' == 'true' and '",
        LaunchConfiguration('rviz_gui'), "' == 'true' else 'false'"
    ])

    world_name_split= PythonExpression([
        "'", LaunchConfiguration('world_name'), "'.replace('.world', '')"
    ])
    robot_sim = IncludeLaunchDescription(
        condition=UnlessCondition(LaunchConfiguration('real_robot')),
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('interbotix_arm_robot_server'),
                'launch',
                'inc',
                'load_interbotix_arm_sim.launch.py'
            ])
        ),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_name': LaunchConfiguration('robot_name'),
            'gui': LaunchConfiguration('gui'),
            'gazebo_gui': gui_and_gazebo_gui,
            'rviz_gui': gui_and_rviz_gui,
            'gazebo_world': LaunchConfiguration('world_name'),
            'dof': LaunchConfiguration('dof'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'roll': LaunchConfiguration('roll'),
            'pitch': LaunchConfiguration('pitch'),
            'yaw': LaunchConfiguration('yaw'),
            'camera_gazebo': LaunchConfiguration('camera_gazebo'),
            'camera_link_x': LaunchConfiguration('camera_link_x'),
            'camera_link_y': LaunchConfiguration('camera_link_y'),
            'camera_link_z': LaunchConfiguration('camera_link_z'),
            'camera_link_roll': LaunchConfiguration('camera_link_roll'),
            'camera_link_pitch': LaunchConfiguration('camera_link_pitch'),
            'camera_link_yaw': LaunchConfiguration('camera_link_yaw'),
            'show_ar_tag': LaunchConfiguration('show_ar_tag'),
            'show_gripper_bar': LaunchConfiguration('show_gripper_bar'),
            'show_gripper_fingers': LaunchConfiguration('show_gripper_fingers'),
        }.items()
        )
    nodes = [
        Node(
            package='interbotix_arm_robot_server',
            executable='joint_trajectory_command_handler.py',
            name='joint_trajectory_command_handler',
            parameters=[{
                'real_robot': LaunchConfiguration('real_robot'),
                'action_cycle_rate': LaunchConfiguration('action_cycle_rate')
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
            package='interbotix_arm_robot_server',
            executable='robot_server.py',
            name='robot_server',
            parameters=[{
                'server_port': LaunchConfiguration('server_port'),
                'real_robot': LaunchConfiguration('real_robot'),
                'robot_model': LaunchConfiguration('robot_model'),
                'max_velocity_scale_factor': LaunchConfiguration('max_velocity_scale_factor'),
                'action_cycle_rate': LaunchConfiguration('action_cycle_rate'),
                'reference_frame': LaunchConfiguration('reference_frame'),
                'ee_frame': LaunchConfiguration('ee_frame'),
                'target_mode': LaunchConfiguration('target_mode'),
                'rs_mode': LaunchConfiguration('rs_mode'),
                'action_mode': LaunchConfiguration('action_mode'),
                'use_voxel_occupancy': LaunchConfiguration('use_voxel_occupancy'),
                'n_objects': LaunchConfiguration('n_objects'),
                'object_0_frame': LaunchConfiguration('object_0_frame'),
                'object_1_frame': LaunchConfiguration('object_1_frame'),
                'object_0_model_name': LaunchConfiguration('object_0_model_name'),
                'object_1_model_name': LaunchConfiguration('object_1_model_name'),
                'objects_controller': LaunchConfiguration('objects_controller'),
            }],
            output='screen'
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('objects_controller')),
            package='simulation_objects',
            executable='objects_controller.py',
            name='objects_controller',
            parameters=[{
                'real_robot': LaunchConfiguration('real_robot'),
                'reference_frame': LaunchConfiguration('reference_frame'),
                'object_trajectory_file_name': LaunchConfiguration('object_trajectory_file_name'),
                'n_objects': LaunchConfiguration('n_objects'),
                'object_1_model_name': LaunchConfiguration('object_1_model_name'),
                'object_1_frame': LaunchConfiguration('object_1_frame'),
                'object_0_model_name': LaunchConfiguration('object_0_model_name'),
                'object_0_frame': LaunchConfiguration('object_0_frame'),
                'world_name': world_name_split,
            }],
            output='screen'
        ),
    ]
    
    return [robot_sim] + nodes

def generate_launch_description():

    robot_model = LaunchConfiguration('robot_model')

    robot_description_filename = PythonExpression([
        "'", robot_model, ".urdf.xacro'"
    ])

    robot_description_file = PathJoinSubstitution([
        FindPackageShare('interbotix_arm_robot_server'),
        'urdf',
        robot_description_filename
    ])
    robot_controller_filename = PythonExpression([
        "'", robot_model, "_trajectory_controllers.yaml'"
    ])

    declared_arguments = [
        DeclareLaunchArgument('robot_model', default_value='rx150'),
        DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model')),
        DeclareLaunchArgument('real_robot', default_value='false'),
        DeclareLaunchArgument('dof', default_value='5'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz_gui', default_value='false'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='empty.world'),
        DeclareLaunchArgument('max_velocity_scale_factor', default_value='1.0'),
        DeclareLaunchArgument('server_port', default_value='50051'),
        DeclareLaunchArgument('action_cycle_rate', default_value='25.0'),
        DeclareLaunchArgument('reference_frame', default_value=[LaunchConfiguration('robot_model'), '/base_link']),
        DeclareLaunchArgument('ee_frame', default_value='gripper_bar_link'),
        DeclareLaunchArgument('target_mode', default_value='only_robot'),
        DeclareLaunchArgument('rs_mode', default_value=''),
        DeclareLaunchArgument('action_mode', default_value='abs_pos'),
        DeclareLaunchArgument('use_voxel_occupancy', default_value='false'),
        DeclareLaunchArgument('objects_controller', default_value='false'),
        DeclareLaunchArgument('n_objects', default_value='0'),
        DeclareLaunchArgument('object_trajectory_file_name', default_value='no_file'),
        DeclareLaunchArgument('object_0_model_name', default_value=''),
        DeclareLaunchArgument('object_0_frame', default_value=''),
        DeclareLaunchArgument('object_1_model_name', default_value=''),
        DeclareLaunchArgument('object_1_frame', default_value=''),
        DeclareLaunchArgument('rviz_config_file', default_value='ur_rl.rviz'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('show_ar_tag', default_value='false'),
        DeclareLaunchArgument('show_gripper_bar', default_value='true'),
        DeclareLaunchArgument('show_gripper_fingers', default_value='true'),
        DeclareLaunchArgument('use_world_frame', default_value='true'),
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        DeclareLaunchArgument('rviz_config_path', default_value=PathJoinSubstitution([
            FindPackageShare('interbotix_arm_robot_server'), 'rviz'])),
        DeclareLaunchArgument('rviz_config_file', default_value='interbotix_rl.rviz'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('camera_gazebo', default_value='False'),
        DeclareLaunchArgument('camera_link_x', default_value='0.0'),
        DeclareLaunchArgument('camera_link_y', default_value='0.0'),
        DeclareLaunchArgument('camera_link_z', default_value='0.0'),
        DeclareLaunchArgument('camera_link_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_link_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_link_yaw', default_value='0.0'),
        DeclareLaunchArgument('robot_description_file', default_value=robot_description_file),
        DeclareLaunchArgument('robot_description_filename', default_value=robot_description_filename),
        DeclareLaunchArgument('controllers_file', default_value=PathJoinSubstitution([
            FindPackageShare('interbotix_arm_robot_server'),
            'config',
            robot_controller_filename
        ])),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
