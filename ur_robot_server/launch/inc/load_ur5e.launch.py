from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('joint_limit_params', default_value=PathJoinSubstitution([
            FindPackageShare('ur_description'), 'config', 'ur5e', 'joint_limits.yaml'])),
        DeclareLaunchArgument('kinematics_params', default_value=PathJoinSubstitution([
            FindPackageShare('ur_description'), 'config', 'ur5e', 'default_kinematics.yaml'])),
        DeclareLaunchArgument('physical_params', default_value=PathJoinSubstitution([
            FindPackageShare('ur_description'), 'config', 'ur5e', 'physical_parameters.yaml'])),
        DeclareLaunchArgument('visual_params', default_value=PathJoinSubstitution([
            FindPackageShare('ur_description'), 'config', 'ur5e', 'visual_parameters.yaml'])),
        DeclareLaunchArgument('initial_positions_file', default_value=PathJoinSubstitution([
            FindPackageShare('ur_description'), 'config', 'initial_positions.yaml'])),
        DeclareLaunchArgument('transmission_hw_interface', default_value='hardware_interface/EffortJointInterface',
                            description='The hardware_interface to expose for each joint in the simulated robot (one of: [PositionJointInterface, VelocityJointInterface, EffortJointInterface])'),
        DeclareLaunchArgument('safety_limits', default_value='false', description='If True, enable the safety limits controller'),
        DeclareLaunchArgument('safety_pos_margin', default_value='0.15', description='The lower/upper limits in the safety controller'),
        DeclareLaunchArgument('safety_k_position', default_value='20', description='Used to set k position in the safety controller'),

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
    
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                get_package_share_directory('ur_robot_server'),
                'launch',
                'inc',
                'load_ur.launch.py'
            ])),
            launch_arguments={
                'joint_limit_params': LaunchConfiguration('joint_limit_params'),
                'kinematics_params': LaunchConfiguration('kinematics_params'),
                'physical_params': LaunchConfiguration('physical_params'),
                'visual_params': LaunchConfiguration('visual_params'),
                'initial_positions_file': LaunchConfiguration('initial_positions_file'),
                'transmission_hw_interface': LaunchConfiguration('transmission_hw_interface'),
                'safety_limits': LaunchConfiguration('safety_limits'),
                'safety_pos_margin': LaunchConfiguration('safety_pos_margin'),
                'safety_k_position': LaunchConfiguration('safety_k_position'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'roll': LaunchConfiguration('roll'),
                'pitch': LaunchConfiguration('pitch'),
                'yaw': LaunchConfiguration('yaw'),
                'camera1_gazebo': LaunchConfiguration('camera1_gazebo'),
                'camera1_link_x': LaunchConfiguration('camera1_link_x'),
                'camera1_link_y': LaunchConfiguration('camera1_link_y'),
                'camera1_link_z': LaunchConfiguration('camera1_link_z'),
                'camera1_link_roll': LaunchConfiguration('camera1_link_roll'),
                'camera1_link_pitch': LaunchConfiguration('camera1_link_pitch'),
                'camera1_link_yaw': LaunchConfiguration('camera1_link_yaw'),
            }.items()
        )
    ]
    
    return LaunchDescription(declared_arguments + actions)