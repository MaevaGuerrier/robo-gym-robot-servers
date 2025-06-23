#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare all launch arguments
    declared_arguments = [
        # Parameter file arguments
        DeclareLaunchArgument('joint_limit_params', description='YAML file containing the joint limit values'),
        DeclareLaunchArgument('kinematics_params', description='YAML file containing the robot\'s kinematic parameters. These will be different for each robot as they contain the robot\'s calibration.'),
        DeclareLaunchArgument('physical_params', description='YAML file containing the physical parameters of the robots'),
        DeclareLaunchArgument('visual_params', description='YAML file containing the visual model of the robots'),
        DeclareLaunchArgument('initial_positions_file', description='YAML file containing the initial joint positions'),
        
        # Common parameters
        DeclareLaunchArgument('transmission_hw_interface', default_value='hardware_interface/EffortJointInterface', 
                            description='The hardware_interface to expose for each joint in the simulated robot (one of: [PositionJointInterface, VelocityJointInterface, EffortJointInterface])'),
        DeclareLaunchArgument('safety_limits', default_value='false', description='If True, enable the safety limits controller'),
        DeclareLaunchArgument('safety_pos_margin', default_value='0.15', description='The lower/upper limits in the safety controller'),
        DeclareLaunchArgument('safety_k_position', default_value='20', description='Used to set k position in the safety controller'),
        
        # Robot Base position
        DeclareLaunchArgument('x', default_value='0.0', description='base_link x coordinate with respect to the world frame'),
        DeclareLaunchArgument('y', default_value='0.0', description='base_link y coordinate with respect to the world frame'),
        DeclareLaunchArgument('z', default_value='0.1', description='base_link z coordinate with respect to the world frame'),
        DeclareLaunchArgument('roll', default_value='0.0', description='base_link roll with respect to the world frame'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='base_link pitch with respect to the world frame'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='base_link yaw with respect to the world frame'),
        
        # Camera 1 activate and position
        DeclareLaunchArgument('camera1_gazebo', default_value='False', description='use camera1 gazebo simulated sensor'),
        DeclareLaunchArgument('camera1_link_x', default_value='0.0', description='camera1_link x coordinate with respect to the world frame'),
        DeclareLaunchArgument('camera1_link_y', default_value='0.0', description='camera1_link y coordinate with respect to the world frame'),
        DeclareLaunchArgument('camera1_link_z', default_value='0.1', description='camera1_link z coordinate with respect to the world frame'),
        DeclareLaunchArgument('camera1_link_roll', default_value='0.0', description='camera1_link roll with respect to the world frame'),
        DeclareLaunchArgument('camera1_link_pitch', default_value='0.0', description='camera1_link pitch with respect to the world frame'),
        DeclareLaunchArgument('camera1_link_yaw', default_value='0.0', description='camera1_link yaw with respect to the world frame'),
    ]
    
    # Robot description using xacro
    robot_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([FindPackageShare('ur_robot_server'), 'urdf', 'ur.urdf.xacro']),
        ' joint_limit_params:=', LaunchConfiguration('joint_limit_params'),
        ' kinematics_params:=', LaunchConfiguration('kinematics_params'),
        ' physical_params:=', LaunchConfiguration('physical_params'),
        ' visual_params:=', LaunchConfiguration('visual_params'),
        ' initial_positions_file:=', LaunchConfiguration('initial_positions_file'),
        ' transmission_hw_interface:=', LaunchConfiguration('transmission_hw_interface'),
        ' safety_limits:=', LaunchConfiguration('safety_limits'),
        ' safety_pos_margin:=', LaunchConfiguration('safety_pos_margin'),
        ' safety_k_position:=', LaunchConfiguration('safety_k_position'),
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
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('ur_robot_server'), "urdf", 'ur.urdf.xacro']),
            ' joint_limit_params:=', LaunchConfiguration('joint_limit_params'),
            ' kinematics_params:=', LaunchConfiguration('kinematics_params'),
            ' physical_params:=', LaunchConfiguration('physical_params'),
            ' visual_params:=', LaunchConfiguration('visual_params'),
            ' initial_positions_file:=', LaunchConfiguration('initial_positions_file'),
            ' transmission_hw_interface:=', LaunchConfiguration('transmission_hw_interface'),
            ' safety_limits:=', LaunchConfiguration('safety_limits'),
            ' safety_pos_margin:=', LaunchConfiguration('safety_pos_margin'),
            ' safety_k_position:=', LaunchConfiguration('safety_k_position'),
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
            ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }
    
    robot_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            "robot_description": ParameterValue(value=robot_description_content, value_type=str)
        }],
        output='screen'
    )
    
    return LaunchDescription(declared_arguments + [robot_description_node])