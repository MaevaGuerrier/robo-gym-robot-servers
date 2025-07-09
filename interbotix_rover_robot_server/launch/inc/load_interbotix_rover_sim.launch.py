import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import launch_ros.descriptions


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration('robot_model')
    robot_name = LaunchConfiguration('robot_name')
    show_ar_tag = LaunchConfiguration("show_ar_tag")
    arm_model = LaunchConfiguration('arm_model')
    show_gripper_bar = LaunchConfiguration('show_gripper_bar')
    show_gripper_fingers = LaunchConfiguration('show_gripper_fingers')
    use_world_frame = LaunchConfiguration('use_world_frame')
    world_name = LaunchConfiguration("world_name")
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
    robot_description_filename = LaunchConfiguration("robot_description_filename")
    controllers_file = LaunchConfiguration("controllers_file")
    hardware_type = LaunchConfiguration("hardware_type")
    base_model = LaunchConfiguration("base_model")

    mesh_path = FindPackageShare('interbotix_xslocobot_descriptions').perform(context)

    share_path = os.path.dirname(mesh_path)
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare('interbotix_rover_robot_server'), "config", controllers_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare('interbotix_rover_robot_server'), 'urdf', robot_description_filename]
        ),
        ' robot_name:=', robot_name,
        ' robot_model:=', robot_model,
        ' arm_model:=', arm_model,
        ' show_ar_tag:=', show_ar_tag,
        ' show_gripper_bar:=', show_gripper_bar,
        ' show_gripper_fingers:=', show_gripper_fingers,
        ' use_world_frame:=', use_world_frame,
        ' x:=', x,
        ' y:=', y,
        ' z:=', z,
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

    robot_description = {"robot_description": launch_ros.descriptions.ParameterValue(robot_description_content)}

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
        arguments=["arm_controller", "gripper_controller", "diffdrive_controller", "-c", "controller_manager"],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "interbotix_rover",
            "-allow_renaming",
            "true",
            "-z",
            z,
        ],
    )

    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world_name]}.items(),
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
    )

    gz_launch_description_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -s -r -v 4 ", world_name]}.items(),
        condition=UnlessCondition(LaunchConfiguration('gazebo_gui')),
    )

    world_name_split = PythonExpression([
        "'", LaunchConfiguration('world_name'), "'.replace('.world', '')"
    ])

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            f"/world/{world_name_split.perform(context)}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
            f"/world/{world_name_split.perform(context)}/model/interbotix_rover/link/camera_locobot_link/sensor/camera_frame_sensor/image" + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image',
            '/world/empty/model/interbotix_rover/link/camera_locobot_link/sensor/camera_frame_sensor/depth_image/points' + '@sensor_msgs/msg/PointCloud2' + '[ignition.msgs.PointCloudPacked'
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
        DeclareLaunchArgument('arm_model', default_value=''),
        DeclareLaunchArgument('show_gripper_bar', default_value='true'),
        DeclareLaunchArgument('show_gripper_fingers', default_value='true'),
        DeclareLaunchArgument('show_ar_tag', default_value='true'),
        DeclareLaunchArgument('use_world_frame', default_value='true'),
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world_name', default_value=''),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('recording', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_position_controllers', default_value='false'),
        DeclareLaunchArgument('use_trajectory_controllers', default_value='true'),
        DeclareLaunchArgument('dof', default_value='5'),
        DeclareLaunchArgument('robot_controller_filename', default_value=''),
        DeclareLaunchArgument('controllers_file', default_value=''),
        DeclareLaunchArgument('hardware_type', default_value='gz_ignition'),
        DeclareLaunchArgument('action_cycle_rate', default_value='25'),
        DeclareLaunchArgument('server_port', default_value='50051'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('camera_gazebo', default_value='false'),
        DeclareLaunchArgument('camera_link_x', default_value='0.0'),
        DeclareLaunchArgument('camera_link_y', default_value='0.0'),
        DeclareLaunchArgument('camera_link_z', default_value='0.0'),
        DeclareLaunchArgument('camera_link_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_link_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_link_yaw', default_value='0.0'),
        DeclareLaunchArgument('base_model', default_value='create3'),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])