entry_points={
    'console_scripts': [
        'robot_server = ur_robot_server.robot_server:main',
        'joint_trajectory_command_handler = ur_robot_server.joint_trajectory_command_handler:main',
    ],
},