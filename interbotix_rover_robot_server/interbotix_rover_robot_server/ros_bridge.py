#!/usr/bin/env python

from geometry_msgs.msg import Twist, Pose
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Header
import copy
from threading import Event
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2
import numpy as np
import time
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
import cv2
from cv_bridge import CvBridge
import base64


class InterbotixRoverRosBridge:

    def __init__(self, node):
        self.node = node
        self.node.get_logger().info('Initializing InterbotixRoverRosBridge...')
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        self.node.declare_parameter('action_cycle_rate', 125.0)
        self.node.declare_parameter('camera', True)
        self.node.declare_parameter('image_width', 160)
        self.node.declare_parameter('image_height', 120)
        self.node.declare_parameter('resize_image', False)
        self.node.declare_parameter('context_size', 3)

        self.real_robot = self.node.get_parameter('real_robot').value
        self.robot_model = self.node.get_parameter('robot_model').value
        self.camera = self.node.get_parameter('camera').value
        self.image_width = self.node.get_parameter('image_width').value
        self.image_height = self.node.get_parameter('image_height').value
        self.resize_image = self.node.get_parameter('resize_image').value
        self.context_size = self.node.get_parameter('context_size').value

        self.context_queue = []

        self.robot_name, arm_model = self.robot_model.split('_')

        if arm_model == 'wx250s':
            self.dof = 6
        elif arm_model == 'px100':
            self.dof = 4
        else:
            self.dof = 5

        if self.real_robot:
            self.base_cmd_pub = self.node.create_publisher(Twist, self.robot_name + '/cmd_vel', 1)
        else:
            self.base_cmd_pub = self.node.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 1)

        if self.real_robot:
            # For real robot, use topic
            self.arm_cmd_pub = self.node.create_publisher(JointTrajectory, self.robot_name + '/env_arm_command', 1)
            self.use_arm_actions = False
        else:
            # For simulation, use action client
            self.arm_action_client = ActionClient(self.node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
            self.node.get_logger().info('Waiting for arm action server...')
            self.arm_action_client.wait_for_server()
            self.node.get_logger().info('Arm action server connected!')
            self.use_arm_actions = True
            self.current_arm_goal_handle = None

        self.rate = self.node.get_parameter('action_cycle_rate').value
        self.control_rate = self.node.create_timer(1.0 / self.rate, lambda: None)

        if self.dof == 4:
            self.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle']
            self.joint_goal_names = ['base_joint_position', 'shoulder_joint_position', 'elbow_joint_position',
                                         'wrist_angle_joint_position']
        elif self.dof == 5:
            self.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
            self.joint_goal_names = ['base_joint_position', 'shoulder_joint_position', 'elbow_joint_position',
                                         'wrist_angle_joint_position', 'wrist_rotate_joint_position']
        elif self.dof == 6:
            self.joint_names = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
            self.joint_goal_names = ['base_joint_position', 'shoulder_joint_position', 'elbow_joint_position',
                                         'forearm_roll_joint_position', 'wrist_angle_joint_position',
                                         'wrist_rotate_joint_position']

        self.base_joint_names = ['right_wheel_joint', 'left_wheel_joint']

        self.joint_position = dict.fromkeys(self.joint_names + self.base_joint_names, 0.0)
        self.joint_velocity = dict.fromkeys(self.joint_names + self.base_joint_names, 0.0)

        self.base_pose = Pose()

        self.node.create_subscription(JointState,  "/joint_states", self._on_joint_states, 10)
        self.node.create_subscription(Odometry, self.robot_name + "/odom", self._on_odom, 10)
        if self.camera:
            self.node.create_subscription(Image, "/world/empty/model/interbotix_rover/link/camera_locobot_link/sensor/camera_frame_sensor/image", self._on_image, 10)

        self.image = None
        self.bridge = CvBridge()
        self.min_traj_duration = 0.5
        self.joint_velocity_limits = self._get_joint_velocity_limits()

        self.arm_goal_completed = Event()
        self.arm_goal_success = False

    def get_state(self):
        self.get_state_event.clear()

        state = []
        state_dict = {}
        joint_position = copy.deepcopy(self.joint_position)
        joint_velocity = copy.deepcopy(self.joint_velocity)

        state += self._get_joint_ordered_value_list(joint_position)
        state += self._get_joint_ordered_value_list(joint_velocity)
        
        base_pose = [self.base_pose.position.x, self.base_pose.position.y, self.base_pose.position.z, 
                     self.base_pose.orientation.x, self.base_pose.orientation.y, self.base_pose.orientation.z, self.base_pose.orientation.w]
        state += base_pose

        string_params = {}

        state_dict.update(self._get_joint_states_dict(joint_position, joint_velocity, base_pose))
        for i, encoded_image in enumerate(self.context_queue):
            string_params[f"camera_image_{i}"] = encoded_image

        string_params["image_count"] = str(len(self.context_queue))
        string_params["context_size"] = str(self.context_size)

        self.get_state_event.set()
        msg = robot_server_pb2.State(
            state=state,
            state_dict=state_dict,
            string_params=string_params,
            success=True
        )
        return msg

    def set_state(self, state_msg):
        if all(j in state_msg.state_dict for j in self.joint_goal_names):
            state_dict = True
        else:
            state_dict = False 

        self.reset.clear()

        if state_dict:
            goal_joint_states = [state_msg.state_dict[joint] for joint in self.joint_goal_names]
        else:
            goal_joint_states = state_msg.state[6:12]

        self.set_initial_position(goal_joint_states)
        self.reset.set()

        for _ in range(20):
            time.sleep(1/self.rate)

        return 1

    def send_action(self, action):
        # split base command from arm command
        executed_action = self.publish_env_arm_cmd(action[:-2])
        executed_action_base = self.publish_env_base_cmd(action[-2:])
        return executed_action + executed_action_base

    def set_initial_position(self, goal_joint_position):
        """Set robot joint positions to a desired value, called on reset"""
        arm_goal = goal_joint_position

        if self.use_arm_actions:
            success = self._send_arm_action_and_wait(arm_goal)
            if not success:
                self.node.get_logger().error('Failed to reach initial arm position')
        else:
            arm_position_reached = False
            while not arm_position_reached:
                self.publish_env_arm_cmd(arm_goal)
                self.get_state_event.clear()
                joint_position = copy.deepcopy(self.joint_position)
                arm = {k: joint_position[k] for k in joint_position.keys() - {'right_wheel_joint', 'left_wheel_joint'}}
                arm_position_reached = np.isclose(arm_goal, self._get_joint_ordered_value_list(arm), atol=0.07).all()
                self.get_state_event.set()

    def publish_env_arm_cmd(self, position_cmd):
        """Publish environment arm command using actions or topics"""

        if self.use_arm_actions:
            # Use action client for simulation
            success = self._send_arm_action_and_wait(position_cmd)
            if not success:
                self.node.get_logger().error('Arm action failed')
        else:
            # Use topic publisher for real robot
            msg = JointTrajectory()
            msg.header = Header()
            msg.joint_names = self.joint_names
            msg.points = [JointTrajectoryPoint()]
            msg.points[0].positions = position_cmd
            dur = []
            for idx, name in enumerate(msg.joint_names):
                pos = self.joint_position[name]
                cmd = position_cmd[idx]
                max_vel = self.joint_velocity_limits[name]
                dur.append(max(abs(cmd-pos)/max_vel, self.min_traj_duration))
            msg.points[0].time_from_start = Duration(seconds=max(dur)).to_msg()
            self.arm_cmd_pub.publish(msg)
            time.sleep(1/self.rate)

        return position_cmd

    def _send_arm_action_and_wait(self, position_cmd, timeout=5.0):
        """Send arm action and wait for completion"""

        if self.current_arm_goal_handle is not None:
            self.current_arm_goal_handle.cancel_goal_async()

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = position_cmd

        dur = []
        for idx, name in enumerate(trajectory.joint_names):
            pos = self.joint_position[name]
            cmd = position_cmd[idx]
            max_vel = self.joint_velocity_limits[name]
            dur.append(max(abs(cmd-pos)/max_vel, self.min_traj_duration))

        point.time_from_start = Duration(seconds=max(dur)).to_msg()
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.arm_goal_completed.clear()
        self.arm_goal_success = False

        send_goal_future = self.arm_action_client.send_goal_async(goal)

        start_time = time.time()
        while not send_goal_future.done():
            if time.time() - start_time > 1.0:  # 1 second timeout for goal acceptance
                self.node.get_logger().error('Timeout waiting for goal acceptance')
                return False
            rclpy.spin_once(self.node, timeout_sec=0.01)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Goal was rejected!')
            return False

        self.current_arm_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._arm_result_callback)

        if self.arm_goal_completed.wait(timeout):
            return self.arm_goal_success
        else:
            self.node.get_logger().error('Timeout waiting for arm action completion')
            return False

    def _arm_result_callback(self, future):
        """Callback for arm action result"""
        result = future.result().result
        self.arm_goal_success = (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
        self.current_arm_goal_handle = None
        self.arm_goal_completed.set()

        if self.arm_goal_success:
            self.node.get_logger().debug('Arm action completed successfully')
        else:
            self.node.get_logger().error(f'Arm action failed with error: {result.error_code}')

    def publish_env_base_cmd(self, goal):
        msg = Twist()
        msg.linear.x = goal[0]
        msg.angular.z = goal[1]
        self.base_cmd_pub.publish(msg)

        time.sleep(1/self.rate)

        return goal

    def _on_joint_states(self, msg):
        if self.get_state_event.is_set():
            for idx, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.joint_position[name] = msg.position[idx]
                    self.joint_velocity[name] = msg.velocity[idx]

    def _on_odom(self, msg):
        if self.get_state_event.is_set():
            self.base_pose = msg.pose.pose

    def _on_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.resize_image:
            image_array = cv2.resize(cv_image, (self.image_width, self.image_height), interpolation=cv2.INTER_LINEAR)
        else:
            image_array = cv_image
        _, buffer = cv2.imencode('.png', image_array)
        image_bytes = buffer.tobytes()

        image_string = base64.b64encode(image_bytes).decode('utf-8')
        if len(self.context_queue) < self.context_size + 1:
            self.context_queue.append(image_string)
        else:
            self.context_queue.pop(0)
            self.context_queue.append(image_string)

    def _get_joint_states_dict(self, joint_position, joint_velocity, base_pose):
        d = {}
        if self.dof <= 6:
            d['base_joint_position'] = joint_position['waist']
            d['shoulder_joint_position'] = joint_position['shoulder']
            d['elbow_joint_position'] = joint_position['elbow']
            d['wrist_angle_joint_position'] = joint_position['wrist_angle']
            d['base_joint_velocity'] = joint_velocity['waist']
            d['shoulder_joint_velocity'] = joint_velocity['shoulder']
            d['elbow_joint_velocity'] = joint_velocity['elbow']
            d['wrist_angle_joint_velocity'] = joint_velocity['wrist_angle']
        if self.dof >= 5:
            d['wrist_rotate_joint_position'] = joint_position['wrist_rotate']
            d['wrist_rotate_joint_velocity'] = joint_velocity['wrist_rotate']
        if self.dof >= 6:
            d['forearm_roll_joint_position'] = joint_position['forearm_roll']
            d['forearm_roll_joint_velocity'] = joint_velocity['forearm_roll']

        d['right_wheel_joint_position'] = joint_position['right_wheel_joint']
        d['left_wheel_joint_position'] = joint_position['left_wheel_joint']
        d['right_wheel_joint_velocity'] = joint_velocity['right_wheel_joint']
        d['left_wheel_joint_velocity'] = joint_velocity['left_wheel_joint']
        d['base_position_x'] = base_pose[0]
        d['base_position_y'] = base_pose[1]
        d['base_position_z'] = base_pose[2]
        d['base_orientation_x'] = base_pose[3]
        d['base_orientation_y'] = base_pose[4]
        d['base_orientation_z'] = base_pose[5]
        d['base_orientation_w'] = base_pose[6]

        return d 

    def _get_joint_ordered_value_list(self, joint_values):
        return [joint_values[name] for name in self.joint_names]

    def _get_joint_velocity_limits(self):
        if self.dof == 4:
            absolute_joint_velocity_limits = {'waist': 2.35, 'shoulder': 2.35, 'elbow': 2.35, \
                                              'forearm_roll': 2.35, 'wrist_angle': 2.35, 'wrist_rotate': 2.35}
        elif self.dof == 5:
            absolute_joint_velocity_limits = {'waist': 2.35, 'shoulder': 2.35, 'elbow': 2.35, \
                                              'forearm_roll': 2.35, 'wrist_angle': 2.35, 'wrist_rotate': 2.35}
        elif self.dof == 6:
            absolute_joint_velocity_limits = {'waist': 2.35, 'shoulder': 2.35, 'elbow': 2.35, \
                                              'forearm_roll': 2.35, 'wrist_angle': 2.35, 'wrist_rotate': 2.35}
        else:
            raise ValueError('robot_model not recognized')

        return {name: absolute_joint_velocity_limits[name] for name in self.joint_names}