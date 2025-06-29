#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from interbotix_xs_msgs.msg import JointGroupCommand
from queue import Queue


class JointTrajectoryCH(Node):
    def __init__(self):
        super().__init__('joint_trajectory_command_handler')

        self.declare_parameter('real_robot', False)
        self.declare_parameter('action_cycle_rate', 10.0)
        self.declare_parameter('robot_model', '')

        self.real_robot = self.get_parameter('real_robot').get_parameter_value().bool_value
        ac_rate = self.get_parameter('action_cycle_rate').get_parameter_value().double_value
        self.robot_model = self.get_parameter('robot_model').get_parameter_value().string_value

        if self.real_robot:
            topic = f'{self.robot_model}/commands/joint_group'
            self.jt_pub = self.create_publisher(JointGroupCommand, topic, 10)
        else:
            topic = f'{self.robot_model}/arm_controller/joint_trajectory'
            self.jt_pub = self.create_publisher(JointTrajectory, topic, 10)

        self.create_subscription(JointTrajectory, 'env_arm_command', self.callback_env_joint_trajectory, 10)

        self.queue = Queue(maxsize=1)
        self.stop_flag = False
        self.joints = []

        self.timer = self.create_timer(1.0 / ac_rate, self.joint_trajectory_publisher)

    def callback_env_joint_trajectory(self, msg):
        try:
            self.queue.put_nowait(msg)
        except:
            pass

    def joint_trajectory_publisher(self):
        if self.queue.full():
            traj_msg = self.queue.get()
            self.get_logger().debug(f'Sending trajectory: {traj_msg}')
            if self.real_robot:
                if traj_msg.points:
                    cmd_msg = JointGroupCommand()
                    cmd_msg.name = 'arm'
                    cmd_msg.cmd = list(traj_msg.points[0].positions)
                    self.jt_pub.publish(cmd_msg)
            else:
                self.get_logger().debug('Publishing joint trajectory...')
                self.jt_pub.publish(traj_msg)
            self.stop_flag = False
            self.joints = list(traj_msg.joint_names)
        else:
            if not self.stop_flag:
                if self.real_robot:
                    self.jt_pub.publish(JointGroupCommand())
                else:
                    self.get_logger().debug('Publishing empty joint trajectory...')
                    cmd = JointTrajectory()
                    cmd.joint_names = self.joints
                    self.jt_pub.publish(cmd)
                self.stop_flag = True


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryCH()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
