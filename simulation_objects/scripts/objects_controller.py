#!/usr/bin/env python
import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from scipy import signal, interpolate
import numpy as np
import copy
import os
import random
from tf2_ros import StaticTransformBroadcaster
import json
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, ListParameters
from ros_gz_interfaces.srv import SetEntityPose
from rclpy.callback_groups import ReentrantCallbackGroup


class ObjectsController(Node):
    def __init__(self):
        super().__init__("objects_controller")

        self.move = False
        self.objects_trajectories = []
        self.trajectories_lens = []
        self.trajectory_indices = None

        self.declare_parameter("real_robot", False)
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("object_trajectory_file_name", "no_file")
        self.declare_parameter("n_objects", 1)
        self.declare_parameter("object_0_model_name", "")
        self.declare_parameter("object_0_frame", "")
        self.declare_parameter("object_1_model_name", "")
        self.declare_parameter("object_1_frame", "")
        self.declare_parameter("world_name", "default")

        self.real_robot = self.get_parameter("real_robot").value
        self.world_name = self.get_parameter("world_name").value

        if not self.real_robot:
            self.gazebo_set_pose = self.create_client(
                SetEntityPose, "/world/" + self.world_name + "/set_pose"
            )

        self.create_subscription(Bool, "move_objects", self.callback_move_objects, 1)

        self.reference_frame = self.get_parameter("reference_frame").value
        self.static_tf2_broadcaster = StaticTransformBroadcaster(self)
        self.future = None

        object_trajectory_file_name = self.get_parameter(
            "object_trajectory_file_name"
        ).value
        if object_trajectory_file_name != "no_file":
            file_path = os.path.join(
                os.path.dirname(__file__),
                "../object_trajectories",
                object_trajectory_file_name + ".json",
            )
            with open(file_path, "r") as json_file:
                self.p = json.load(json_file)

        self.objects_initialization()

        self.timer = self.create_timer(0.01, self.objects_state_update_callback)

        self.cli = self.create_client(GetParameters, "/robot_server/get_parameters", callback_group=ReentrantCallbackGroup())

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.get_logger().info("Objects_controller started")

    def get_params_from_server(self, params_name_list):
        request = GetParameters.Request()
        request.names = params_name_list

        future = self.cli.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            return future.result()
        else:
            self.get_logger().error("Parameter request timed out!")
            return None

    def callback_move_objects(self, data):
        if data.data == True:
            self.move = True
            self.get_logger().debug("Moving objects")
            self.generate_trajectories()
        else:
            self.move = False
            self.get_logger().debug("Stopping object movement")

    def generate_trajectories(self):
        """Generate movement trajectories for all objects"""
        self.objects_trajectories = []
        self.trajectories_lens = []

        for i in range(self.n_objects):
            params = ["object_" + repr(i) + "_function"]
            response = self.get_params_from_server(params)
            if response is None:
                self.get_logger().error("Failed to get parameters")
                return
            function = response.values[0].string_value
            if function == "fixed_position":
                params = [
                    "object_" + repr(i) + "_x",
                    "object_" + repr(i) + "_y",
                    "object_" + repr(i) + "_z",
                ]
                response = self.get_params_from_server(params)
                x, y, z = (
                    response.values[0].double_value,
                    response.values[1].double_value,
                    response.values[2].double_value,
                )
                x_trajectory, y_trajectory, z_trajectory = self.get_fixed_position(
                    x, y, z
                )
            elif function == "triangle_wave":
                params = [
                    "object_" + repr(i) + "_x",
                    "object_" + repr(i) + "_y",
                    "object_" + repr(i) + "_z_amplitude",
                    "object_" + repr(i) + "_z_frequency",
                    "object_" + repr(i) + "_z_offset",
                ]
                response = self.get_params_from_server(params)
                x, y, a, f, o = (
                    response.values[0].double_value,
                    response.values[1].double_value,
                    response.values[2].double_value,
                    response.values[3].double_value,
                    response.values[4].double_value,
                )
                x_trajectory, y_trajectory, z_trajectory = self.get_triangle_wave(
                    x, y, a, f, o
                )
            elif function == "3d_spline":
                params = [
                    "object_" + repr(i) + "_x_min",
                    "object_" + repr(i) + "_x_max",
                    "object_" + repr(i) + "_y_min",
                    "object_" + repr(i) + "_y_max",
                    "object_" + repr(i) + "_z_min",
                    "object_" + repr(i) + "_z_max",
                    "object_" + repr(i) + "_n_points",
                    "object_" + repr(i) + "_n_sampling_points",
                ]
                response = self.get_params_from_server(params)
                (x_min,
                x_max,
                y_min,
                y_max,
                z_min,
                z_max,
                n_points,
                n_sampling_points) = (
                    response.values[0].double_value,
                    response.values[1].double_value,
                    response.values[2].double_value,
                    response.values[3].double_value,
                    response.values[4].double_value,
                    response.values[5].double_value,
                    response.values[6].integer_value,
                    response.values[7].integer_value)
                x_trajectory, y_trajectory, z_trajectory = self.get_3d_spline(
                    x_min,
                    x_max,
                    y_min,
                    y_max,
                    z_min,
                    z_max,
                    n_points,
                    n_sampling_points,
                )
            elif function == "3d_spline_ur5_workspace":
                params = [
                    "object_" + repr(i) + "_x_min",
                    "object_" + repr(i) + "_x_max",
                    "object_" + repr(i) + "_y_min",
                    "object_" + repr(i) + "_y_max",
                    "object_" + repr(i) + "_z_min",
                    "object_" + repr(i) + "_z_max",
                    "object_" + repr(i) + "_n_points",
                    "object_" + repr(i) + "_n_sampling_points",
                ]
                response = self.get_params_from_server(params)
                (x_min,
                x_max,
                y_min,
                y_max,
                z_min,
                z_max,
                n_points,
                n_sampling_points) = (
                    response.values[0].double_value,
                    response.values[1].double_value,
                    response.values[2].double_value,
                    response.values[3].double_value,
                    response.values[4].double_value,
                    response.values[5].double_value,
                    response.values[6].integer_value,
                    response.values[7].integer_value,
                )
                x_trajectory, y_trajectory, z_trajectory = (
                    self.get_3d_spline_ur5_workspace(
                        x_min,
                        x_max,
                        y_min,
                        y_max,
                        z_min,
                        z_max,
                        n_points,
                        n_sampling_points,
                    )
                )
            elif function == "fixed_trajectory":
                params = ["object_" + repr(i) + "_trajectory_id"]
                response = self.get_params_from_server(params)
                trajectory_id = response.values[0].integer_value
                x_trajectory, y_trajectory, z_trajectory = self.get_fixed_trajectory(
                    trajectory_id
                )
            elif function == "fixed_position_ab":
                params = [
                    "object_" + repr(i) + "_x_a",
                    "object_" + repr(i) + "_y_a",
                    "object_" + repr(i) + "_z_a",
                    "object_" + repr(i) + "_x_b",
                    "object_" + repr(i) + "_y_b",
                    "object_" + repr(i) + "_z_b",
                    "object_" + repr(i) + "_hold_a",
                    "object_" + repr(i) + "_hold_b",
                ]
                response = self.get_params_from_server(params)
                x_a, y_a, z_a, x_b, y_b, z_b, hold_a, hold_b = (
                    response.values[0].double_value,
                    response.values[1].double_value,
                    response.values[2].double_value,
                    response.values[3].double_value,
                    response.values[4].double_value,
                    response.values[5].double_value,
                    response.values[6].bool_value,
                    response.values[7].bool_value)
                x_trajectory, y_trajectory, z_trajectory = self.get_fixed_position_a_b(
                    x_a, y_a, z_a, x_b, y_b, z_b, hold_a, hold_b
                )
            elif function == "interpolated_abc":
                params = [
                    "object_" + repr(i) + "_x_a",
                    "object_" + repr(i) + "_y_a",
                    "object_" + repr(i) + "_z_a",
                    "object_" + repr(i) + "_x_b",
                    "object_" + repr(i) + "_y_b",
                    "object_" + repr(i) + "_z_b",
                    "object_" + repr(i) + "_x_c",
                    "object_" + repr(i) + "_y_c",
                    "object_" + repr(i) + "_z_c",
                    "object_" + repr(i) + "_hold_a",
                    "object_" + repr(i) + "_hold_b",
                    "object_" + repr(i) + "_hold_c",
                    "object_" + repr(i) + "_n_sampling_points_ab",
                    "object_" + repr(i) + "_n_sampling_points_bc",
                ]
                response = self.get_params_from_server(params)
                (
                    x_a,
                    y_a,
                    z_a,
                    x_b,
                    y_b,
                    z_b,
                    x_c,
                    y_c,
                    z_c,
                    hold_a,
                    hold_b,
                    hold_c,
                    n_sampling_points_ab,
                    n_sampling_points_bc,
                ) = (
                    response.values[0].double_value,
                    response.values[1].double_value,
                    response.values[2].double_value,
                    response.values[3].double_value,
                    response.values[4].double_value,
                    response.values[5].double_value,
                    response.values[6].double_value,
                    response.values[7].double_value,
                    response.values[8].double_value,
                    response.values[9].bool_value,
                    response.values[10].bool_value,
                    response.values[11].bool_value,
                    response.values[12].integer_value,
                    response.values[13].integer_value)
                x_trajectory, y_trajectory, z_trajectory = self.get_interpolated_a_b_c(
                    x_a,
                    y_a,
                    z_a,
                    x_b,
                    y_b,
                    z_b,
                    x_c,
                    y_c,
                    z_c,
                    hold_a,
                    hold_b,
                    hold_c,
                    n_sampling_points_ab,
                    n_sampling_points_bc,
                )
            else:
                self.get_logger().error(
                    'Object trajectory function "' + function + '" not recognized'
                )
                continue

            self.objects_trajectories.append([x_trajectory, y_trajectory, z_trajectory])
            self.trajectories_lens.append(len(x_trajectory))

        self.trajectory_indices = np.zeros((self.n_objects,), dtype=int)

    def objects_state_update_callback(self):
        """Timer callback for updating object states"""
        if self.move and self.objects_trajectories:
            self.trajectory_indices = np.mod(
                self.trajectory_indices, self.trajectories_lens
            )

            for i in range(self.n_objects):
                idx = self.trajectory_indices[i]
                x_pos = self.objects_trajectories[i][0][idx]
                y_pos = self.objects_trajectories[i][1][idx]
                z_pos = self.objects_trajectories[i][2][idx]

                # Update Gazebo model if not real robot
                if not self.real_robot:
                    self.objects_pose[i].pose.position.x = float(x_pos)
                    self.objects_pose[i].pose.position.y = float(y_pos)
                    self.objects_pose[i].pose.position.z = float(z_pos)
                    self.gazebo_set_pose.call_async(self.objects_pose[i])

                t = TransformStamped()
                t.header.frame_id = self.reference_frame
                t.header.stamp = self.get_clock().now().to_msg()
                t.child_frame_id = self.objects_tf_frame[i]
                t.transform.translation.x = x_pos
                t.transform.translation.y = y_pos
                t.transform.translation.z = z_pos
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                self.static_tf2_broadcaster.sendTransform(t)

            self.trajectory_indices += 1
        else:
            self.move_objects_up()

    def get_fixed_position(self, x, y, z):
        """Generate trajectory for object in a fixed position

        Args:
            x (float): x coordinate (m).
            y (float): y coordinate (m).
            z (float): z coordinate (m).

        Returns:
            list: x coordinate function
            list: y coordinate function
            list: z coordinate function
        """
        x_function = [x]
        y_function = [y]
        z_function = [z]

        return x_function, y_function, z_function

    def get_triangle_wave(self, x, y, amplitude, frequency, offset):
        """Generate samples of triangle wave function with amplitude in the z axis direction.

        Args:
            x (float): x coordinate (m).
            y (float): y coordinate (m).
            amplitude (float): amplitude of the triangle wave (m).
            frequency (float): frequency of the triangle wave (Hz).
            offset (float): offset from the ground of the zero of the triangle wave (m).


        Returns:
            np.array: Samples of the x coordinate of the function over time.
            np.array: Samples of the y coordinate of the function over time.
            np.array: Samples of the z coordinate of the function over time.

        """

        # Create array with time samples over 1 full function period
        sampling_rate = copy.deepcopy(self.update_rate)
        samples_len = int(sampling_rate / frequency)
        t = np.linspace(0, (1 / frequency), samples_len)

        x_function = np.full(samples_len, x)
        y_function = np.full(samples_len, y)
        z_function = offset + amplitude * signal.sawtooth(
            2 * np.pi * frequency * t, 0.5
        )

        return x_function, y_function, z_function

    def get_3d_spline(
        self,
        x_min,
        x_max,
        y_min,
        y_max,
        z_min,
        z_max,
        n_points=10,
        n_sampling_points=4000,
    ):
        """Generate samples of the cartesian coordinates of a 3d spline.

        Args:
            x_min (float): min x coordinate of random points used to interpolate spline (m).
            x_max (float): max x coordinate of random points used to interpolate spline (m).
            y_min (float): min y coordinate of random points used to interpolate spline (m).
            y_max (float): max y coordinate of random points used to interpolate spline (m).
            z_min (float): min z coordinate of random points used to interpolate spline (m).
            z_max (float): max z coordinate of random points used to interpolate spline (m).
            n_points (int): number of random points used to interpolate the 3d spline.
            n_sampling_points (int): number of the samples to take over the whole length of the spline.

        Returns:
            np.array: Samples of the x coordinate of the function over time.
            np.array: Samples of the y coordinate of the function over time.
            np.array: Samples of the z coordinate of the function over time.

        """

        n_points = int(n_points)
        n_sampling_points = int(n_sampling_points)

        x = np.random.uniform(x_min, x_max, n_points)
        y = np.random.uniform(y_min, y_max, n_points)
        z = np.random.uniform(z_min, z_max, n_points)

        # set last point equal to first to have a closed trajectory
        x[n_points - 1] = x[0]
        y[n_points - 1] = y[0]
        z[n_points - 1] = z[0]

        smoothness = 0
        tck, u = interpolate.splprep([x, y, z], s=smoothness)
        u_fine = np.linspace(0, 1, n_sampling_points)
        x_function, y_function, z_function = interpolate.splev(u_fine, tck)

        return x_function, y_function, z_function

    def get_3d_spline_ur5_workspace(
        self,
        x_min,
        x_max,
        y_min,
        y_max,
        z_min,
        z_max,
        n_points=10,
        n_sampling_points=4000,
    ):
        """Generate samples of the cartesian coordinates of a 3d spline that do not cross a vertical
            cylinder of radius r_min centered in 0,0.

        Args:
            x_min (float): min x coordinate of random points used to interpolate spline (m).
            x_max (float): max x coordinate of random points used to interpolate spline (m).
            y_min (float): min y coordinate of random points used to interpolate spline (m).
            y_max (float): max y coordinate of random points used to interpolate spline (m).
            z_min (float): min z coordinate of random points used to interpolate spline (m).
            z_max (float): max z coordinate of random points used to interpolate spline (m).
            n_points (int): number of random points used to interpolate the 3d spline.
            n_sampling_points (int): number of the samples to take over the whole length of the spline.

        Returns:
            np.array: Samples of the x coordinate of the function over time.
            np.array: Samples of the y coordinate of the function over time.
            np.array: Samples of the z coordinate of the function over time.

        """

        r_min_cylinder = 0.2
        r_min_sphere_base = 0.35

        n_points = int(n_points)
        n_sampling_points = int(n_sampling_points)

        search = True
        while search:
            x = np.random.uniform(x_min, x_max, n_points)
            y = np.random.uniform(y_min, y_max, n_points)
            z = np.random.uniform(z_min, z_max, n_points)

            # set first point oustide of square of size 0.5m centered in 0,0
            x[0] = random.choice(
                [np.random.uniform(-1.0, -0.5), np.random.uniform(0.5, 1.0)]
            )
            y[0] = random.choice(
                [np.random.uniform(-1.0, -0.5), np.random.uniform(0.5, 1.0)]
            )

            # set last point equal to first to have a closed trajectory
            x[n_points - 1] = x[0]
            y[n_points - 1] = y[0]
            z[n_points - 1] = z[0]

            smoothness = 0
            tck, u = interpolate.splprep([x, y, z], s=smoothness)
            u_fine = np.linspace(0, 1, n_sampling_points)
            x_function, y_function, z_function = interpolate.splev(u_fine, tck)

            search = False
            for i in range(len(x_function)):
                if (x_function[i] ** 2 + y_function[i] ** 2) ** (
                    1 / 2
                ) <= r_min_cylinder or (
                    x_function[i] ** 2 + y_function[i] ** 2 + z_function[i] ** 2
                ) ** (
                    1 / 2
                ) <= r_min_sphere_base:
                    search = True

        return x_function, y_function, z_function

    def get_fixed_trajectory(self, trajectory_id):
        # file_name = "obstacle_trajectories.yaml"
        trajectory_name = "trajectory_" + str(int(trajectory_id))
        x_function = self.p[trajectory_name]["x"]
        y_function = self.p[trajectory_name]["y"]
        z_function = self.p[trajectory_name]["z"]

        return x_function, y_function, z_function

    def get_fixed_position_a_b(self, x_a, y_a, z_a, x_b, y_b, z_b, hold_a, hold_b):
        """Generate trajectory for object in a fixed position a for hold_a time and
            in a fixed position b for hold_b time.

        Args:
            x_a (float): x coordinate position a (m).
            y_a (float): y coordinate position a (m).
            z_a (float): z coordinate position a (m).
            x_b (float): x coordinate position b (m).
            y_b (float): y coordinate position b (m).
            z_b (float): z coordinate position b (m).
            hold_a (float): time in position a (s).
            hold_b (float): time in position b (s).

        Returns:
            list: x coordinate function
            list: y coordinate function
            list: z coordinate function
        """
        x_a_function = np.full(int(self.update_rate * hold_a), x_a)
        y_a_function = np.full(int(self.update_rate * hold_a), y_a)
        z_a_function = np.full(int(self.update_rate * hold_a), z_a)
        x_b_function = np.full(int(self.update_rate * hold_b), x_b)
        y_b_function = np.full(int(self.update_rate * hold_b), y_b)
        z_b_function = np.full(int(self.update_rate * hold_b), z_b)

        x_function = np.concatenate((x_a_function, x_b_function))
        y_function = np.concatenate((y_a_function, y_b_function))
        z_function = np.concatenate((z_a_function, z_b_function))

        return x_function, y_function, z_function

    def get_interpolated_a_b_c(
        self,
        x_a,
        y_a,
        z_a,
        x_b,
        y_b,
        z_b,
        x_c,
        y_c,
        z_c,
        hold_a,
        hold_b,
        hold_c,
        n_sampling_points_ab,
        n_sampling_points_bc,
    ):
        """Generate trajectory for object:
            - a for hold_a time
            - move from a to b
            - b for hold_c time
            - move from b to c
            - c for hold_c time

        Args:
            x_a (float): x coordinate position a (m).
            y_a (float): y coordinate position a (m).
            z_a (float): z coordinate position a (m).
            x_b (float): x coordinate position b (m).
            y_b (float): y coordinate position b (m).
            z_b (float): z coordinate position b (m).
            x_c (float): x coordinate position c (m).
            y_c (float): y coordinate position c (m).
            z_c (float): z coordinate position c (m).
            hold_a (float): time in position a (s).
            hold_b (float): time in position b (s).
            hold_c (float): time in position c (s).
            n_sampling_points_ab (int): number of the samples to take over the spline from a to b.
            n_sampling_points_bc (int): number of the samples to take over the spline from b to c.

        Returns:
            list: x coordinate function
            list: y coordinate function
            list: z coordinate function
        """
        x_a_function = np.full(int(self.update_rate * hold_a), x_a)
        y_a_function = np.full(int(self.update_rate * hold_a), y_a)
        z_a_function = np.full(int(self.update_rate * hold_a), z_a)

        tck_ab, _ = interpolate.splprep([[x_a, x_b], [y_a, y_b], [z_a, z_b]], s=0, k=1)
        u_fine_ab = np.linspace(0, 1, n_sampling_points_ab)
        x_ab_function, y_ab_function, z_ab_function = interpolate.splev(
            u_fine_ab, tck_ab
        )

        x_b_function = np.full(int(self.update_rate * hold_b), x_b)
        y_b_function = np.full(int(self.update_rate * hold_b), y_b)
        z_b_function = np.full(int(self.update_rate * hold_b), z_b)

        tck_bc, _ = interpolate.splprep([[x_b, x_c], [y_b, y_c], [z_b, z_c]], s=0, k=1)
        u_fine_bc = np.linspace(0, 1, n_sampling_points_bc)
        x_bc_function, y_bc_function, z_bc_function = interpolate.splev(
            u_fine_bc, tck_bc
        )

        x_c_function = np.full(int(self.update_rate * hold_c), x_c)
        y_c_function = np.full(int(self.update_rate * hold_c), y_c)
        z_c_function = np.full(int(self.update_rate * hold_c), z_c)

        x_function = np.concatenate(
            (x_a_function, x_ab_function, x_b_function, x_bc_function, x_c_function)
        )
        y_function = np.concatenate(
            (y_a_function, y_ab_function, y_b_function, y_bc_function, y_c_function)
        )
        z_function = np.concatenate(
            (z_a_function, z_ab_function, z_b_function, z_bc_function, z_c_function)
        )

        return x_function, y_function, z_function

    def objects_initialization(self):
        self.n_objects = int(self.get_parameter("n_objects").value)
        if not self.real_robot:
            self.objects_pose = [SetEntityPose.Request() for i in range(self.n_objects)]
            for i in range(self.n_objects):
                self.objects_pose[i].entity.name = self.get_parameter(
                    "object_" + repr(i) + "_model_name"
                ).value
        self.objects_tf_frame = [
            self.get_parameter("object_" + repr(i) + "_frame").value
            for i in range(self.n_objects)
        ]

    def move_objects_up(self):
        for i in range(self.n_objects):
            if not self.real_robot:
                self.objects_pose[i].pose.position.x = float(i)
                self.objects_pose[i].pose.position.y = 0.0
                self.objects_pose[i].pose.position.z = 3.0
                self.gazebo_set_pose.call_async(self.objects_pose[i])
            t = TransformStamped()
            t.header.frame_id = self.reference_frame
            t.header.stamp = self.get_clock().now().to_msg()
            t.child_frame_id = self.objects_tf_frame[i]
            t.transform.translation.x = float(i)
            t.transform.translation.y = 0.0
            t.transform.translation.z = 3.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.static_tf2_broadcaster.sendTransform(t)


if __name__ == "__main__":
    rclpy.init()
    oc = ObjectsController()
    try:
        rclpy.spin(oc)
    except KeyboardInterrupt:
        pass
    oc.destroy_node()
    rclpy.shutdown()
