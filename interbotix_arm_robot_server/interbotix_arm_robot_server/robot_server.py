import grpc
import rclpy
from rclpy.node import Node
from concurrent import futures

from interbotix_arm_robot_server.ros_bridge import InterbotixArmRosBridge
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc


class RobotServerServicer(robot_server_pb2_grpc.RobotServerServicer):
    def __init__(self, node: Node):
        self.node = node
        self.rosbridge = InterbotixArmRosBridge(node)

    def GetState(self, request, context):
        try:
            return self.rosbridge.get_state()
        except Exception as e:
            self.node.get_logger().error(f'Failed to get state: {e}', exc_info=True)
            return robot_server_pb2.State(success=0)

    def SetState(self, request, context):
        try:
            self.rosbridge.set_state(state_msg=request)
            return robot_server_pb2.Success(success=1)
        except Exception as e:
            self.node.get_logger().error(f'Failed to set state: {e}', exc_info=True)
            return robot_server_pb2.Success(success=0)

    def SendAction(self, request, context):
        try:
            self.rosbridge.send_action(request.action)
            return robot_server_pb2.Success(success=1)
        except Exception as e:
            self.node.get_logger().error(f'Failed to send action: {e}', exc_info=True)
            return robot_server_pb2.Success(success=0)

    def SendActionGetState(self, request, context):
        try:
            self.rosbridge.send_action(request.action)
            return self.rosbridge.get_state()
        except Exception as e:
            self.node.get_logger().error(f'Failed to send action and get state: {e}', exc_info=True)
            return robot_server_pb2.State(success=0)


def serve():
    rclpy.init()
    node = rclpy.create_node('robot_server')

    wait_time = 5.0
    node.get_logger().info(f'Waiting {wait_time}s before starting initialization of Robot Server...')
    node.create_timer(wait_time, lambda: None)  # Just to simulate wait time

    server_port = node.declare_parameter('server_port', 50051).value
    real_robot = node.declare_parameter('real_robot', False).value
    robot_model = node.declare_parameter('robot_model', 'locobot').value

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_server_pb2_grpc.add_RobotServerServicer_to_server(
        RobotServerServicer(node, real_robot=real_robot, robot_model=robot_model),
        server
    )
    server.add_insecure_port(f'[::]:{server_port}')
    server.start()

    if real_robot:
        node.get_logger().info(f"{robot_model} Real Robot Server started at {server_port}")
    else:
        node.get_logger().info(f"{robot_model} Sim Robot Server started at {server_port}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down robot server...')
    finally:
        server.stop(0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    serve()
