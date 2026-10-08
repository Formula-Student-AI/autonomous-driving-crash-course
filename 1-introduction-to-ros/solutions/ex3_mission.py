"""Exercise 3 solution. Run directly: python3 ex3_mission.py"""
import rclpy
from rclpy.node import Node
from eufs_msgs.srv import SetMission
from std_srvs.srv import Trigger


def call_service(node, client, request):
    """Wait for a service, call it, and return the response."""
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f'Service {client.srv_name} is not available. Is the simulator running?')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = Node('mission_starter')

    mission_client = node.create_client(SetMission, '/set_mission')
    request = SetMission.Request()
    request.mission = SetMission.Request.TRACK_DRIVE
    response = call_service(node, mission_client, request)
    node.get_logger().info(f'set_mission success: {response.success}')

    go_client = node.create_client(Trigger, '/go')
    response = call_service(node, go_client, Trigger.Request())
    node.get_logger().info(f'go success: {response.success}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
