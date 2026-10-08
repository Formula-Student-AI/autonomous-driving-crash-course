"""Exercise 3: start the car from code.

Call the /set_mission and /go services, replacing the two `ros2 service call`
commands from Module 0.
"""
import rclpy
from rclpy.node import Node
from eufs_msgs.srv import SetMission
from std_srvs.srv import Trigger


def call_service(node, client, request):
    """Wait for a service, call it, and return the response."""
    # TODO 1: wait up to 5 seconds for the service with client.wait_for_service(timeout_sec=5.0).
    #         If it is not available, raise RuntimeError with a helpful message.

    # TODO 2: send the request asynchronously with client.call_async(request),
    #         wait for it with rclpy.spin_until_future_complete(node, future),
    #         and return future.result().
    raise NotImplementedError


def main(args=None):
    rclpy.init(args=args)
    node = Node('mission_starter')

    # TODO 3: create a client for SetMission on '/set_mission'.
    #         Hint: node.create_client(srv_type, name)

    # TODO 4: build a SetMission.Request, set request.mission to
    #         SetMission.Request.TRACK_DRIVE, call the service, and log response.success.

    # TODO 5: do the same for Trigger on '/go' (Trigger.Request() has no fields to set).

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
