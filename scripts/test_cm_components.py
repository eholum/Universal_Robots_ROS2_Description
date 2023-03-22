#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State
import sys


class SetHardwareComponentStateNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.cli = self.create_client(SetHardwareComponentState, '/controller_manager/set_hardware_component_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetHardwareComponentState.Request()
        self.name = ""
        self.target_state = State.PRIMARY_STATE_INACTIVE
        self.future = None

    def send_request(self, name, id):
        self.req.name = name
        self.req.target_state.id = id
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    node = SetHardwareComponentStateNode()

    node.send_request(str(sys.argv[1]), int(sys.argv[2]))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()