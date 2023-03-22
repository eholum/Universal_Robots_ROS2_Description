#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListHardwareComponents

import sys


class GetHardwareComponentStateNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.cli = self.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ListHardwareComponents.Request()
        self.future = None

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    node = GetHardwareComponentStateNode()
    name_arg = str(sys.argv[1])
    res = node.send_request()

    for c in res.component:
        if c.name == name_arg:
            print(c.type)
            print(c.class_type)
            print(c.state)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()