import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('service_client')
        self.get_logger().info("Service Client Node Start!")
        self.cli = self.create_client(SetBool, 'tutorial1')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, data):
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(True)
    minimal_client.get_logger().info("Service returns: %s" % (response.success))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()