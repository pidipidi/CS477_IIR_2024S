from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('service_server')
        self.get_logger().info('Service Server Node Start!')
        self.srv = self.create_service(SetBool, 'tutorial1', self.set_bool_srv_callback)

    def set_bool_srv_callback(self, request, response):
        self.get_logger().info('Request: %s' % (request.data))
        if request.data == True:
            response.success = True
            response.message = 'test'
        else:
            response.success = False
            response.message = 'test'

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()