import rclpy
import rclpy.node

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('parameter_client')
        self.get_logger().info("Tutorial-parameter Start!")

        ## Declare a ROS parameter
        self.declare_parameter('/tutorial1', 'test1')
        self.get_logger().info("Declared a ROS parameter: test1 => /tutorial1")

        ## Get a ROS parameter
        my_param = self.get_parameter('/tutorial1').get_parameter_value().string_value
        self.get_logger().info('Get a ROS parameter: /tutorial1 => %s!' % my_param)

        ## Set a ROS parameter
        my_new_param = rclpy.parameter.Parameter(
            '/tutorial1',
            rclpy.Parameter.Type.STRING,
            'test2'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)
        self.get_logger().info('Set a ROS parameter: /tutorial1 => test2!')
        self.timer = self.create_timer(1, self.timer_callback)
    
    def timer_callback(self):
        my_param = self.get_parameter('/tutorial1').get_parameter_value().string_value
        self.get_logger().info('Get a ROS parameter: /tutorial1 => %s!' % my_param)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()