import os
import numpy as np
import rclpy
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import manip_challenge.misc as misc
import time

def main():
    # Start node

    rclpy.init()
    
    node = rclpy.create_node("gazebo_object_spawner")
    
    time.sleep(5)

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    def spawn_cube(xyzrpy):
        # Set data for request
        urdf_file_path = os.path.join(
        get_package_share_directory("tutorial_2"), "cube.urdf")
        request = SpawnEntity.Request()
        request.name = "cube"
        request.xml = open(urdf_file_path, 'r').read()
        request.robot_namespace = "cube"
        request.initial_pose = misc.list2Pose(xyzrpy)

        node.get_logger().info("Sending service request to `/spawn_entity`")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            print('response: %r' % future.result())
        else:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception())

    spawn_cube([0.9,0.2,1.0, 0, 0, np.pi/4.])
    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()