from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    griperdriver_node = Node(package='robotiq_85_driver',
                             executable='robotiq_85_driver',
                             name='robotiq_85_driver',
                             parameters=[{"num_grippers": 1}, {"comport": "/dev/ttyUSB0"}, {"baud": "115200"}],
                             output='screen',)

    return LaunchDescription([griperdriver_node])