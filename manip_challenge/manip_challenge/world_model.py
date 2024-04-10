#!/usr/bin/env python3  
import io
import os
import json
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from riro_srvs.srv import NoneString

class WorldModel(Node):
    """ Interface for world model node """

    world = None
    world_channel = None
    obj_channel = None

    def __init__(self, allow_undeclared_parameters=True):
    
        self.declare_parameters(
            namespace='',
            parameters=[
                ("world_channel", Parameter.Type.STRING),
                ('obj_channel', Parameter.Type.STRING),
                ('world_frame', Parameter.Type.STRING),
                ("ee_frame", Parameter.Type.STRING),
                ('tool_frame', Parameter.Type.DOUBLE_ARRAY),
                ]
        )

        
    def add_obj_to_world(self, obj_name, obj_type, obj_pose_list, artag_id=None, **kwargs):
        """ Add an identified object to the world model """
        
        self.get_logger().info('Object received (name, type): {}, {}'.format(obj_name, obj_type))
        for obj in self.world:
            if obj['name'] == obj_name:
                return 'The requested object is already in WM', obj_name

        obj_dict = {'name': obj_name,
                    'type': obj_type,
                    'wordnet': None,
                    'fact': 'none',
                    'pose': obj_pose_list,
                    'artag_id': artag_id }
        for key in kwargs.keys():
            obj_dict[key] = kwargs[key]
        
            
        self.world.append(obj_dict)
        self.get_logger().info('Added a new object to WM: {}'.format(obj_dict))

        
    def rm_obj_from_world(self, obj_name):
        """ Remove the selected object from the world model """
        
        index = None
        for i, obj in enumerate(self.world):
            if obj['name'] == obj_name:
                index = i
                break
        if index is None:
            print("{} not in WM".format(obj_name))
            return
        del self.world[i]
        self.get_logger().info('Removed an object from WM: '+obj_name)

        
    #@staticmethod
    def start_world_node(self):
        """ """
        # Load params
        self.world_channel = self.get_parameter_or("world_channel", "world_model").get_parameter_value().string_value

        self.world = []

        # Pub/Sub
        # publisher for world model
        self._world_pub = self.create_publisher(String, self.world_channel, 0)
        ## self.create_subscription(String,
        ##                          self.obj_channel,
        ##                          self.add_obj_to_world, 10)
        

def main():
    """ Run the world model node """

    rclpy.init()        
    wm = WorldModel()
    wm.start_world_node()
    rclpy.spin()
    

if __name__ == '__main__':
    main()
