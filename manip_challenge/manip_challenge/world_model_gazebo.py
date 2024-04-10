#!/usr/bin/env python3

import sys
import math
import numpy as np
from copy import copy, deepcopy
import threading
import json
import argparse

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import String
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose #PointStamped,
from riro_srvs.srv import StringNone, StringPose, StringString, NoneString

from manip_challenge.world_model import WorldModel
from manip_challenge import misc
import PyKDL
import collections
from bs4 import BeautifulSoup


class WorldModelPublisher(WorldModel):
    """ Gazebo Only """
    def __init__(self):
        """
        """
        super(WorldModel, self).__init__("world_model_publisher")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ("world_channel", Parameter.Type.STRING),
                ('obj_channel', Parameter.Type.STRING),
                ('world_frame', Parameter.Type.STRING),
                ("ee_frame", Parameter.Type.STRING),
                ('tool_frame', Parameter.Type.DOUBLE_ARRAY),
                ('black_list', Parameter.Type.STRING_ARRAY),
                ]
        )

        self._world_lock = threading.RLock() ## joint state lock

        self._initParams()
        self.start_world_node()
        self._initComms()        
        
        self.get_logger().info('Initialized')

    def _initParams(self):
        """ Load parameters from ROS server """
        self._world_frame_id = self.get_parameter("world_frame").get_parameter_value().string_value
        self._ee_frame_id    = self.get_parameter("ee_frame").get_parameter_value().string_value
        self._tool_offset    = self.get_parameter_or("tool_frame", Parameter('def', Parameter.Type.DOUBLE_ARRAY, [0., 0.,0.,0.,0.,0.]).value).get_parameter_value().double_array_value
        self._tool_offset = misc.list2KDLframe(self._tool_offset)

        self._black_list = self.get_parameter("black_list").get_parameter_value().string_array_value

    def _initComms(self):
        """ Initialize ROS publishers/subscribers """
        
        # subscriber
        self.create_subscription(LinkStates,\
                                 "/ros2_grasp/link_states", \
                                 self.gz_link_callback, 10)
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Services
        self.create_service(StringNone, 'remove_wm_object', \
                                self._remove_callback)
        self.create_service(StringPose, 'get_object_pose', self._object_pose_srv)
        ## self.create_service(StringPose, 'get_object_grasp_pose', self._object_grasp_pose_srv)
        ## self.create_service(StringPose, 'get_object_height', self._object_height_srv)

        
    def gz_link_callback(self, msg):
        """ Callback a Gazebo Topic and update world model """

        link_names = msg.name
        link_poses = msg.pose
        with self._world_lock:
            gz_obj_list = []
            
            for i, link_name in enumerate(link_names):
                # link_name is like "o3::o3-base"
                name = link_name.split('::')[0]

                # exclude the robot, ground, default frames
                if name == "/": continue
                if name in self._black_list: continue
                if "default" in name: continue

                # the reference frame is the world frame (=base_footprint in the begining)
                # which also equals to odom and base_link's xy in the begining
                pose = deepcopy(link_poses[i])
                pose = misc.pose2list(pose)
                pose = misc.list_quat2list_rpy(pose)

                updated_flag = False
                for j, obj in enumerate(self.world):
                    if obj['type'] == 'na': continue
                    # check if the link name is in WM
                    if obj['name'] == name:
                        self.world[j]['pose'] = pose
                        updated_flag = True                        
                        break                

                gz_obj_list.append(name)
                if updated_flag is True: continue

                obj_type = name

                ## add object to the world
                self.add_obj_to_world(name, obj_type, pose)


        # remove an object in WM if there is no matching link
        for j, obj in enumerate(self.world):
            if obj['type'] == 'na': continue
            if obj['name'] not in gz_obj_list:
                self.rm_obj_from_world(obj['name'])
            

    def _remove_callback(self, request, response):
        """ callback function: when a message arrives, return """
        if self.world is None:
            self.get_logger().info("World Model: No world model exists.")
            return response
        
        for j, obj in enumerate(self.world):
            if obj['name'] == request.data:
                del self.world[j]
                self.get_logger().info("World Model: Removed {}".format(request.data))
                break
        return response


    # -----------------------------------------------------------------------------------------
    # Services
    # -----------------------------------------------------------------------------------------
    def _object_pose_srv(self, request, response):
        p = self.get_pose_from_world(request.data)
        assert p is not False, "{} pose is not available".format(request.data)
        response.pose = misc.list2Pose(p)        
        return response  

    ## def _object_grasp_pose_srv(self, request, response):
    ##     p = self.get_pose_from_world(request.data, return_grip=True)
    ##     assert p is not False, "{} pose is not available".format(request.data)
    ##     response.pose = misc.list2Pose(p)        
    ##     return response  

    ## def _object_height_srv(self, request, response):
    ##     p = self.get_object_height(request.data)
    ##     response.pose = misc.list2Pose([0,0,p,0,0,0])        
    ##     return response

    def get_pose_from_world(self, name, return_top=True):
        """ Return a pair of position and quaternion as a list """
        
        with self._world_lock:
            
            if self.world is None: return False
            ## if not("world" in self._world_model): return False

            for i, model in enumerate(self.world):

                base_name = name

                if model["name"] == base_name:
                    if model["pose"] == (0,0,0,0,0,0,0): return False
                    frame = misc.array2KDLframe(model["pose"])

                    ## if return_top:
                    ##     height = self.get_object_height(base_name)
                    ##     offset = PyKDL(PyKDL.Vector(0,0,height))
                    ##     frame *= offset

                    return misc.KDLframe2List(frame)

            return False
    
    
    def run(self):
        self.get_logger().info("Start running world_model! ")
        start_time = self.get_clock().now()
        while rclpy.ok():
            
            with self._world_lock:
                msg = String()
                msg.data = json.dumps({'world': self.world})
                self._world_pub.publish(msg)

            # rate.sleep sleeps forever. So manually implemented..
            while rclpy.ok():
                time_now = self.get_clock().now()
                if time_now - start_time > rclpy.duration.Duration(nanoseconds=5e+8):
                    start_time = time_now
                    break
                rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None): # argv=sys.argv):
    # get options
    args = get_args(sysargv=sys.argv)[0]
    
    rclpy.init() 
    wse = WorldModelPublisher()
    wse.run()

    
def get_args(sysargv=None):
    parser = argparse.ArgumentParser(
        description="Builds the ROS2 repositories as a single batch job")
    argv = sysargv[1:] if sysargv is not None else sys.argv[1:]
    args = parser.parse_known_args(argv)

    return args

    
if __name__ == '__main__':
    main()
    
