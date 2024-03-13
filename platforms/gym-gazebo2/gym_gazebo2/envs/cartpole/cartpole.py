import gym
import gym_gazebo2
gym.logger.set_level(40) # hide warnings
import time
import numpy as np
import copy
import math
import os
import psutil
import signal
import sys
import xacro
import rclpy
from ros2pkg.api import get_prefix_path
from ament_index_python.packages import get_package_prefix

from gym import utils, spaces
from gym.utils import seeding
from gym_gazebo2.utils import ut_generic, ut_launch, ut_mara, ut_math, ut_gazebo
from gazebo_msgs.srv import SpawnEntity

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler


def getArgsParser():
    import argparse
    parser = argparse.ArgumentParser(description='Environment argument provider.')
    parser.add_argument('-g', '--gzclient', action='store_true', help='Run user interface.')
    parser.add_argument('-r', '--realSpeed', action='store_true', help='Execute the simulation in \
        real speed and using the running specific driver.')
    parser.add_argument('-v', '--velocity', type=float, default=0.5, help='Set servo motor \
        velocity. Keep < 1.57 for real speed. Applies only with -r --realSpeed option.')

    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument('-m', '--multiInstance', action='store_true', help='Provide network \
        segmentation to allow multiple instances.')
    group.add_argument('-p', '--port', type=int, default=11345, help='Provide exact port to the \
        network segmentation to allow multiple instances.')

    return parser

def generateLaunchDescription(gzclient, realSpeed, multiInstance, port, xacro_file):
    """
        Returns ROS2 LaunchDescription object.
        Args:
            realSpeed: bool   True if RTF must be set to 1, False if RTF must be set to maximum.
    """

    # Generate ROBOT_DESCRIPTION for UR5 ROBOT:
    robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description','-entity', 'cartpole_gazebo', '-z -0.1'],
                        emulate_tty=True,
                        output='screen')

    load_cartpole_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'cartpole_controller'],
        output='screen'
    )
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )


    if port != 11345: # Default gazebo port
        os.environ["ROS_DOMAIN_ID"] = str(port)
        os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + str(port)
        print("******* Manual network segmentation *******")
        print("ROS_DOMAIN_ID=" + os.environ['ROS_DOMAIN_ID'])
        print("GAZEBO_MASTER_URI=" + os.environ['GAZEBO_MASTER_URI'])
        print("")
    elif multiInstance:
        # Exclusive network segmentation, which allows to launch multiple instances of ROS2+Gazebo
        networkParams = getExclusiveNetworkParameters()
        os.environ["ROS_DOMAIN_ID"] = networkParams.get('ros_domain_id')
        os.environ["GAZEBO_MASTER_URI"] = networkParams.get('gazebo_master_uri')
        print("******* Exclusive network segmentation *******")
        print("ROS_DOMAIN_ID=" + networkParams.get('ros_domain_id'))
        print("GAZEBO_MASTER_URI=" + networkParams.get('gazebo_master_uri'))
        print("")

    try:
        envs = {}
        for key in os.environ.__dict__["_data"]:
            key = key.decode("utf-8")
            if key.isupper():
                envs[key] = os.environ[key]
    except BaseException as exception:
        print("Error with Envs: " + str(exception))
        return None

    
    # Gazebo visual interfaze. GUI/no GUI options.
    if gzclient:
        gazeboCmd = "gazebo"
    else:
        gazeboCmd = "gzserver"

    # Creation of ROS2 LaunchDescription obj.

    if realSpeed:
        worldPath = os.path.join(os.path.dirname(gym_gazebo2.__file__), 'worlds',
                                 'empty.world')
    else:
        worldPath = os.path.join(os.path.dirname(gym_gazebo2.__file__), 'worlds',
                                 'empty_speed_up.world')

    launchDesc = LaunchDescription([
        ExecuteProcess(
            cmd=[gazeboCmd, '-s', 'libgazebo_ros_factory.so', '-s',
                 'libgazebo_ros_init.so', worldPath], output='screen', env=envs),
        node_robot_state_publisher,
        load_cartpole_controller,
        load_joint_state_controller,
        spawn_entity,
    ])
    return launchDesc





class GazeboCartPoleEnv(gym.Env):
    """
    TODO. Define the environment.
    """

    def __init__(self):
        """
        Initialize the CartPole environemnt
        """
        # Manage command line args
        args = getArgsParser().parse_args()
        self.gzclient = args.gzclient
        self.realSpeed = args.realSpeed
        self.velocity = args.velocity
        self.multiInstance = args.multiInstance
        self.port = args.port

        # Set the path of the corresponding URDF file
        if self.realSpeed:
            xacro_file = get_prefix_path("cartpole_gazebo") + "/share/cartpole_gazebo/urdf/cartpole_gazebo.urdf.xacro"
        else:
            xacro_file = get_prefix_path("cartpole_gazebo") + "/share/cartpole_gazebo/urdf/cartpole_gazebo.urdf.xacro"

        # Launch cartpole in a new Process
        self.launch_subp = ut_launch.startLaunchServiceProcess(
            generateLaunchDescription(
                self.gzclient, self.realSpeed, self.multiInstance, self.port, xacro_file))

        # Create the node after the new ROS_DOMAIN_ID is set in generate_launch_description()
        rclpy.init()
        self.node = rclpy.create_node(self.__class__.__name__)

        # class variables
        self._observation_msg = None
        self.max_episode_steps = 1024 #default value, can be updated from baselines
        self.iterator = 0
        self.reset_jnts = True
        self.steps_beyond_done = None

        #############################
        #   Environment hyperparams
        #############################
        # # Topics for the robot publisher and subscriber.
        COMMAND_PUBLISHER = '/cartpole_controller/commands'
        STATE_SUBSCRIBER = '/joint_states'

        # Angle at which to fail the episode
        self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.x_threshold = 15        
        #############################

        # Subscribe to the appropriate topics, taking into account the particular robot
        self._pub = self.node.create_publisher(Float64MultiArray, COMMAND_PUBLISHER, qos_profile=qos_profile_sensor_data)
        self._sub = self.node.create_subscription(JointState, STATE_SUBSCRIBER, self.observation_callback, qos_profile=qos_profile_sensor_data)

        self.unpause = self.node.create_client(Empty, '/unpause_physics')
        self.pause   = self.node.create_client(Empty, '/pause_physics') 
        self.reset_sim = self.node.create_client(Empty, '/reset_simulation')
        self.set_model = self.node.create_client(SetModelConfiguration, '/gazebo/set_model_configuration')


        high = np.array([
            self.x_threshold * 2,
            np.finfo(np.float32).max,
            self.theta_threshold_radians * 2,
            np.finfo(np.float32).max])
        
        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(-high, high)

        # Seed the environment
        self.seed()


    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointState
        """
        self._observation_msg = message
        
    def set_episode_size(self, episode_size):
        self.max_episode_steps = episode_size

    def take_observation(self):
        """
        Take observation from the environment and return it.
        :return: state.
        """
        # Take an observation
        rclpy.spin_once(self.node)
        obs_message = self._observation_msg

        # Check that the observation is not prior to the action
        while rclpy.ok() and (obs_message is None): # or int(str(self._observation_msg.header.stamp.sec)+(str(self._observation_msg.header.stamp.nanosec))) < self.ros_clock):
            # print("I am in obs_message is none")
            rclpy.spin_once(self.node)
            obs_message = self._observation_msg

        #Set observation to None after it has been read.
        self._observation_msg = None
        
        # Concatenate the information that defines the robot state
        angle = math.atan(math.tan(obs_message.position[0]))
        state = [obs_message.position[1], obs_message.velocity[1], angle, obs_message.velocity[0]]        
        
        return state
        
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]


    def step(self, action):
        """
        Implement the environment step abstraction. Execute action and returns:
            - action
            - observation
            - reward
            - done (status)
        """
        ## # pause simulation
        ## while not self.unpause.wait_for_service(timeout_sec=1.0):
        ##     self.node.get_logger().info('/unpause_physics service not available, waiting again...')

        ## reset_future = self.unpause.call_async(Empty.Request())
        ## rclpy.spin_until_future_complete(self.node, reset_future)
        
        
        self.iterator+=1
        
        # Execute "action"
        action_msg = Float64MultiArray()        
        action_msg.data = [float(action)] 
        self._pub.publish(action_msg)

        self.ros_clock = rclpy.clock.Clock().now().nanoseconds

        # Take an observation
        state = self.take_observation()
        x, x_dot, theta, theta_dot = state
        
        done =  x < -self.x_threshold \
                or x > self.x_threshold \
                or theta < -self.theta_threshold_radians \
                or theta > self.theta_threshold_radians
        done = bool(done)
        
        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            # Pole just fall!
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                self.node.get_logger().warning("You are calling 'step()' even though this environment has already returned done = True. You should always call 'reset()' once you receive 'done = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0

        # Calculate if the env has been solved
        info = {}
        
        ## # pause simulation to make observation
        ## while not self.pause.wait_for_service(timeout_sec=1.0):
        ##     self.node.get_logger().info('/pause_physics service not available, waiting again...')

        ## reset_future = self.pause.call_async(Empty.Request())
        ## rclpy.spin_until_future_complete(self.node, reset_future)

        
        # Return the corresponding state, rewards, etc.
        return state, reward, done, info
    

    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """
        self.iterator = 0
        #self.node.get_logger().info('----------------Reset --------------------------')

        if self.reset_jnts is True:
            # pause simulation
            while not self.pause.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/pause_physics service not available, waiting again...')

            reset_future = self.pause.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)

            action_msg = Float64MultiArray()        
            action_msg.data = [float(0.0)] 
            self._pub.publish(action_msg)
            self._pub.publish(action_msg)
            
            # reset simulation
            ## while not self.reset_sim.wait_for_service(timeout_sec=1.0):
            ##     self.node.get_logger().info('/reset_simulation service not available, waiting again...')

            ## reset_future = self.reset_sim.call_async(Empty.Request())
            ## rclpy.spin_until_future_complete(self.node, reset_future)

            ## while not self.set_model.wait_for_service(timeout_sec=1.0):
            ##     self.node.get_logger().info('/gazebo/set_model_configuration service not available, waiting again ...')

            angle = np.random.uniform(-0.4, 0.4)
            req = SetModelConfiguration.Request()
            req.model_name="cartpole_gazebo"
            req.urdf_param_name=""
            req.joint_names=["cart_to_pole", "slider_to_cart"]
            req.joint_positions=[float(angle), 0.0]
                                                 
            reset_future = self.set_model.call_async(req)
            rclpy.spin_until_future_complete(self.node, reset_future)

            # unpause simulation
            while not self.unpause.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/unpause_physics service not available, waiting again...')

            reset_future = self.unpause.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)
            
            

        self.ros_clock = rclpy.clock.Clock().now().nanoseconds

                
        # Take an observation
        state = self.take_observation()
        
        self.steps_beyond_done = None
        
        # Return the corresponding state
        return state

    def close(self):
        print("Closing " + self.__class__.__name__ + " environment.")
        self.node.destroy_node()
        parent = psutil.Process(self.launch_subp.pid)
        for child in parent.children(recursive=True):
            child.kill()
        rclpy.shutdown()
        parent.kill()
    


        
