"""--------------------------------------------------------------------
COPYRIGHT 2014 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company")
for its licensed Segway RMP Robotic Platforms is intended and supplied to you,
the Company's customer, for use solely and exclusively with Stanley Innovation
products. The software is owned by the Company and/or its supplier, and is
protected under applicable copyright laws.  All rights are reserved. Any use in
violation of the foregoing restrictions may subject the user to criminal
sanctions under applicable laws, as well as to civil liability for the
breach of the terms and conditions of this license. The Company may
immediately terminate this Agreement upon your use of the software with
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use
of the software is therefore subject to the terms and conditions of the
OSI- approved open source license viewable at http://www.python.org/.
You are solely responsible for ensuring your compliance with the Python
open source license.

You shall indemnify, defend and hold the Company harmless from any claims,
demands, liabilities or expenses, including reasonable attorneys fees, incurred
by the Company as a result of any claim or proceeding against the Company
arising out of or based upon:

(i) The combination, operation or use of the software by you with any hardware,
    products, programs or data not supplied or approved in writing by the Company,
    if such claim or proceeding would have been avoided but for such combination,
    operation or use.

(ii) The modification of the software by or on behalf of you

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

 \file   robotiq_85_test

 \brief  Node for testing Robotiq 85 communication

 \Platform: Linux/ROS Foxy
--------------------------------------------------------------------"""
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from robotiq_85_msgs.msg import GripperCmd, GripperStat


class Robotiq85GripperTestClose(Node):
    def __init__(self):
        super().__init__('robotiq_85_test_close')

        self.declare_parameter('num_grippers', 1)
        self.declare_parameter('open', True)
        self.declare_parameter('close', False)

        self._num_grippers = self.get_parameter('num_grippers').get_parameter_value().integer_value
        self.open = self.get_parameter('open').get_parameter_value().bool_value
        self.close = self.get_parameter('close').get_parameter_value().bool_value

        if (self._num_grippers == 1):
            self.create_subscription(GripperStat, "/gripper/stat", self._update_gripper_stat, 10)
            self._gripper_pub = self.create_publisher(GripperCmd, '/gripper/cmd', 10)
        elif (self._num_grippers == 2):
            self.create_subscription(GripperStat, "/left_gripper/stat", self._update_gripper_stat, 10)
            self._left_gripper_pub = self.create_publisher(GripperCmd, '/left_gripper/stat', 10)
            self.create_subscription(GripperStat, "/right_gripper/stat", self._update_right_gripper_stat, 10)
            self._right_gripper_pub = self.create_publisher(GripperCmd, '/right_gripper/cmd', 10)
        else:
            self.get_logger().error("Number of grippers not supported (needs to be 1 or 2)")
            return

        self._gripper_stat = [GripperStat()] * self._num_grippers
        self._gripper_cmd = [GripperCmd()]  * self._num_grippers

        self.timer = self.create_timer(1, self._timer_callback)


    def _update_gripper_stat(self, stat):
        self._gripper_stat[0] = stat


    def _update_right_gripper_stat(self, stat):
        self._gripper_stat[1] = stat


    def _timer_callback(self):
        self.get_logger().info('Running Gripper Test')
        test_state = 0
        ready = False

        while not (ready):
            ready = True
            for i in range(self._num_grippers):
                ready &= self._gripper_stat[i].is_ready

        if (0 == test_state and (self.open or self.close)):
            self.get_logger().info('Opening/Closing Gripper')
            for i in range(self._num_grippers):
                if self.open:
                    self._gripper_cmd[i].position = 0.085
                else:
                    self._gripper_cmd[i].position = 0.0

                self._gripper_cmd[i].speed = 0.02
                self._gripper_cmd[i].force = 1.0
            test_state = 1
        elif (1 == test_state):
            success = True
            for i in range(self._num_grippers):
                if (self._gripper_stat[i].is_moving):
                    success = False
            if success:
                test_state = 2

        if (self._num_grippers == 1):
            self._gripper_pub.publish(self._gripper_cmd[0])
        elif (self._num_grippers == 2):
            self._left_gripper_pub.publish(self._gripper_cmd[0])
            self._right_gripper_pub.publish(self._gripper_cmd[1])


def main(args=None):
    rclpy.init(args=args)
    gripper_test = Robotiq85GripperTestClose()
    rclpy.spin(gripper_test)

    gripper_test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
