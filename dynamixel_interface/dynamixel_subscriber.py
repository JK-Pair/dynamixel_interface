#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from dynamixel_interface.dynamixel_node import *


class dynamixel_subscriber(Node):

    def __init__(self):
        super().__init__('dxl_sub_node')

        #Need to set these parameters first, preparing for seperate ros protocol [serive]
        self.dxl_obj = dxl_interface()
        self.dxl_obj.openPort()
        # self.dxl_obj.motorIdManagement(np.array([["11","12","13"], 
        #                                          ["21","22","23"], 
        #                                          ["31","32","33"], 
        #                                          ["41","42","43"], 
        #                                          ["51","52","53"], 
        #                                          ["61","62","63"]]))

        self.dxl_obj.motorIdManagement(["1","2"])

        # self.dxl_obj.motorIdManagement(["21","22","23"])
        
        self.dxl_obj.configReadingProtocol(["Present Position", "Present Load", "Present Velocity"])
        self.dxl_obj.writeOperatingMode([0x05,0x05])
        self.dxl_obj.writeGoalCurrent([54,54])
        
        # self.dxl_obj.motorIdManagement(["11","12","13", 
        #                                 "21","22","23"])

        
        self.dxl_obj.torqueOperation(1) #Torque on
        ##################################END##################################

        self.publisher_ = self.create_publisher(JointState, 'dynamixel_feedback', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.jointState_callback)

        self.subscription = self.create_subscription(
            JointState,
            'dynamixel_target',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def jointState_callback(self):
        joy_state_msg = JointState()
        # joy_state_msg.name = self.dxl_obj.readConnectedID()
        read_data = self.dxl_obj.readDataFromDXL()
        joy_state_msg.effort = read_data[1]
        # joy_state_msg.velocity = read_data[1]
        joy_state_msg.position = read_data[0]
        # joy_state_msg.position = self.dxl_obj.readPresentPosition()
        # joy_state_msg.effort = self.dxl_obj.readPresentCurrent()

        self.publisher_.publish(joy_state_msg)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: {} at {}'.format(msg.name, msg.position))
        numpyFormat = list(msg.position)

        self.dxl_obj.writeGoalPosition(numpyFormat)
        self.get_logger().info('I heard: {}'.format(msg.position))

        # self.jointState_callback()

def main(args=None):

    rclpy.init(args=args)

    ros2_sub_obj = dynamixel_subscriber()
    rclpy.spin(ros2_sub_obj)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # ros2_sub_obj.dxl_obj.torqueOperation(0)
    ros2_sub_obj.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
