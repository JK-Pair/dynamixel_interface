#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from dynamixel_interface.dynamixel_node import *

class dynamixel_publisher(Node):
    def __init__(self):
        super().__init__('dxl_pub_node')
        self.publisher_ = self.create_publisher(JointState, 'dynamixel_target', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.jointState_callback)

    def jointState_callback(self):
        targetDistance = [-10, -20]
        joy_state_msg = JointState()
        # joy_state_msg.name = ["11", "12", "13", "21", "22", "23"]

        joy_state_msg.position = [self.distanceCalculation(x) for x in targetDistance]

        self.publisher_.publish(joy_state_msg)
        self.get_logger().info('Publishing: {} , Positions: {}'.format(joy_state_msg.name,joy_state_msg.position))
        
        
    def distanceCalculation(self, targetDistance):
        '''
        targetDistance has a mm unit.

        '''

        rawDXL = (targetDistance / 8) * 4095 #1 rev = 8 mm, change it when using differnet structure.
        return float(rawDXL)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = dynamixel_publisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
