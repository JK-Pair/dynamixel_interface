#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from dynamixel_interface.dynamixel_node import *
import time

class dynamixel_subscriber(Node):

    def __init__(self):
        super().__init__('dxl_sub_node')

        #Need to set these parameters first, preparing for seperate ros protocol [serive]
        self.dxl_obj = dxl_interface()
        self.dxl_obj.openPort()
        
        ##################################CONFIGURATION START##################################
        self.dxl_obj.motorIdManagement(TARGET_DXL)
        self.dxl_obj.torqueOperation(0) #Torque on
        time.sleep(2)
        
        # self.dxl_obj.writeVelocityLimit([1023, 1023, 1023])
        self.dxl_obj.configReadingProtocol(["Present Position", "Present Load", "Present Velocity"])

        self.dxl_obj.writeOperatingMode([POS_MODE]*len(TARGET_DXL)) #VELOCITY_MODE

        # self.dxl_obj.writeVelocityLimit([50]*18)
        self.dxl_obj.writeProfileVelocity([0]*18) # 50 = 11.45 rev/min 32767

        # self.dxl_obj.writeGoalCurrent([100, 100, 100])

        
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
        self.subscription  # prevent unused variable warning force_control dynamixel_target
        
        self.prev_pos_mode = 0
        self.prev_vel_mode = 0
        self.write_mode = 0

        #self.vel_numpy = list()

    def jointState_callback(self):
        joy_state_msg = JointState()
        joy_state_msg.name = TARGET_DXL #TARGET_DXL self.dxl_obj.readConnectedID()
        read_data = self.dxl_obj.readDataFromDXL() 
        
        joy_state_msg.position = read_data[0]
        joy_state_msg.effort = read_data[1]
        joy_state_msg.velocity = read_data[2]
        
        self.publisher_.publish(joy_state_msg)
        
    def switchingMode(self, pos_activate, vel_activate):
        
        if((self.prev_pos_mode == pos_activate) and (self.prev_vel_mode == vel_activate)):
            self.write_mode = 0

        else:
            self.write_mode = 1
            self.prev_pos_mode = pos_activate
            self.prev_vel_mode = vel_activate

        #Default mode is current extended position control 
        if(pos_activate and not vel_activate and self.write_mode):

            self.dxl_obj.torqueOperation(0) #Torque off
            self.dxl_obj.writeOperatingMode([POS_MODE, POS_MODE, POS_MODE])
            self.dxl_obj.torqueOperation(1) #Torque on
            self.default_mode_status = 1

        elif(not pos_activate and vel_activate and self.write_mode):
            
            self.dxl_obj.torqueOperation(0)
            self.dxl_obj.writeOperatingMode([VELOCITY_MODE, VELOCITY_MODE, VELOCITY_MODE])
            self.dxl_obj.torqueOperation(1)
            
        elif(pos_activate and vel_activate and self.write_mode):
            self.dxl_obj.torqueOperation(0) #Torque off
            
            self.dxl_obj.writeOperatingMode([VELOCITY_MODE, VELOCITY_MODE, CURRENT_EXTEND_POS])
            self.dxl_obj.torqueOperation(1) #Torque on
            self.default_mode_status = 0
        
        else:
            pass

    def listener_callback(self, msg):
        pos_numpy = list(msg.position)
        vel_numpy = list(msg.velocity)

        pos_mode = 0
        vel_mode = 0

        if(all(vel_numpy) == 1 or (vel_numpy[-1] != 0.0)):
            vel_mode = 1
        elif(any(vel_numpy) == 1):
            pos_mode = 1
            vel_mode = 1
        else:
            pos_mode = 1

        # self.switchingMode(pos_mode, vel_mode)
        print("pos_mode: {}, vel_mode: {}".format(pos_mode, vel_mode))

        self.dxl_obj.writeGoalPosition(pos_numpy)
        self.dxl_obj.writeGoalVelocity(vel_numpy)

    def distorque(self):
        self.dxl_obj.ledOperation(1) #Torque on

def main(args=None):

    rclpy.init(args=args)

    ros2_sub_obj = dynamixel_subscriber()
    while rclpy.ok():
        rclpy.spin(ros2_sub_obj)

    ros2_sub_obj.distorque()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # ros2_sub_obj.dxl_obj.torqueOperation(0)
    ros2_sub_obj.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()