#!/usr/bin/env python

import rospy
import os
import numpy as np
import sys, tty, termios
import math
from dynamixel_sdk import *
from phantomx_reactor_arm_controller.srv import UpdatePos

# Control table address
ADDR_AX_TORQUE_ENABLE      = 24
ADDR_AX_GOAL_POSITION      = 30
ADDR_AX_MOVING_SPEED       = 32
ADDR_AX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Data Byte Length
LEN_AX_GOAL_POSITION       = 2
LEN_AX_PRESENT_POSITION    = 2
LEN_AX_MOVING_SPEED        = 2

# Default setting
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller


TORQUE_ENABLE               = 1               # Value for enabling the torque
TORQUE_DISABLE              = 0               # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value
DXL_MOVING_STATUS_THRESHOLD = 20              # Dynamixel moving status threshold

NUM_MOTORS                  = 8
NUM_JOINTS                  = 6
JOINT_AXIS                  = np.array([1,1,1,1,-1,1])
JOINT_OFFSETS               = np.array([512 for i in range(NUM_JOINTS)],dtype=int)
JOINT_LIMITS                = np.array([[200,800],
                                          [  0,900],
                                          [200,800],
                                          [0,1023],
                                          [256,768],
                                          [256,768]],dtype=int)
ARM_JOINTS_POS              = [0,2,5,6,7,8]
JOINT_NAMES                 = ['elbow_pitch_joint', 'gripper_joint',
                               'shoulder_pitch_joint', 'shoulder_yaw_joint',
                               'wrist_pitch_joint', 'wrist_roll_joint']
JOINT_MOTORS                = [[5,4],[8],[2,3],[1],[6],[7]]


class armHandler:

    portHandler = None
    packetHandler = None

    def __init__(self):
        self.get_port_handler()
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

    def get_port_handler(self):
        self.portHandler = PortHandler(DEVICENAME)

        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    def close_port(self):
        self.portHandler.closePort()

def handle_update_pos(req, handler):
    check_connection(handler)
    enable_torque(handler)
    goal = get_goal_pos(req.newState.position)
    current = get_current_pose(handler)
    speeds = calculate_speeds(goal, current)

    if not set_moving_speeds(speeds, handler):
        print("Cannot reach the specified goal position")
        return 2
    if not execute_goal_pos(goal, handler):
        print("Cannot reach the specified goal position")
        return 3
    return 0

def check_connection(handler):
    isConnected = True
    for i in range(NUM_MOTORS):
        id = i+1
        status = check_motor(id, handler)
        if not status:
            isConnected = False
            print("Cannot communicate with motor id: %d"% (id))
    if not isConnected:
        print("Cannot communicate with all the motors")
        print("Cannot reach the specified goal position")
        exit(1)

def check_motor(id, handler):
    val = True
    dxl_model_number, dxl_comm_result, dxl_error = handler.packetHandler.ping(handler.portHandler, id)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % handler.packetHandler.getTxRxResult(dxl_comm_result))
        val = False
    elif dxl_error != 0:
        print("%s" % handler.packetHandler.getRxPacketError(dxl_error))
        val = False
    else:
        print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (id, dxl_model_number))

    return val

def enable_torque(handler):
    for i in range(NUM_MOTORS):
        id = i+1
        dxl_comm_result, dxl_error = handler.packetHandler.write1ByteTxRx(handler.portHandler, id, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % handler.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % handler.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % id)

def disable_torque(handler):
    for i in range(NUM_MOTORS):
        id = i+1
        dxl_comm_result, dxl_error = handler.packetHandler.write1ByteTxRx(handler.portHandler, id, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % handler.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % handler.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d torque has been successfully disabled" % id)


def get_goal_pos(position):
    goal_rad = np.array(position,dtype=float)[ARM_JOINTS_POS]
    goal_rad[1] = (goal_rad[1]-0.03)*5*np.pi/(6*0.03)
    goal = 1.2*512*goal_rad/np.pi
    goal = np.multiply(goal,JOINT_AXIS)
    goal = goal + JOINT_OFFSETS
    if goal[1] < 0: goal[1] = 0
    if goal[1] > 512: goal[1] = 512
    '''neg_i = np.where(goal<0)
    goal[neg_i] = goal[neg_i]+1229'''
    goal = np.array(goal, dtype=int)

    return goal

def get_current_pose(handler):
    pos = np.zeros((NUM_JOINTS),dtype=int)
    for i in range(NUM_JOINTS):
        id = JOINT_MOTORS[i][0]
        dxl_present_position, dxl_comm_result, dxl_error = handler.packetHandler.read2ByteTxRx(handler.portHandler, id, ADDR_AX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % handler.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % handler.packetHandler.getRxPacketError(dxl_error))

        pos[i] = dxl_present_position
        print("[ID:%03d] PresPos:%d" % (id, dxl_present_position))

    return pos

def calculate_speeds(goal, current):
    diff = np.absolute(goal - current)
    m = diff.max()
    m_speed = m/(10*0.111)
    if m_speed > 80: m_speed = 80
    if m_speed < 10: m_speed = 10
    speeds = m_speed*diff/m
    speeds = np.array(speeds,dtype=int)
    return speeds

def set_moving_speeds(speeds, handler):

    groupSyncWrite = GroupSyncWrite(handler.portHandler, handler.packetHandler, ADDR_AX_MOVING_SPEED, LEN_AX_MOVING_SPEED)
    param_moving_speeds = [0 for i in range(NUM_JOINTS)]
    # Allocate moving speed value into byte array
    for i in range(NUM_JOINTS):
        param_moving_speeds[i] = [DXL_LOBYTE(DXL_LOWORD(speeds[i])), DXL_HIBYTE(DXL_LOWORD(speeds[i]))]

    # Add Dynamixel moving speed value to the Syncwrite parameter storage
    for i in range(NUM_JOINTS):
        for j in range(len(JOINT_MOTORS[i])):
            id = JOINT_MOTORS[i][j]
            dxl_addparam_result = groupSyncWrite.addParam(id, param_moving_speeds[i])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % id)
                quit()

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % handler.packetHandler.getTxRxResult(dxl_comm_result))
    else:
        print("Moving speeds have been successfully written")

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()
    return True

def execute_goal_pos(goal, handler):
    groupSyncWrite = GroupSyncWrite(handler.portHandler, handler.packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION)
    param_goal_positions = [0 for i in range(NUM_MOTORS)]
    pos = [0 for i in range(NUM_MOTORS)]
    # Allocate moving speed value into byte array
    for i in range(NUM_JOINTS):
        for j in range(len(JOINT_MOTORS[i])):
            id = JOINT_MOTORS[i][j]
            pos[id-1] = j*1023 + (1-2*j)*goal[i]
            param_goal_positions[id-1] = [DXL_LOBYTE(DXL_LOWORD(pos[id-1])), DXL_HIBYTE(DXL_LOWORD(pos[id-1]))]

    # Add Dynamixel moving speed value to the Syncwrite parameter storage
    for i in range(NUM_MOTORS):
        id = i+1
        print("Dynamixel#%d goal pos: %d" % (id, pos[id-1]))
        dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_positions[id-1])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % id)
            quit()

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % handler.packetHandler.getTxRxResult(dxl_comm_result))
    else:
        print("Goal positions have been successfully written")
        print("Moving PhantomX Reactor Arm")

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()
    return True

def update_pos_server(handler):
    rospy.init_node('update_pos_server')
    s = rospy.Service('UpdatePos', UpdatePos, lambda req: handle_update_pos(req, handler))
    rospy.spin()

if __name__ == "__main__":

    handler = armHandler()
    update_pos_server(handler)
    handler.close_port()
