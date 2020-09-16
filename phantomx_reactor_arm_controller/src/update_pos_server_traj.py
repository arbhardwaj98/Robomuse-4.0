#!/usr/bin/env python

import rospy
import os
import numpy as np
import sys, tty, termios
import math
from dynamixel_sdk import *
from moveit_msgs.msg import DisplayTrajectory
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

NUM_MOTORS                  = 7
NUM_JOINTS                  = 5
JOINT_AXIS                  = np.array([1,1,1,-1,1])
JOINT_OFFSETS               = np.array([512 for i in range(NUM_JOINTS)],dtype=int)
JOINT_LIMITS                = np.array([[200,800],
                                          [200,800],
                                          [0,1023],
                                          [256,768],
                                          [256,768]],dtype=int)
ARM_JOINTS_POS              = [2,1,0,3,4]
JOINT_NAMES                 = ['elbow_pitch_joint',
                               'shoulder_pitch_joint', 'shoulder_yaw_joint',
                               'wrist_pitch_joint', 'wrist_roll_joint']
JOINT_MOTORS                = [[5,4],[2,3],[1],[6],[7]]


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
    traj = read_traj()
    points = traj.trajectory[0].joint_trajectory.points
    '''if req.data.position[0] == 0: gripper_open(req.data.position[0], handler)
    if req.data.position[0] == 1: gripper_open(req.data.position[0], handler)'''
    a = 1
    if a==1:
        for i in range(1,len(points)):
            goal = get_goal_pos(points[i])
            current = get_current_pose(points[i-1])
            move_time = get_move_time(points[i],points[i-1])
            speeds = calculate_speeds(goal, current, move_time)

            if not set_moving_speeds(speeds, handler):
                print("Cannot reach the specified goal position")
                return 2
            if not execute_goal_pos(goal, handler):
                print("Cannot reach the specified goal position")
                return 3
            wait(move_time-0.02)
    return 0

def wait(sec):
  c = time.time()
  while(True):
  	if time.time()>c+sec:
  		break
def read_traj():
    traj = rospy.wait_for_message("move_group/display_planned_path", DisplayTrajectory)
    return traj

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


def get_goal_pos(point):
    goal_rad = np.array(point.positions,dtype=float)[ARM_JOINTS_POS]
    goal = 1.2*512*goal_rad/np.pi
    goal = np.multiply(goal,JOINT_AXIS)
    goal = goal + JOINT_OFFSETS
    neg_i = np.where(goal<0)
    goal[neg_i] = goal[neg_i]+1229
    goal = np.array(goal, dtype=int)

    return goal

def get_current_pose(point):
    pos_rad = np.array(point.positions,dtype=float)[ARM_JOINTS_POS]
    pos = 1.2*512*pos_rad/np.pi
    pos = np.multiply(pos,JOINT_AXIS)
    pos = pos + JOINT_OFFSETS
    neg_i = np.where(pos<0)
    pos[neg_i] = pos[neg_i]+1229
    pos = np.array(pos, dtype=int)

    return pos

def get_move_time(goal, current):
    move_time = goal.time_from_start.to_sec()-current.time_from_start.to_sec()
    return move_time

def calculate_speeds(goal, current, move_time):
    diff = np.absolute(goal - current)
    speeds = diff/(move_time*0.111)
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

def gripper_open(data,handler):
    groupSyncWrite = GroupSyncWrite(handler.portHandler, handler.packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION)
    param_goal_positions = 0
    if data==0: pos = 100
    if data==1: pos = 700
    param_goal_positions = [DXL_LOBYTE(DXL_LOWORD(pos)), DXL_HIBYTE(DXL_LOWORD(pos))]

    id = 8
    print("Dynamixel#%d goal pos: %d" % (id, pos))
    dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_positions)
    if dxl_addparam_result != True:
        print("[ID:8] groupSyncWrite addparam failed")
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
