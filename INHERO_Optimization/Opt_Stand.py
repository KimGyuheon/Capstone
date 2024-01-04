import numpy as np
from math import pi, sin, cos, atan2, degrees, radians, sqrt
from dynamixel_sdk import *
import time
import threading

#--------------------------------Definition--------------------------------#

PORT = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

# 모터 ID와 모터의 최대. 최소 각도 설정
ID = [1, 11, 21, 2, 12, 22, 3, 13, 23, 4, 14, 24, 5, 15, 25, 6, 16, 26]
MIN_ANGLE = 0
MAX_ANGLE = 4095

# 포트 열기
portHandler = PortHandler(PORT)
portHandler.openPort()

# 포트 설정
packetHandler = PacketHandler(PROTOCOL_VERSION)
portHandler.setBaudRate(BAUDRATE)

# 모터 초기 설정
for id in ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, 10, 4) # Drive Mode - Time based
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, 11, 3) # Operating Position
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, 24, 20) # Moving Threshold

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, 64, 1) # Torque Enable
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 48, MIN_ANGLE) # Motor MIN_ANGLE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 52, MAX_ANGLE) # Motor MAX_ANGLE

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, 76, 1000) # Velocity I Gain
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, 78, 100) # Velocity P Gain
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, 80, 2000) # Position D Gain
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, 82, 0) # Position I Gain
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, 84, 640) # Position P Gain

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, 108, 150) # Profile acceleration
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, id, 112, 400) # Profile velocity

# groupSyncWrite 객체 생성
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 116, 4)

# Robot Geometry
CoxaLength = 60
FemurLength = 75
TibiaLength = 130

BodySideLength = 98.5
BodyCenterOffset1 = 64.84
BodyCenterOffset2 = 124.842

Offset = 120
Height = 205 #FemurLength + TibiaLength

# Body Center Offset X
BodyCenterOffsetX_1 = BodyCenterOffset1
BodyCenterOffsetX_2 = BodySideLength
BodyCenterOffsetX_3 = BodyCenterOffset1
BodyCenterOffsetX_4 = -BodyCenterOffset1
BodyCenterOffsetX_5 = -BodySideLength
BodyCenterOffsetX_6 = -BodyCenterOffset1

# Body Center Offset Y
BodyCenterOffsetY_1 = BodyCenterOffset2
BodyCenterOffsetY_2 = 0
BodyCenterOffsetY_3 = -BodyCenterOffset2
BodyCenterOffsetY_4 = -BodyCenterOffset2
BodyCenterOffsetY_5 = 0
BodyCenterOffsetY_6 = BodyCenterOffset2

# Angles of Feet
FeetAngle_1 = 45
FeetAngle_2 = 0
FeetAngle_3 = -45
FeetAngle_4 = -135
FeetAngle_5 = 180
FeetAngle_6 = 135

# List of Robot Geometry
BodyCenterOffsetX = [BodyCenterOffset1, BodySideLength, BodyCenterOffset1, -BodyCenterOffset1, -BodySideLength, -BodyCenterOffset1]
BodyCenterOffsetY = [BodyCenterOffset2, 0, -BodyCenterOffset2, -BodyCenterOffset2, 0, BodyCenterOffset2]
FeetAngle = [FeetAngle_1, FeetAngle_2, FeetAngle_3, FeetAngle_4, FeetAngle_5, FeetAngle_6]

# List of Input
PosX = [0, 0, 0, 0, 0, 0]
PosY = [0, 0, 0, 0, 0, 0]
PosZ = [0, 0, 0, 0, 0, 0]
    
# List of Output
CoxaAngle = [0, 0, 0, 0, 0, 0]
FemurAngle = [0, 0, 0, 0, 0, 0]
TibiaAngle = [0, 0, 0, 0, 0, 0]

#-----------------------Calculation-------------------------#

# Initial Motion
def Initial():
    
    for i in range (6):
        
        PosX[i] = BodyCenterOffsetX[i] + Offset * cos(radians(FeetAngle[i]))
        PosY[i] = BodyCenterOffsetY[i] + Offset * sin(radians(FeetAngle[i]))
        PosZ[i] = -Height

    return (PosX, PosY, PosZ)

# Inverse Kinematics
def IK(PosX, PosY, PosZ):
    
    for i in range(6):
        CoxaAngle[i] = degrees(atan2((PosY[i] - BodyCenterOffsetY[i]),
                                     (PosX[i] - BodyCenterOffsetX[i]))) - FeetAngle[i]
        TibiaAngle[i] = degrees(atan2(-sqrt(4 * FemurLength**2 * TibiaLength**2 - ((PosX[i] - BodyCenterOffsetX[i])**2 + (PosY[i] - BodyCenterOffsetY[i])**2 + PosZ[i]**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * sqrt((PosX[i] - BodyCenterOffsetX[i])**2 + (PosY[i] - BodyCenterOffsetY[i])**2))**2),
                                      (PosX[i] - BodyCenterOffsetX[i])**2 + (PosY[i] - BodyCenterOffsetY[i])**2 + PosZ[i]**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * sqrt((PosX[i] - BodyCenterOffsetX[i])**2 + (PosY[i] - BodyCenterOffsetY[i])**2)))
        FemurAngle[i] = degrees(atan2((FemurLength + TibiaLength * cos(radians(TibiaAngle[i]))) * PosZ[i] - TibiaLength * sin(radians(TibiaAngle[i])) * (sqrt((PosX[i] - BodyCenterOffsetX[i])**2 + (PosY[i] - BodyCenterOffsetY[i])**2) - CoxaLength), 
                                      (sqrt((PosX[i] - BodyCenterOffsetX[i])**2 + (PosY[i] - BodyCenterOffsetY[i])**2) - CoxaLength) * (FemurLength + TibiaLength * cos(radians(TibiaAngle[i]))) + PosZ[i] * TibiaLength * sin(radians(TibiaAngle[i]))))

    return ([CoxaAngle, FemurAngle, TibiaAngle])

# Motor Generating
def Generating(th):

    for i in range (6):

        # Coxa
        dxl_goal_position = round(th[0][i] * (4096/360)) + 2048
        
        if dxl_goal_position < 0:
            dxl_goal_position = dxl_goal_position + 4096
            
        elif dxl_goal_position > 4096:
            dxl_goal_position = dxl_goal_position - 4096
            
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(i+1, param_goal_position)

        # Femur
        dxl_goal_position = round((th[1][i]) * (4096/360)) + 2048

        if dxl_goal_position < 0:
            dxl_goal_position = dxl_goal_position + 4096

        elif dxl_goal_position > 4096:
            dxl_goal_position = dxl_goal_position - 4096

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(i+11, param_goal_position)

        # Tibia
        dxl_goal_position = round((th[2][i]) * (4096/360)) + 2048

        if dxl_goal_position < 0:
            dxl_goal_position = dxl_goal_position + 4096

        elif dxl_goal_position > 4096:
            dxl_goal_position = dxl_goal_position - 4096

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(i+21, param_goal_position)

    dxl_comm_result = groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()

# ------------------------------------ Generate ------------------------------------ #

# Initial Position(Pose)
PosX, PosY, PosZ = Initial()
th = IK(PosX, PosY, PosZ)
Generating(th)