from dynamixel_sdk import *
import time
import math

# 사용할 포트와 프로토콜 버전 설정
PORT = '/dev/ttyUSB0'
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
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, 64, 1) # 모터 회전 가능하도록 설정
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 48, MIN_ANGLE) # 모터 최소 각도 설정
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 52, MAX_ANGLE) # 모터 최대 각도 설정

# groupSyncWrite 객체 생성
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 116, 4)

# Inverse Kinematics
PI = math.pi

# Robot Geometry
BodySideLength = 98.5
CoxaLength = 60
FemurLength = 61.847
TibiaLength = 126.487

BodyCenterOffset1 = 64.84
BodyCenterOffset2 = 124.842

# Body Center Offset X
BodyCenterOffsetX_1 = BodyCenterOffset2
BodyCenterOffsetX_2 = 0
BodyCenterOffsetX_3 = -BodyCenterOffset2
BodyCenterOffsetX_4 = -BodyCenterOffset2
BodyCenterOffsetX_5 = 0
BodyCenterOffsetX_6 = BodyCenterOffset2

# Body Center Offset Y
BodyCenterOffsetY_1 = BodyCenterOffset1
BodyCenterOffsetY_2 = BodySideLength
BodyCenterOffsetY_3 = BodyCenterOffset1
BodyCenterOffsetY_4 = -BodyCenterOffset1
BodyCenterOffsetY_5 = -BodySideLength
BodyCenterOffsetY_6 = -BodyCenterOffset1

# Initial of Angles
FeetAngle_1 = -62.553
FeetAngle_2 = 0
FeetAngle_3 = 62.553
FeetAngle_4 = 180 - 62.553
FeetAngle_5 = 180
FeetAngle_6 = 180 + 62.553

IniCoxaAngle_1 = 107.553
IniCoxaAngle_2 = 90
IniCoxaAngle_3 = 72.447
IniCoxaAngle_4 = -252.447
IniCoxaAngle_5 = -270
IniCoxaAngle_6 = -387.94177183096554

# Control Input
# Leg 1
PosX_1 = BodyCenterOffsetX_1 + (CoxaLength + FemurLength) * math.sin(math.radians(45))
PosY_1 = BodyCenterOffsetY_1 + (CoxaLength + FemurLength) * math.cos(math.radians(45))
PosZ_1 = TibiaLength

# Leg 2
#PosX_2 = BodyCenterOffsetX_2 + (CoxaLength + FemurLength) * math.sin(math.radians(0)) # 0
#PosY_2 = BodyCenterOffsetY_2 + (CoxaLength + FemurLength ) * math.cos(math.radians(0)) # 220.347
#PosZ_2 = TibiaLength                                                               # 126.487

PosX_2 = 0
PosY_2 = 250
PosZ_2 = 126.487 / 2

# Leg 3
PosX_3 = BodyCenterOffsetX_3 + (CoxaLength + FemurLength) * math.sin(math.radians(-45))
PosY_3 = BodyCenterOffsetY_3 + (CoxaLength + FemurLength) * math.cos(math.radians(-45))
PosZ_3 = -TibiaLength

# Leg 4
PosX_4 = BodyCenterOffsetX_4 + (CoxaLength + FemurLength) * math.sin(math.radians(-135))
PosY_4 = BodyCenterOffsetY_4 + (CoxaLength + FemurLength) * math.cos(math.radians(-135))
PosZ_4 = -TibiaLength

# Leg 5
PosX_5 = BodyCenterOffsetX_5 + (CoxaLength + FemurLength) * math.sin(math.radians(180))
PosY_5 = BodyCenterOffsetY_5 + (CoxaLength + FemurLength) * math.cos(math.radians(180))
PosZ_5 = -TibiaLength

# Leg 6
PosX_6 = BodyCenterOffsetX_6 + (CoxaLength + FemurLength) * math.sin(math.radians(135))
PosY_6 = BodyCenterOffsetY_6 + (CoxaLength + FemurLength) * math.cos(math.radians(135))
PosZ_6 = -TibiaLength

# Angles
# Leg 1
CoxaAngle_1 = math.degrees(math.atan2((PosY_1 - BodyCenterOffsetY_1), (PosX_1 - BodyCenterOffsetX_1))) - FeetAngle_1
TibiaAngle_1 = math.degrees(math.acos(((PosX_1 - BodyCenterOffsetX_1)**2 + (PosY_1 - BodyCenterOffsetY_1)**2 + PosZ_1**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_1 - BodyCenterOffsetX_1)**2 + (PosY_1 - BodyCenterOffsetY_1)**2)) / (2 * FemurLength * TibiaLength)))
FemurAngle_1 = math.degrees(math.atan2((FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_1))) * PosZ_1 - TibiaLength * math.sin(math.radians(TibiaAngle_1)) * (math.sqrt((PosX_1 - BodyCenterOffsetX_1)**2 + (PosY_1 - BodyCenterOffsetY_1)**2) - CoxaLength), (math.sqrt((PosX_1 - BodyCenterOffsetX_1)**2 + (PosY_1 - BodyCenterOffsetY_1)**2) - CoxaLength) * (FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_1))) + PosZ_1 * TibiaLength * math.sin(math.radians(TibiaAngle_1))))
print(CoxaAngle_1)
print(FemurAngle_1)
print(TibiaAngle_1, "\n")

# Leg 2
CoxaAngle_2 = math.degrees(math.atan2((PosY_2 - BodyCenterOffsetY_2), (PosX_2 - BodyCenterOffsetX_2))) - FeetAngle_2
TibiaAngle_2 = math.degrees(math.acos(((PosX_2 - BodyCenterOffsetX_2)**2 + (PosY_2 - BodyCenterOffsetY_2)**2 + PosZ_2**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_2 - BodyCenterOffsetX_2)**2 + (PosY_2 - BodyCenterOffsetY_2)**2)) / (2 * FemurLength * TibiaLength)))
FemurAngle_2 = math.degrees(math.atan2((FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_2))) * PosZ_2 - TibiaLength * math.sin(math.radians(TibiaAngle_2)) * (math.sqrt((PosX_2 - BodyCenterOffsetX_2)**2 + (PosY_2 - BodyCenterOffsetY_2)**2) - CoxaLength), (math.sqrt((PosX_2 - BodyCenterOffsetX_2)**2 + (PosY_2 - BodyCenterOffsetY_2)**2) - CoxaLength) * (FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_2))) + PosZ_2 * TibiaLength * math.sin(math.radians(TibiaAngle_2))))
print(CoxaAngle_2)
print(FemurAngle_2)
print(TibiaAngle_2, "\n")


# Leg 3
CoxaAngle_3 = math.degrees(math.atan2((PosY_3 - BodyCenterOffsetY_3), (PosX_3 - BodyCenterOffsetX_3))) - FeetAngle_3
TibiaAngle_3 = math.degrees(math.acos(((PosX_3 - BodyCenterOffsetX_3)**2 + (PosY_3 - BodyCenterOffsetY_3)**2 + PosZ_3**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_3 - BodyCenterOffsetX_3)**2 + (PosY_3 - BodyCenterOffsetY_3)**2)) / (2 * FemurLength * TibiaLength)))
print(CoxaAngle_3)
#print(FemurAngle_3)
print(TibiaAngle_3, "\n")

# Leg 4
CoxaAngle_4 = math.degrees(math.atan2((PosY_4 - BodyCenterOffsetY_4), (PosX_4 - BodyCenterOffsetX_4))) - FeetAngle_4
FemurAngle_4 = 0
TibiaAngle_4 = math.degrees(math.acos(((PosX_4 - BodyCenterOffsetX_4)**2 + (PosY_4 - BodyCenterOffsetY_4)**2 + PosZ_4**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_4 - BodyCenterOffsetX_4)**2 + (PosY_4 - BodyCenterOffsetY_4)**2)) / (2 * FemurLength * TibiaLength)))
print(CoxaAngle_4)
print(FemurAngle_4)
print(TibiaAngle_4, "\n")

# Leg 5
CoxaAngle_5 = math.degrees(math.atan2((PosY_5 - BodyCenterOffsetY_5), (PosX_5 - BodyCenterOffsetX_5))) - FeetAngle_5
FemurAngle_5 = 0
TibiaAngle_5 = math.degrees(math.acos(((PosX_5 - BodyCenterOffsetX_5)**2 + (PosY_5 - BodyCenterOffsetY_5)**2 + PosZ_5**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_5 - BodyCenterOffsetX_5)**2 + (PosY_5 - BodyCenterOffsetY_5)**2)) / (2 * FemurLength * TibiaLength)))
print(CoxaAngle_5)
print(FemurAngle_5)
print(TibiaAngle_5, "\n")

# Leg 6
CoxaAngle_6 = math.degrees(math.atan2((PosY_6 - BodyCenterOffsetY_6), (PosX_2 - BodyCenterOffsetX_6))) - FeetAngle_6
FemurAngle_6 = 0
TibiaAngle_6 = math.degrees(math.acos(((PosX_6 - BodyCenterOffsetX_6)**2 + (PosY_6 - BodyCenterOffsetY_6)**2 + PosZ_6**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_6 - BodyCenterOffsetX_6)**2 + (PosY_6 - BodyCenterOffsetY_6)**2)) / (2 * FemurLength * TibiaLength)))
print(CoxaAngle_6)
print(FemurAngle_6)
print(TibiaAngle_6, "\n")

# DXL Output
CoxaAngle_1 = (CoxaAngle_1 - IniCoxaAngle_1) * (4096/360) + 2048
CoxaAngle_2 = round((CoxaAngle_2 - IniCoxaAngle_2) * (4096/360)) + 2048
CoxaAngle_3 = CoxaAngle_3 - IniCoxaAngle_3
CoxaAngle_4 = CoxaAngle_4 - IniCoxaAngle_4
CoxaAngle_5 = CoxaAngle_5 - IniCoxaAngle_5
CoxaAngle_6 = CoxaAngle_6 - IniCoxaAngle_6
FemurAngle_2 = -FemurAngle_2
FemurAngle_2 = round((FemurAngle_2) * (4096/360)) + 2048
TibiaAngle_2 = round((TibiaAngle_2) * (4096/360)) + 2048
print(CoxaAngle_2)
print(FemurAngle_2)
print(TibiaAngle_2)

"""
"""
dxl_goal_position = CoxaAngle_2
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(2, param_goal_position)
"""
dxl_goal_position = center + CoxaAngle_2
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(3, param_goal_position)

dxl_goal_position = center + CoxaAngle_3
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(6, param_goal_position)

dxl_goal_position = center + CoxaAngle_42048
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(5, param_goal_position)

dxl_goal_position = 0
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(4, param_goal_position)

dxl_goal_position = center + CoxaAngle_6
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(1, 150param_goal_position)
"""
dxl_comm_result = groupSyncWrite.txPacket()
time.sleep(1)
groupSyncWrite.clearParam()

dxl_goal_position = FemurAngle_2 + 150
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(12, param_goal_position)
"""
dxl_goal_position = center + FemurAngle_2
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(13, param_goal_position)

dxl_goal_position = center + FemurAngle_3
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(16, param_goal_position)

dxl_goal_position = center + FemurAngle_4
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(15, param_goal_position)

dxl_goal_position = 0
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(14, param_goal_position)

dxl_goal_position = center + FemurAngle_6
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(11, param_goal_position)
"""
dxl_comm_result = groupSyncWrite.txPacket()
time.sleep(1)
groupSyncWrite.clearParam()
"""
"""
dxl_goal_position = TibiaAngle_2 - 200
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(22, param_goal_position)
"""
dxl_goal_position = 2048 + 820
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(23, param_goal_position)

dxl_goal_position = center + TibiaAngle_3
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(26, param_goal_position)

dxl_goal_position = center + TibiaAngle_4
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(25, param_goal_position)

dxl_goal_position = 0
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(24, param_goal_position)

dxl_goal_position = center + TibiaAngle_6
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
dxl_addparam_result = groupSyncWrite.addParam(21, param_goal_position)
"""
dxl_comm_result = groupSyncWrite.txPacket()
time.sleep(1)
groupSyncWrite.clearParam()
"""
"""
# 포트 닫기
portHandler.closePort()
