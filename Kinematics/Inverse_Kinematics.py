from dynamixel_sdk import *
import time
import math
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
# 사용할 포트와 프로토콜 버전 설정
PORT = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

# 모터 ID와 모터의 최대. 최소 각도 설정
ID = [1, 11, 21, 2, 12, 22, 3, 13, 23, 4, 14, 24, 5, 15, 25, 6, 16, 26]
MIN_ANGLE = 0
MAX_ANGLE = 4095;

index = 0

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

# Command Inputs
# p"" = control input ,r"" = control input or (zyro"" = zyrosensor미정)
PosX = 0
PosY = 0
PosZ = 0
RotX = 0
RotY = 0
RotZ = 0

#RotX = zyrox
#RotY = zyroy
#RotZ = zyroz

# Body O point to 2 % 5
BodySideLength = 98.5
# Body O point to 1 & 3 & 4 & 6
BodyCrossLength = 140.676

# Leg Definition
CoxaLength = 60
FemurLength = 61.847
TibiaLength = 126.487

BodyCenterOffset1 = BodyCrossLength*math.cos(90-62.553) # 64.84
BodyCenterOffset2 = math.sqrt(BodyCrossLength**2 - BodyCenterOffset1**2) # 124.842

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

PI = math.pi
# Initial Feet Positions
# Leg 1
FeetPosX_1 = math.cos(62.553/180*PI) * (CoxaLength + FemurLength)
FeetPosZ_1 = TibiaLength
FeetPosY_1 = math.sin(62.553/180*PI) * (CoxaLength + FemurLength)

# Leg 2
FeetPosX_2 = CoxaLength + FemurLength
FeetPosZ_2 = TibiaLength
FeetPosY_2 = 0

# Leg 3
FeetPosX_3 = math.cos(62.553/180*PI) * (CoxaLength + FemurLength)
FeetPosZ_3 = TibiaLength
FeetPosY_3 = math.sin(-62.553/180*PI) * (CoxaLength + FemurLength)

# Leg 4
FeetPosX_4 = -math.cos(62.553/180*PI) * (CoxaLength + FemurLength)
FeetPosZ_4 = TibiaLength
FeetPosY_4 = math.sin(-62.553/180*PI) * (CoxaLength + FemurLength)

# Leg 5
FeetPosX_5 = -(CoxaLength + FemurLength)
FeetPosZ_5 = TibiaLength
FeetPosY_5 = 0

# Leg 6
FeetPosX_6 = -math.cos(62.553/180*PI) * (CoxaLength + FemurLength)
FeetPosZ_6 = TibiaLength
FeetPosY_6 = math.sin(62.553/180*PI) * (CoxaLength + FemurLength)




# Body Inverse Kinematics
# Leg 1
TotalY_1 = FeetPosY_1 + BodyCenterOffsetY_1 + PosY
TotalX_1 = FeetPosX_1 + BodyCenterOffsetX_1 + PosX
DistBodyCenterFeet_1 = math.sqrt(TotalY_1**2 + TotalX_1**2)
AngleBodyCenterX_1 = PI/2 - math.atan2(TotalY_1, TotalX_1)
RollZ_1 = math.tan(RotZ * PI/180) * TotalX_1
PitchZ_1 = math.tan(RotX * PI/180) * TotalY_1
BodyIKX_1 = math.cos(AngleBodyCenterX_1 + (RotY * PI/180)) * DistBodyCenterFeet_1 - TotalX_1
BodyIKY_1 = (math.sin(AngleBodyCenterX_1 + (RotY * PI/180)) * DistBodyCenterFeet_1) - TotalY_1
BodyIKZ_1 = RollZ_1 + PitchZ_1

# Leg 2
TotalY_2 = FeetPosY_2 + BodyCenterOffsetY_2 + PosY
TotalX_2 = FeetPosX_2 + BodyCenterOffsetX_2 + PosX
DistBodyCenterFeet_2 = math.sqrt(TotalY_2**2 + TotalX_2**2)
AngleBodyCenterX_2 = PI/2 - math.atan2(TotalY_2, TotalX_2)
RollZ_2 = math.tan(RotZ * PI/180) * TotalX_2
PitchZ_2 = math.tan(RotX * PI/180) * TotalY_2
BodyIKX_2 = math.cos(AngleBodyCenterX_2 + (RotY * PI/180)) * DistBodyCenterFeet_2 - TotalX_2
BodyIKY_2 = (math.sin(AngleBodyCenterX_2 + (RotY * PI/180)) * DistBodyCenterFeet_2) - TotalY_2
BodyIKZ_2 = RollZ_2 + PitchZ_2

# Leg 3
TotalY_3 = FeetPosY_3 + BodyCenterOffsetY_3 + PosY
TotalX_3 = FeetPosX_3 + BodyCenterOffsetX_3 + PosX
DistBodyCenterFeet_3 = math.sqrt(TotalY_3**2 + TotalX_3**2)
AngleBodyCenterX_3 = PI/2 - math.atan2(TotalY_3, TotalX_3)
RollZ_3 = math.tan(RotZ * PI/180) * TotalX_3
PitchZ_3 = math.tan(RotX * PI/180) * TotalY_3
BodyIKX_3 = math.cos(AngleBodyCenterX_3 + (RotY * PI/180)) * DistBodyCenterFeet_3 - TotalX_3
BodyIKY_3 = (math.sin(AngleBodyCenterX_3 + (RotY * PI/180)) * DistBodyCenterFeet_3) - TotalY_3
BodyIKZ_3 = RollZ_3 + PitchZ_3

# Leg 4
TotalY_4 = FeetPosY_4 + BodyCenterOffsetY_4 + PosY
TotalX_4 = FeetPosX_4 + BodyCenterOffsetX_4 + PosX
DistBodyCenterFeet_4 = math.sqrt(TotalY_4**2 + TotalX_4**2)
AngleBodyCenterX_4 = PI/2 - math.atan2(TotalY_4, TotalX_4)
RollZ_4 = math.tan(RotZ * PI/180) * TotalX_4
PitchZ_4 = math.tan(RotX * PI/180) * TotalY_4
BodyIKX_4 = math.cos(AngleBodyCenterX_4 + (RotY * PI/180)) * DistBodyCenterFeet_4 - TotalX_4
BodyIKY_4 = (math.sin(AngleBodyCenterX_4 + (RotY * PI/180)) * DistBodyCenterFeet_4) - TotalY_4
BodyIKZ_4 = RollZ_4 + PitchZ_4

# Leg 5
TotalY_5 = FeetPosY_5 + BodyCenterOffsetY_5 + PosY
TotalX_5 = FeetPosX_5 + BodyCenterOffsetX_5 + PosX
DistBodyCenterFeet_5 = math.sqrt(TotalY_5**2 + TotalX_5**2)
AngleBodyCenterX_5 = PI/2 - math.atan2(TotalY_5, TotalX_5)
RollZ_5 = math.tan(RotZ * PI/180) * TotalX_5
PitchZ_5 = math.tan(RotX * PI/180) * TotalY_5
BodyIKX_5 = math.cos(AngleBodyCenterX_5 + (RotY * PI/180)) * DistBodyCenterFeet_5 - TotalX_5
BodyIKY_5 = (math.sin(AngleBodyCenterX_5 + (RotY * PI/180)) * DistBodyCenterFeet_5) - TotalY_5
BodyIKZ_5 = RollZ_5 + PitchZ_5

# Leg 6
TotalY_6 = FeetPosY_6 + BodyCenterOffsetY_6 + PosY
TotalX_6 = FeetPosX_6 + BodyCenterOffsetX_6 + PosX
DistBodyCenterFeet_6 = math.sqrt(TotalY_6**2 + TotalX_6**2)
AngleBodyCenterX_6 = PI/2 - math.atan2(TotalY_6, TotalX_6)
RollZ_6 = math.tan(RotZ * PI/180) * TotalX_6
PitchZ_6 = math.tan(RotX * PI/180) * TotalY_6
BodyIKX_6 = math.cos(AngleBodyCenterX_6 + (RotY * PI/180)) * DistBodyCenterFeet_6 - TotalX_6
BodyIKY_6 = (math.sin(AngleBodyCenterX_6 + (RotY * PI/180)) * DistBodyCenterFeet_6) - TotalY_6
BodyIKZ_6 = RollZ_6 + PitchZ_6





# Leg Inverse Kinematics
# Leg 1
NewPosX_1 = FeetPosX_1 + PosX + BodyIKX_1
NewPosZ_1 = FeetPosZ_1 + PosZ + BodyIKZ_1
NewPosY_1 = FeetPosY_1 + PosY + BodyIKY_1
CoxaFeetDist_1 =math. sqrt(NewPosX_1**2 + NewPosY_1**2)
IKSW_1 = math.sqrt((CoxaFeetDist_1 - CoxaLength)**2 + NewPosZ_1**2)
IKA1_1 = math.atan((CoxaFeetDist_1 - CoxaLength) / NewPosZ_1)
IKA2_1 = math.acos((TibiaLength**2 - FemurLength**2 - IKSW_1**2) / (-2 * IKSW_1 * FemurLength))
TAngle_1 = math.acos((IKSW_1**2 - TibiaLength**2 - FemurLength**2) / (-2 * FemurLength * TibiaLength))
IKTibiaAngle_1 = 90 - TAngle_1 * 180/PI
IKFemurAngle_1 = 90 - (IKA1_1 + IKA2_1) * 180/PI
IKCoxaAngle_1 = 90 - math.atan2(NewPosY_1, NewPosX_1) * 180/PI

# Leg 2
NewPosX_2 = FeetPosX_2 + PosX + BodyIKX_2
NewPosZ_2 = FeetPosZ_2 + PosZ + BodyIKZ_2
NewPosY_2 = FeetPosY_2 + PosY + BodyIKY_2
CoxaFeetDist_2 = math.sqrt(NewPosX_2**2 + NewPosY_2**2)
IKSW_2 = math.sqrt((CoxaFeetDist_2 - CoxaLength)**2 + NewPosZ_2**2)
IKA1_2 = math.atan((CoxaFeetDist_2 - CoxaLength) / NewPosZ_2)
IKA2_2 = math.acos((TibiaLength**2 - FemurLength**2 - IKSW_2**2) / (-2 * IKSW_2 * FemurLength))
TAngle_2 = math.acos((IKSW_2**2 - TibiaLength**2 - FemurLength**2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_2 = 90 - TAngle_2 * 180/PI
IKFemurAngle_2 = 90 - (IKA1_2 + IKA2_2) * 180/PI
IKCoxaAngle_2 = 90 - math.atan2(NewPosY_2, NewPosX_2) * 180/PI

# Leg 3
NewPosX_3 = FeetPosX_3 + PosX + BodyIKX_3
NewPosZ_3 = FeetPosZ_3 + PosZ + BodyIKZ_3
NewPosY_3 = FeetPosY_3 + PosY + BodyIKY_3
CoxaFeetDist_3 = math.sqrt(NewPosX_3**2 + NewPosY_3**2)
IKSW_3 = math.sqrt((CoxaFeetDist_3 - CoxaLength)**2 + NewPosZ_3**2)
IKA1_3 = math.atan((CoxaFeetDist_3 - CoxaLength) / NewPosZ_3)
IKA2_3 = math.acos((TibiaLength**2 - FemurLength**2 - IKSW_3**2) / (-2 * IKSW_3 * FemurLength))
TAngle_3 = math.acos((IKSW_3**2 - TibiaLength**2 - FemurLength**2) / (-2 * FemurLength * TibiaLength))
IKTibiaAngle_3 = 90 - TAngle_3 * 180/PI
IKFemurAngle_3 = 90 - (IKA1_3 + IKA2_3) * 180/PI
IKCoxaAngle_3 = 90 - math.atan2(NewPosY_3, NewPosX_3) * 180/PI

# Leg 4
NewPosX_4 = FeetPosX_4 + PosX + BodyIKX_4
NewPosZ_4 = FeetPosZ_4 + PosZ + BodyIKZ_4
NewPosY_4 = FeetPosY_4 + PosY + BodyIKY_4
CoxaFeetDist_4 = math.sqrt(NewPosX_4**2 + NewPosY_4**2)
IKSW_4 = math.sqrt((CoxaFeetDist_4 - CoxaLength)**2 + NewPosZ_4**2)
IKA1_4 = math.atan((CoxaFeetDist_4 - CoxaLength) / NewPosZ_4)
IKA2_4 = math.acos((TibiaLength**2 - FemurLength**2 - IKSW_4**2) / (-2 * IKSW_4 *  FemurLength))
TAngle_4 = math.acos((IKSW_4**2 - TibiaLength**2 - FemurLength**2) / (-2 * FemurLength * TibiaLength))
IKTibiaAngle_4 = 90 - TAngle_4 * 180/PI
IKFemurAngle_4 = 90 - (IKA1_4 + IKA2_4) * 180/PI
IKCoxaAngle_4 = 90 - math.atan2(NewPosY_4, NewPosX_4) * 180/PI

# Leg 5
NewPosX_5 = FeetPosX_5 + PosX + BodyIKX_5
NewPosZ_5 = FeetPosZ_5 + PosZ + BodyIKZ_5
NewPosY_5 = FeetPosY_5 + PosY + BodyIKY_5
CoxaFeetDist_5 = math.sqrt(NewPosX_5**2 + NewPosY_5**2)
IKSW_5 = math.sqrt((CoxaFeetDist_5 - CoxaLength)**2 + NewPosZ_5**2)
IKA1_5 = math.atan((CoxaFeetDist_5 - CoxaLength) / NewPosZ_5)
IKA2_5 = math.acos((TibiaLength**2 - FemurLength**2 - IKSW_5**2) / (-2 * IKSW_5 *  FemurLength))
TAngle_5 = math.acos((IKSW_5**2 - TibiaLength**2 - FemurLength**2) / (-2 * FemurLength * TibiaLength))
IKTibiaAngle_5 = 90 - TAngle_5 * 180/PI
IKFemurAngle_5 = 90 - (IKA1_5 + IKA2_5) * 180/PI
IKCoxaAngle_5 = 90 - math.atan2(NewPosY_5, NewPosX_5) * 180/PI

# Leg 6
NewPosX_6 = FeetPosX_6 + PosX +  BodyIKX_6
NewPosZ_6 = FeetPosZ_6 + PosZ + BodyIKZ_6
NewPosY_6 = FeetPosY_6 + PosY + BodyIKY_6
CoxaFeetDist_6 = math.sqrt(NewPosX_6**2   + NewPosY_6**2)
IKSW_6 = math.sqrt((CoxaFeetDist_6 - CoxaLength)**2 + NewPosZ_6**2)
IKA1_6 = math.atan((CoxaFeetDist_6 - CoxaLength) / NewPosZ_6)
IKA2_6 = math.acos((TibiaLength**2 - FemurLength**2 - IKSW_6**2) / (-2 * IKSW_6 *  FemurLength))
TAngle_6 = math.acos((IKSW_6**2 - TibiaLength**2 - FemurLength**2) / (-2 * FemurLength * TibiaLength))
IKTibiaAngle_6 = 90 - TAngle_6 * 180/PI
IKFemurAngle_6 = 90 - (IKA1_6 + IKA2_6) * 180/PI
IKCoxaAngle_6 = 90 - math.atan2(NewPosY_6, NewPosX_6) * 180/PI


# Dynamixel Angles
# Leg 1
CoxaAngle_1 = (IKCoxaAngle_1 - 62.553) * (4096/360)
FemurAngle_1 = IKFemurAngle_1 * (4096/360)
TibiaAngle_1 = IKTibiaAngle_1 * (4096/360)

# Leg 2
CoxaAngle_2 = IKCoxaAngle_2 * (4096/360)
FemurAngle_2 = IKFemurAngle_2 * (4096/360)
TibiaAngle_2 = IKTibiaAngle_2 * (4096/360)

# Leg 3
CoxaAngle_3 = (IKCoxaAngle_3 + 62.553) * (4096/360)
FemurAngle_3 = IKFemurAngle_3 * (4096/360)
TibiaAngle_3 = IKTibiaAngle_3 * (4096/360)

# Leg 4
CoxaAngle_4 = IKCoxaAngle_4 - (180 + 62.553) * (4096/360)
FemurAngle_4 = IKFemurAngle_4 * (4096/360)
TibiaAngle_4 = IKTibiaAngle_4 * (4096/360)

# Leg 5
CoxaAngle_5 = (IKCoxaAngle_5 - 180) * (4096/360)
FemurAngle_5 = IKFemurAngle_5 * (4096/360)
TibiaAngle_5 = IKTibiaAngle_5 * (4086/360)

# Leg 6
CoxaAngle_6 = (IKCoxaAngle_6 - 180 + 62.553) * (4096/360)
FemurAngle_6 = IKFemurAngle_6 * (4096/360)
TibiaAngle_6 = IKTibiaAngle_6 * (4096/360)



# Give Angles to Dynamixel

# Leg 1
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(CoxaAngle_1)), DXL_HIBYTE(DXL_LOWORD(CoxaAngle_1)), DXL_LOBYTE(DXL_HIWORD(CoxaAngle_1)), DXL_HIBYTE(DXL_HIWORD(CoxaAngle_1))]
dxl_addparam_result = groupSyncWrite.addParam(1, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(FemurAngle_1)), DXL_HIBYTE(DXL_LOWORD(FemurAngle_1)), DXL_LOBYTE(DXL_HIWORD(FemurAngle_1)), DXL_HIBYTE(DXL_HIWORD(FemurAngle_1))]
dxl_addparam_result = groupSyncWrite.addParam(11, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(TibiaAngle_1)), DXL_HIBYTE(DXL_LOWORD(TibiaAngle_1)), DXL_LOBYTE(DXL_HIWORD(TibiaAngle_1)), DXL_HIBYTE(DXL_HIWORD(TibiaAngle_1))]
dxl_addparam_result = groupSyncWrite.addParam(21, param_goal_position)

dxl_comm_result = groupSyncWrite.txPacket()
groupSyncWrite.clearParam()


# Leg 2
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(CoxaAngle_2)), DXL_HIBYTE(DXL_LOWORD(CoxaAngle_2)), DXL_LOBYTE(DXL_HIWORD(CoxaAngle_2)), DXL_HIBYTE(DXL_HIWORD(CoxaAngle_2))]
dxl_addparam_result = groupSyncWrite.addParam(2, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(FemurAngle_2)), DXL_HIBYTE(DXL_LOWORD(FemurAngle_2)), DXL_LOBYTE(DXL_HIWORD(FemurAngle_2)), DXL_HIBYTE(DXL_HIWORD(FemurAngle_2))]
dxl_addparam_result = groupSyncWrite.addParam(12, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(TibiaAngle_2)), DXL_HIBYTE(DXL_LOWORD(TibiaAngle_2)), DXL_LOBYTE(DXL_HIWORD(TibiaAngle_2)), DXL_HIBYTE(DXL_HIWORD(TibiaAngle_2))]
dxl_addparam_result = groupSyncWrite.addParam(22, param_goal_position)

dxl_comm_result = groupSyncWrite.txPacket()
groupSyncWrite.clearParam()


# Leg 3
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(CoxaAngle_3)), DXL_HIBYTE(DXL_LOWORD(CoxaAngle_3)), DXL_LOBYTE(DXL_HIWORD(CoxaAngle_3)), DXL_HIBYTE(DXL_HIWORD(CoxaAngle_3))]
dxl_addparam_result = groupSyncWrite.addParam(3, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(FemurAngle_3)), DXL_HIBYTE(DXL_LOWORD(FemurAngle_3)), DXL_LOBYTE(DXL_HIWORD(FemurAngle_3)), DXL_HIBYTE(DXL_HIWORD(FemurAngle_3))]
dxl_addparam_result = groupSyncWrite.addParam(13, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(TibiaAngle_3)), DXL_HIBYTE(DXL_LOWORD(TibiaAngle_3)), DXL_LOBYTE(DXL_HIWORD(TibiaAngle_3)), DXL_HIBYTE(DXL_HIWORD(TibiaAngle_3))]
dxl_addparam_result = groupSyncWrite.addParam(23, param_goal_position)

dxl_comm_result = groupSyncWrite.txPacket()
groupSyncWrite.clearParam()


# Leg 4
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(CoxaAngle_4)), DXL_HIBYTE(DXL_LOWORD(CoxaAngle_4)), DXL_LOBYTE(DXL_HIWORD(CoxaAngle_4)), DXL_HIBYTE(DXL_HIWORD(CoxaAngle_4))]
dxl_addparam_result = groupSyncWrite.addParam(4, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(FemurAngle_4)), DXL_HIBYTE(DXL_LOWORD(FemurAngle_4)), DXL_LOBYTE(DXL_HIWORD(FemurAngle_4)), DXL_HIBYTE(DXL_HIWORD(FemurAngle_4))]
dxl_addparam_result = groupSyncWrite.addParam(14, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(TibiaAngle_4)), DXL_HIBYTE(DXL_LOWORD(TibiaAngle_4)), DXL_LOBYTE(DXL_HIWORD(TibiaAngle_4)), DXL_HIBYTE(DXL_HIWORD(TibiaAngle_4))]
dxl_addparam_result = groupSyncWrite.addParam(24, param_goal_position)

dxl_comm_result = groupSyncWrite.txPacket()
groupSyncWrite.clearParam()


# Leg 5
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(CoxaAngle_5)), DXL_HIBYTE(DXL_LOWORD(CoxaAngle_5)), DXL_LOBYTE(DXL_HIWORD(CoxaAngle_5)), DXL_HIBYTE(DXL_HIWORD(CoxaAngle_5))]
dxl_addparam_result = groupSyncWrite.addParam(5, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(FemurAngle_5)), DXL_HIBYTE(DXL_LOWORD(FemurAngle_5)), DXL_LOBYTE(DXL_HIWORD(FemurAngle_5)), DXL_HIBYTE(DXL_HIWORD(FemurAngle_5))]
dxl_addparam_result = groupSyncWrite.addParam(15, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(TibiaAngle_5)), DXL_HIBYTE(DXL_LOWORD(TibiaAngle_5)), DXL_LOBYTE(DXL_HIWORD(TibiaAngle_5)), DXL_HIBYTE(DXL_HIWORD(TibiaAngle_5))]
dxl_addparam_result = groupSyncWrite.addParam(25, param_goal_position)

dxl_comm_result = groupSyncWrite.txPacket()
groupSyncWrite.clearParam()


# Leg 6
param_goal_position = [DXL_LOBYTE(DXL_LOWORD(CoxaAngle_6)), DXL_HIBYTE(DXL_LOWORD(CoxaAngle_6)), DXL_LOBYTE(DXL_HIWORD(CoxaAngle_6)), DXL_HIBYTE(DXL_HIWORD(CoxaAngle_6))]
dxl_addparam_result = groupSyncWrite.addParam(6, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(FemurAngle_6)), DXL_HIBYTE(DXL_LOWORD(FemurAngle_6)), DXL_LOBYTE(DXL_HIWORD(FemurAngle_6)), DXL_HIBYTE(DXL_HIWORD(FemurAngle_6))]
dxl_addparam_result = groupSyncWrite.addParam(16, param_goal_position)

param_goal_position = [DXL_LOBYTE(DXL_LOWORD(TibiaAngle_6)), DXL_HIBYTE(DXL_LOWORD(TibiaAngle_6)), DXL_LOBYTE(DXL_HIWORD(TibiaAngle_6)), DXL_HIBYTE(DXL_HIWORD(TibiaAngle_6))]
dxl_addparam_result = groupSyncWrite.addParam(26, param_goal_position)

dxl_comm_result = groupSyncWrite.txPacket()
groupSyncWrite.clearParam()
   
    
# 포트 닫기
portHandler.closePort()
