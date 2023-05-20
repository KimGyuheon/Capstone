from dynamixel_sdk import *
import time
import math # 각도 계산을 위한 것
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
PosX = px
PosY = py
PosZ = pz
RotX = rx
RotY = ry
RotZ = rz

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

math.pi = PI
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
DistBodyCenterFeet_1 = math.sqrt(TotalY_1**2, TotalX_1**2)
AngleBodyCenterX_1 = PI/2 - math.atan2(TotalY_1, TotalX_1)
RollZ_1 = math.tan(RotZ * PI/180) * TotalX_1
PitchZ_1 = math.tan(RotX * PI/180) * TotalY_1
BodyIKX_1 = math.cos(AngleBodyCenterX_1 + (RotY * PI/180)) * DistBodyCenterFeet_1 - TotalX_1
BodyIKY_1 = (math.sin(AngleBodyCenterX_1 + (RotY * PI/180)) * DistBodyCenterFeet_1) - TotalY_1
BodyIKZ_1 = RollZ_1 + PitchZ_1

# Leg 2
TotalY_2 = FeetPosY_2 + BodyCenterOffsetY_2 + PosY
TotalX_2 = FeetPosX_2 + BodyCenterOffsetX_2 + PosX
DistBodyCenterFeet_2 = math.sqrt(TotalY_2**2, TotalX_2**2)
AngleBodyCenterX_2 = PI/2 - math.atan2(TotalY_2, TotalX_2)
RollZ_2 = math.tan(RotZ * PI/180) * TotalX_2
PitchZ_2 = math.tan(RotX * PI/180) * TotalY_2
BodyIKX_2 = math.cos(AngleBodyCenterX_2 + (RotY * PI/180)) * DistBodyCenterFeet_2 - TotalX_2
BodyIKY_2 = (math.sin(AngleBodyCenterX_2 + (RotY * PI/180)) * DistBodyCenterFeet_2) - TotalY_2
BodyIKZ_2 = RollZ_2 + PitchZ_2

# Leg 3
TotalY_3 = FeetPosY_3 + BodyCenterOffsetY_3 + PosY
TotalX_3 = FeetPosX_3 + BodyCenterOffsetX_3 + PosX
DistBodyCenterFeet_3 = math.sqrt(TotalY_1**2, TotalX_1**2)
AngleBodyCenterX_3 = PI/2 - math.atan2(TotalY_3, TotalX_3)
RollZ_3 = math.tan(RotZ * PI/180) * TotalX_3
PitchZ_3 = math.tan(RotX * PI/180) * TotalY_3
BodyIKX_3 = math.cos(AngleBodyCenterX_3 + (RotY * PI/180)) * DistBodyCenterFeet_3 - TotalX_3
BodyIKY_3 = (math.sin(AngleBodyCenterX_3 + (RotY * PI/180)) * DistBodyCenterFeet_3) - TotalY_3
BodyIKZ_3 = RollZ_3 + PitchZ_3

# Leg 4
TotalY_4 = FeetPosY_4 + BodyCenterOffsetY_4 + PosY
TotalX_4 = FeetPosX_4 + BodyCenterOffsetX_4 + PosX
DistBodyCenterFeet_4 = math.sqrt(TotalY_4**2, TotalX_4**2)
AngleBodyCenterX_4 = PI/2 - math.atan2(TotalY_4, TotalX_4)
RollZ_4 = math.tan(RotZ * PI/180) * TotalX_4
PitchZ_4 = math.tan(RotX * PI/180) * TotalY_4
BodyIKX_4 = math.cos(AngleBodyCenterX_4 + (RotY * PI/180)) * DistBodyCenterFeet_4 - TotalX_4
BodyIKY_4 = (math.sin(AngleBodyCenterX_4 + (RotY * PI/180)) * DistBodyCenterFeet_4) - TotalY_4
BodyIKZ_4 = RollZ_4 + PitchZ_4

# Leg 5
TotalY_5 = FeetPosY_5 + BodyCenterOffsetY_5 + PosY
TotalX_5 = FeetPosX_5 + BodyCenterOffsetX_5 + PosX
DistBodyCenterFeet_5 = math.sqrt(TotalY_5**2, TotalX_5**2)
AngleBodyCenterX_5 = PI/2 - math.atan2(TotalY_5, TotalX_5)
RollZ_5 = math.tan(RotZ * PI/180) * TotalX_5
PitchZ_5 = math.tan(RotX * PI/180) * TotalY_5
BodyIKX_5 = math.cos(AngleBodyCenterX_5 + (RotY * PI/180)) * DistBodyCenterFeet_5 - TotalX_5
BodyIKY_5 = (math.sin(AngleBodyCenterX_5 + (RotY * PI/180)) * DistBodyCenterFeet_5) - TotalY_5
BodyIKZ_5 = RollZ_5 + PitchZ_5

# Leg 6
TotalY_6 = FeetPosY_6 + BodyCenterOffsetY_6 + PosY
TotalX_6 = FeetPosX_6 + BodyCenterOffsetX_6 + PosX
DistBodyCenterFeet_6 = math.sqrt(TotalY_6**2, TotalX_6**2)
AngleBodyCenterX_6 = PI/2 - math.atan2(TotalY_6, TotalX_6)
RollZ_6 = math.tan(RotZ * PI/180) * TotalX_6
PitchZ_6 = math.tan(RotX * PI/180) * TotalY_6
BodyIKX_6 = math.cos(AngleBodyCenterX_6 + (RotY * PI/180)) * DistBodyCenterFeet_6 - TotalX_6
BodyIKY_6 = (math.sin(AngleBodyCenterX_6 + (RotY * PI/180)) * DistBodyCenterFeet_6) - TotalY_6
BodyIKZ_6 = RollZ_6 + PitchZ_6




# Without Inverse Kinematics
center_data = 2048

dxl_goal_position = center_data

dxl_goal_position_L = center_data
dxl_goal_position_R = center_data


# 10번대 다리 초기위치와 동일하게
dxl_goal_position_ten_L_U = center_data - 512
dxl_goal_position_ten_R_U = center_data + 512

dxl_goal_position_ten_L_D = center_data - 512
dxl_goal_position_ten_R_D = center_data + 512



# General Posture
# Nothing to Press(Default)
# Push 1
if getch() == chr(49):

    # odd Leg

    # leg 1
    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(1, param_goal_position)

    dxl_goal_position = center_data - 512
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(11, param_goal_position)

    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(21, param_goal_position)

    dxl_comm_result = groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()

    # leg 3
    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(3, param_goal_position)

    dxl_goal_position = center_data + 512
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(13, param_goal_position)

    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(23, param_goal_position)

    dxl_comm_result = groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()

    # leg 5
    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(5, param_goal_position)

    dxl_goal_position = center_data - 512
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(15, param_goal_position)

    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(25, param_goal_position)

    dxl_comm_result = groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()

#    time.sleep(0.5)


    # even Leg

    # leg 2
    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(2, param_goal_position)

    dxl_goal_position = center_data + 512
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(12, param_goal_position)

    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(22, param_goal_position)

    dxl_comm_result = groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()


    # leg 4
    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(4, param_goal_position)

    dxl_goal_position = center_data - 512
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(14, param_goal_position)

    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(24, param_goal_position)

    dxl_comm_result = groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()

    # leg 6
    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(6, param_goal_position)

    dxl_goal_position = center_data + 512
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(16, param_goal_position)

    dxl_goal_position = center_data
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(26, param_goal_position)

    dxl_comm_result = groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()







#while True:
#    print("start")
#    dxl_goal_position = [0, 2048, 4095] # 순차적으로 goal position 접근

#    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), #DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), #DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]

#    for id in ID:
#        dxl_addparam_result = groupSyncWrite.addParam(15, param_goal_position) # 모터 각도 추가
#        dxl_addparam_result = groupSyncWrite.addParam(25, param_goal_position)
#    dxl_comm_result = groupSyncWrite.txPacket() # 모터 각도 한 번에 설정
#    groupSyncWrite.clearParam # 추가한 모터 각도 제거
#    index = index++1
#    print(index)
#    time.sleep(2) # 2초간 대기
   
    
# 포트 닫기
portHandler.closePort()
