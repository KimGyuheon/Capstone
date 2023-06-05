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

# <Inverse Kinematics>
PI = math.pi

# Robot Geometry
CoxaLength = 60
FemurLength = 61.847
TibiaLength = 126.487

BodySideLength = 98.5
BodyCenterOffset1 = 64.84
BodyCenterOffset2 = 124.842

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

# Angles of Bracket
FemurBracketAngle = 14.036
TibiaBracketAngle = 32.079 - FemurBracketAngle

def Generating():
    # Femur Up
    def FemurUp_135():
        dxl_goal_position = 3500
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(11, param_goal_position)

        dxl_goal_position = 3500
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(13, param_goal_position)

        dxl_goal_position = 3500
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(15, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        time.sleep(0.1)
        groupSyncWrite.clearParam()

    def FemurUp_246():
        dxl_goal_position = 3500
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(12, param_goal_position)

        dxl_goal_position = 3500
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(14, param_goal_position)

        dxl_goal_position = 3500
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(16, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        time.sleep(0.1)
        groupSyncWrite.clearParam()

    # Step Index
    i = 0

    while 1:
        # Control Input
        if i == 0: # 1 STEP
            FemurUp_135()

            # Leg 1
            PosX_1 = BodyCenterOffsetX_1 + 150 * math.cos(math.radians(FeetAngle_1))
            PosY_1 = BodyCenterOffsetY_1 + 150 * math.sin(math.radians(FeetAngle_1)) + 50
            PosZ_1 = -TibiaLength * 2 / 3

            # Leg 2
            PosX_2 = BodyCenterOffsetX_2 + 150 * math.cos(math.radians(FeetAngle_2))
            PosY_2 = BodyCenterOffsetY_2 + 150 * math.sin(math.radians(FeetAngle_2)) - 50
            PosZ_2 = -TibiaLength * 2 / 3

            # Leg 3
            PosX_3 = BodyCenterOffsetX_3 + 150 * math.cos(math.radians(FeetAngle_3))
            PosY_3 = BodyCenterOffsetY_3 + 150 * math.sin(math.radians(FeetAngle_3)) + 50
            PosZ_3 = -TibiaLength * 2 / 3

            # Leg 4
            PosX_4 = BodyCenterOffsetX_4 + 150 * math.cos(math.radians(FeetAngle_4))
            PosY_4 = BodyCenterOffsetY_4 + 150 * math.sin(math.radians(FeetAngle_4)) - 50
            PosZ_4 = -TibiaLength * 2 / 3

            # Leg 5
            PosX_5 = BodyCenterOffsetX_5 + 150 * math.cos(math.radians(FeetAngle_5))
            PosY_5 = BodyCenterOffsetY_5 + 150 * math.sin(math.radians(FeetAngle_5)) + 50
            PosZ_5 = -TibiaLength * 2 / 3

            # Leg 6
            PosX_6 = BodyCenterOffsetX_6 + 150 * math.cos(math.radians(FeetAngle_6))
            PosY_6 = BodyCenterOffsetY_6 + 150 * math.sin(math.radians(FeetAngle_6)) - 50
            PosZ_6 = -TibiaLength * 2 / 3

        elif i == 1: # 2 STEP
            FemurUp_246()

            # Leg 1
            PosX_1 = BodyCenterOffsetX_1 + 150 * math.cos(math.radians(FeetAngle_1))
            PosY_1 = BodyCenterOffsetY_1 + 150 * math.sin(math.radians(FeetAngle_1)) - 50
            PosZ_1 = -TibiaLength * 2 / 3

            # Leg 2
            PosX_2 = BodyCenterOffsetX_2 + 150 * math.cos(math.radians(FeetAngle_2))
            PosY_2 = BodyCenterOffsetY_2 + 150 * math.sin(math.radians(FeetAngle_2)) + 50
            PosZ_2 = -TibiaLength * 2 / 3

            # Leg 3
            PosX_3 = BodyCenterOffsetX_3 + 150 * math.cos(math.radians(FeetAngle_3))
            PosY_3 = BodyCenterOffsetY_3 + 150 * math.sin(math.radians(FeetAngle_3)) - 50
            PosZ_3 = -TibiaLength * 2 / 3

            # Leg 4
            PosX_4 = BodyCenterOffsetX_4 + 150 * math.cos(math.radians(FeetAngle_4))
            PosY_4 = BodyCenterOffsetY_4 + 150 * math.sin(math.radians(FeetAngle_4)) + 50
            PosZ_4 = -TibiaLength * 2 / 3

            # Leg 5
            PosX_5 = BodyCenterOffsetX_5 + 150 * math.cos(math.radians(FeetAngle_5))
            PosY_5 = BodyCenterOffsetY_5 + 150 * math.sin(math.radians(FeetAngle_5)) - 50
            PosZ_5 = -TibiaLength * 2 / 3

            # Leg 6
            PosX_6 = BodyCenterOffsetX_6 + 150 * math.cos(math.radians(FeetAngle_6))
            PosY_6 = BodyCenterOffsetY_6 + 150 * math.sin(math.radians(FeetAngle_6)) + 50
            PosZ_6 = -TibiaLength * 2 / 3

        else:
            break

        i = i + 1

        # Angles
        # Leg 1
        CoxaAngle_1 = math.degrees(math.atan2((PosY_1 - BodyCenterOffsetY_1), (PosX_1 - BodyCenterOffsetX_1))) - FeetAngle_1
        TibiaAngle_1 = -math.degrees(math.acos(((PosX_1 - BodyCenterOffsetX_1)**2 + (PosY_1 - BodyCenterOffsetY_1)**2 + PosZ_1**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_1 - BodyCenterOffsetX_1)**2 + (PosY_1 - BodyCenterOffsetY_1)**2)) / (2 * FemurLength * TibiaLength)))
        FemurAngle_1 = math.degrees(math.atan2((FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_1))) * PosZ_1 - TibiaLength * math.sin(math.radians(TibiaAngle_1)) * (math.sqrt((PosX_1 - BodyCenterOffsetX_1)**2 + (PosY_1 - BodyCenterOffsetY_1)**2) - CoxaLength), (math.sqrt((PosX_1 - BodyCenterOffsetX_1)**2 + (PosY_1 - BodyCenterOffsetY_1)**2) - CoxaLength) * (FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_1))) + PosZ_1 * TibiaLength * math.sin(math.radians(TibiaAngle_1))))

        # Leg 2
        CoxaAngle_2 = math.degrees(math.atan2((PosY_2 - BodyCenterOffsetY_2), (PosX_2 - BodyCenterOffsetX_2))) - FeetAngle_2
        TibiaAngle_2 = -math.degrees(math.acos(((PosX_2 - BodyCenterOffsetX_2)**2 + (PosY_2 - BodyCenterOffsetY_2)**2 + PosZ_2**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_2 - BodyCenterOffsetX_2)**2 + (PosY_2 - BodyCenterOffsetY_2)**2)) / (2 * FemurLength * TibiaLength)))
        FemurAngle_2 = math.degrees(math.atan2((FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_2))) * PosZ_2 - TibiaLength * math.sin(math.radians(TibiaAngle_2)) * (math.sqrt((PosX_2 - BodyCenterOffsetX_2)**2 + (PosY_2 - BodyCenterOffsetY_2)**2) - CoxaLength), (math.sqrt((PosX_2 - BodyCenterOffsetX_2)**2 + (PosY_2 - BodyCenterOffsetY_2)**2) - CoxaLength) * (FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_2))) + PosZ_2 * TibiaLength * math.sin(math.radians(TibiaAngle_2))))

        # Leg 3
        CoxaAngle_3 = math.degrees(math.atan2((PosY_3 - BodyCenterOffsetY_3), (PosX_3 - BodyCenterOffsetX_3))) - FeetAngle_3
        TibiaAngle_3 = -math.degrees(math.acos(((PosX_3 - BodyCenterOffsetX_3)**2 + (PosY_3 - BodyCenterOffsetY_3)**2 + PosZ_3**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_3 - BodyCenterOffsetX_3)**2 + (PosY_3 - BodyCenterOffsetY_3)**2)) / (2 * FemurLength * TibiaLength)))
        FemurAngle_3 = math.degrees(math.atan2((FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_3))) * PosZ_3 - TibiaLength * math.sin(math.radians(TibiaAngle_3)) * (math.sqrt((PosX_3 - BodyCenterOffsetX_3)**2 + (PosY_3 - BodyCenterOffsetY_3)**2) - CoxaLength), (math.sqrt((PosX_3 - BodyCenterOffsetX_3)**2 + (PosY_3 - BodyCenterOffsetY_3)**2) - CoxaLength) * (FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_3))) + PosZ_3 * TibiaLength * math.sin(math.radians(TibiaAngle_3))))

        # Leg 4
        CoxaAngle_4 = math.degrees(math.atan2((PosY_4 - BodyCenterOffsetY_4), (PosX_4 - BodyCenterOffsetX_4))) - FeetAngle_4
        TibiaAngle_4 = -math.degrees(math.acos(((PosX_4 - BodyCenterOffsetX_4)**2 + (PosY_4 - BodyCenterOffsetY_4)**2 + PosZ_4**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_4 - BodyCenterOffsetX_4)**2 + (PosY_4 - BodyCenterOffsetY_4)**2)) / (2 * FemurLength * TibiaLength)))
        FemurAngle_4 = math.degrees(math.atan2((FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_4))) * PosZ_4 - TibiaLength * math.sin(math.radians(TibiaAngle_4)) * (math.sqrt((PosX_4 - BodyCenterOffsetX_4)**2 + (PosY_4 - BodyCenterOffsetY_4)**2) - CoxaLength), (math.sqrt((PosX_4 - BodyCenterOffsetX_4)**2 + (PosY_4 - BodyCenterOffsetY_4)**2) - CoxaLength) * (FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_4))) + PosZ_4 * TibiaLength * math.sin(math.radians(TibiaAngle_4))))

        # Leg 5
        CoxaAngle_5 = math.degrees(math.atan2((PosY_5 - BodyCenterOffsetY_5), (PosX_5 - BodyCenterOffsetX_5))) - FeetAngle_5
        TibiaAngle_5 = -math.degrees(math.acos(((PosX_5 - BodyCenterOffsetX_5)**2 + (PosY_5 - BodyCenterOffsetY_5)**2 + PosZ_5**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_5 - BodyCenterOffsetX_5)**2 + (PosY_5 - BodyCenterOffsetY_5)**2)) / (2 * FemurLength * TibiaLength)))
        FemurAngle_5 = math.degrees(math.atan2((FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_5))) * PosZ_5 - TibiaLength * math.sin(math.radians(TibiaAngle_5)) * (math.sqrt((PosX_5 - BodyCenterOffsetX_5)**2 + (PosY_5 - BodyCenterOffsetY_5)**2) - CoxaLength), (math.sqrt((PosX_5 - BodyCenterOffsetX_5)**2 + (PosY_5 - BodyCenterOffsetY_5)**2) - CoxaLength) * (FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_5))) + PosZ_5 * TibiaLength * math.sin(math.radians(TibiaAngle_5))))
        #print("Leg 5: ", CoxaAngle_5, FemurAngle_5, TibiaAngle_5, "\n")

        # Leg 6
        CoxaAngle_6 = math.degrees(math.atan2((PosY_6 - BodyCenterOffsetY_6), (PosX_6 - BodyCenterOffsetX_6))) - FeetAngle_6
        TibiaAngle_6 = -math.degrees(math.acos(((PosX_6 - BodyCenterOffsetX_6)**2 + (PosY_6 - BodyCenterOffsetY_6)**2 + PosZ_6**2 + CoxaLength**2 - FemurLength**2 - TibiaLength**2 - 2 * CoxaLength * math.sqrt((PosX_6 - BodyCenterOffsetX_6)**2 + (PosY_6 - BodyCenterOffsetY_6)**2)) / (2 * FemurLength * TibiaLength)))
        FemurAngle_6 = math.degrees(math.atan2((FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_6))) * PosZ_6 - TibiaLength * math.sin(math.radians(TibiaAngle_6)) * (math.sqrt((PosX_6 - BodyCenterOffsetX_6)**2 + (PosY_6 - BodyCenterOffsetY_6)**2) - CoxaLength), (math.sqrt((PosX_6 - BodyCenterOffsetX_6)**2 + (PosY_6 - BodyCenterOffsetY_6)**2) - CoxaLength) * (FemurLength + TibiaLength * math.cos(math.radians(TibiaAngle_6))) + PosZ_6 * TibiaLength * math.sin(math.radians(TibiaAngle_6))))

        # DXL Output
        # Leg 1
        CoxaAngle_1 = round((CoxaAngle_1) * (4096/360)) + 2048
        FemurAngle_1 = round((FemurAngle_1 + FemurBracketAngle) * (4096/360)) + 2048
        TibiaAngle_1 = round((TibiaAngle_1 + TibiaBracketAngle) * (4096/360)) + 2048

        # Leg 2
        CoxaAngle_2 = round((CoxaAngle_2) * (4096/360)) + 2048
        FemurAngle_2 = round((FemurAngle_2 + FemurBracketAngle) * (4096/360)) + 2048
        TibiaAngle_2 = round((TibiaAngle_2 + TibiaBracketAngle) * (4096/360)) + 2048

        # Leg 3
        CoxaAngle_3 = round((CoxaAngle_3) * (4096/360)) + 2048
        FemurAngle_3 = round((FemurAngle_3 + FemurBracketAngle) * (4096/360)) + 2048
        TibiaAngle_3 = round((TibiaAngle_3 + TibiaBracketAngle) * (4096/360)) + 2048

        # Leg 4
        CoxaAngle_4 = round((CoxaAngle_4) * (4096/360)) + 2048
        FemurAngle_4 = round((FemurAngle_4 + FemurBracketAngle) * (4096/360)) + 2048
        TibiaAngle_4 = round((TibiaAngle_4 + TibiaBracketAngle) * (4096/360)) + 2048

        # Leg 5
        CoxaAngle_5 = round((CoxaAngle_5) * (4096/360)) + 2048
        if CoxaAngle_5 < 0:
            CoxaAngle_5 = CoxaAngle_5 + 4096
        FemurAngle_5 = round((FemurAngle_5 + FemurBracketAngle) * (4096/360)) + 2048
        TibiaAngle_5 = round((TibiaAngle_5 + TibiaBracketAngle) * (4096/360)) + 2048
        #print("Leg 5: ", CoxaAngle_5, FemurAngle_5, TibiaAngle_5, "\n")

        # Leg 6
        CoxaAngle_6 = round((CoxaAngle_6) * (4096/360)) + 2048
        FemurAngle_6 = round((FemurAngle_6 + FemurBracketAngle) * (4096/360)) + 2048
        TibiaAngle_6 = round((TibiaAngle_6 + TibiaBracketAngle) * (4096/360)) + 2048

        # Generating
        # Coxa
        dxl_goal_position = CoxaAngle_1
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(1, param_goal_position)

        dxl_goal_position = CoxaAngle_2
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(2, param_goal_position)

        dxl_goal_position = CoxaAngle_3
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(3, param_goal_position)

        dxl_goal_position = CoxaAngle_4
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(4, param_goal_position)

        dxl_goal_position = CoxaAngle_5
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(5, param_goal_position)

        dxl_goal_position = CoxaAngle_6
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(6, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        time.sleep(0.05)
        groupSyncWrite.clearParam()

        # Tibia
        dxl_goal_position = TibiaAngle_1
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(21, param_goal_position)

        dxl_goal_position = TibiaAngle_2
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(22, param_goal_position)

        dxl_goal_position = TibiaAngle_3
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(23, param_goal_position)

        dxl_goal_position = TibiaAngle_4
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(24, param_goal_position)

        dxl_goal_position = TibiaAngle_5
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(25, param_goal_position)

        dxl_goal_position = TibiaAngle_6
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(26, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        time.sleep(0.05)
        groupSyncWrite.clearParam()

        # Femur
        dxl_goal_position = FemurAngle_1
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(11, param_goal_position)

        dxl_goal_position = FemurAngle_2
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(12, param_goal_position)

        dxl_goal_position = FemurAngle_3
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(13, param_goal_position)

        dxl_goal_position = FemurAngle_4
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(14, param_goal_position)

        dxl_goal_position = FemurAngle_5
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(15, param_goal_position)

        dxl_goal_position = FemurAngle_6
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(16, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        time.sleep(0.3)
        groupSyncWrite.clearParam()

# 포트 닫기
#portHandler.closePort()
