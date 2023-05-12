from dynamixel_sdk import *
import time

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
BAUDRATE = 2000000
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

center_data = 2048

dxl_goal_position_L = center_data
dxl_goal_position_R = center_data

dxl_goal_position_ten_L_U = center_data -500
dxl_goal_position_ten_R_U = center_data -500
dxl_goal_position_ten_L_D = center_data +500
dxl_goal_position_ten_R_D = center_data +500



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
time.sleep(1)
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
time.sleep(1)
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
time.sleep(1)
groupSyncWrite.clearParam()


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
time.sleep(1)
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
time.sleep(1)
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
time.sleep(1)
groupSyncWrite.clearParam()

while True:
#    print("Press any key to continue! (or press ESC to quit!)")
#    if getch() == chr(0x1b):
#        break
#    ch = getch()

    # 전진
    if getch() == chr(49):

        # odd Leg
        # mid (ten) (Up Motion)

        dxl_goal_position_ten_L_U -= 500
        dxl_goal_position_ten_R_U += 500
        # Leg 1
        # ten : 10번대 모터, L/R : 좌/우 다리, U : 위로 올리는 동작

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U))]
        dxl_addparam_result = groupSyncWrite.addParam(11, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        # Leg 3_
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U))]
        dxl_addparam_result = groupSyncWrite.addParam(13, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        #Leg 5
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U))]
        dxl_addparam_result = groupSyncWrite.addParam(15, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        time.sleep(1)


        # up

        dxl_goal_position_L += 100
        dxl_goal_position_R -= 100
        # leg 1

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_L))]
        dxl_addparam_result = groupSyncWrite.addParam(1, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # leg 3
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_R))]
        dxl_addparam_result = groupSyncWrite.addParam(3, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # leg 5
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_L))]
        dxl_addparam_result = groupSyncWrite.addParam(5, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        time.sleep(1)


        # mid (ten) (Down Motion)
        dxl_goal_position_ten_L_D -= 500
        dxl_goal_position_ten_R_D += 500

        # Leg 1
        # ten : 10번대 모터, L/R : 좌/우 다리, D : 아래로 내리는 동작


        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D))]
        dxl_addparam_result = groupSyncWrite.addParam(11, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        # Leg 3
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D))]
        dxl_addparam_result = groupSyncWrite.addParam(13, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        #Leg 5
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D))]
        dxl_addparam_result = groupSyncWrite.addParam(15, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        time.sleep(1)

#        # low (twen)
#        # Leg 1
#        dxl_goal_position_twen += 500
#        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_twen)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#        dxl_addparam_result = groupSyncWrite.addParam(21, param_goal_position)

#        dxl_comm_result = groupSyncWrite.txPacket()
#        #time.sleep(0.1)
#        groupSyncWrite.clearParam()

#        # Leg 3
#        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_twen)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#        dxl_addparam_result = groupSyncWrite.addParam(23, param_goal_position)

#        dxl_comm_result = groupSyncWrite.txPacket()
#        #time.sleep(0.1)
#        groupSyncWrite.clearParam()

#        #Leg 5
#        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_twen)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#        dxl_addparam_result = groupSyncWrite.addParam(25, param_goal_position)

#        dxl_comm_result = groupSyncWrite.txPacket()
#        #time.sleep(0.1)
#        groupSyncWrite.clearParam()


#        time.sleep(1)


        # even Leg

        # mid (ten)  (Up Motion)
        # Leg 2
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U))]
        dxl_addparam_result = groupSyncWrite.addParam(12, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        # Leg 4
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U))]
        dxl_addparam_result = groupSyncWrite.addParam(14, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        #Leg 6
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U))]
        dxl_addparam_result = groupSyncWrite.addParam(16, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        time.sleep(1)



        # up
        # Leg 2
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_R))]
        dxl_addparam_result = groupSyncWrite.addParam(2, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # Leg 4

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_L))]
        dxl_addparam_result = groupSyncWrite.addParam(4, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # Leg 6
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_R))]
        dxl_addparam_result = groupSyncWrite.addParam(6, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()



        time.sleep(1)


        # odd Leg Rotate Back
        # up

        dxl_goal_position_L -= 200
        dxl_goal_position_R += 200
        # leg 1

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_L))]
        dxl_addparam_result = groupSyncWrite.addParam(1, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # leg 3
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_R))]
        dxl_addparam_result = groupSyncWrite.addParam(3, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # leg 5
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_L))]
        dxl_addparam_result = groupSyncWrite.addParam(5, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        time.sleep(1)


#        dxl_goal_position_ten_L_D += 500
#        dxl_goal_position_ten_R_D -= 500

        # even Leg Down
        # mid (ten)  (Down Motion)
        dxl_goal_position_ten_L_D -= 500
        dxl_goal_position_ten_R_D += 500
        # Leg 2
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D))]
        dxl_addparam_result = groupSyncWrite.addParam(12, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        # Leg 4
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D))]
        dxl_addparam_result = groupSyncWrite.addParam(14, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        # Leg 6
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D))]
        dxl_addparam_result = groupSyncWrite.addParam(16, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        time.sleep(1)





#        dxl_goal_position_ten_L_U -= 500
#        dxl_goal_position_ten_R_U += 500


        # odd Leg Up￩￩
        # mid (ten) (Up Motion)

        dxl_goal_position_ten_L_U += 500
        dxl_goal_position_ten_R_U -= 500
        # Leg 1
        # ten : 10번대 모터, L/R : 좌/우 다리, U : 위로 올리는 동작

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U))]
        dxl_addparam_result = groupSyncWrite.addParam(11, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        # Leg 3
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U))]
        dxl_addparam_result = groupSyncWrite.addParam(13, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        #Leg 5
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U))]
        dxl_addparam_result = groupSyncWrite.addParam(15, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        time.sleep(1)






        # even Leg Rotate Back
        # up
        # Leg 2
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_R))]
        dxl_addparam_result = groupSyncWrite.addParam(2, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # Leg 4

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_L))]
        dxl_addparam_result = groupSyncWrite.addParam(4, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # Leg 6
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_R))]
        dxl_addparam_result = groupSyncWrite.addParam(6, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()



        time.sleep(1)



        # odd Leg Rotate Back to centerpoint

        # up

        dxl_goal_position_L += 100
        dxl_goal_position_R -= 100
        # leg 1

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_L))]
        dxl_addparam_result = groupSyncWrite.addParam(1, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # leg 3
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_R))]
        dxl_addparam_result = groupSyncWrite.addParam(3, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # leg 5
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_L))]
        dxl_addparam_result = groupSyncWrite.addParam(5, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        time.sleep(1)








        # odd Leg Down Motion to centerpoint
        dxl_goal_position_ten_L_D += 500
        dxl_goal_position_ten_R_D -= 500

        # Leg 1
        # ten : 10번대 모터, L/R : 좌/우 다리, D : 아래로 내리는 동작


        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D))]
        dxl_addparam_result = groupSyncWrite.addParam(11, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        # Leg 3
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D))]
        dxl_addparam_result = groupSyncWrite.addParam(13, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        #Leg 5
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D))]
        dxl_addparam_result = groupSyncWrite.addParam(15, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        time.sleep(1)




        # even Leg Up
        # mid (ten)  (Up Motion)


        # Leg 2
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U))]
        dxl_addparam_result = groupSyncWrite.addParam(12, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        # Leg 4
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_U))]
        dxl_addparam_result = groupSyncWrite.addParam(14, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        #Leg 6
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_U)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_U))]
        dxl_addparam_result = groupSyncWrite.addParam(16, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        time.sleep(1)






        # even Leg Rotate to centerpoint

        # Leg 2
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_R))]
        dxl_addparam_result = groupSyncWrite.addParam(2, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # Leg 4

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_L)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_L)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_L))]
        dxl_addparam_result = groupSyncWrite.addParam(4, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        # Leg 6
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_R)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_R)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_R))]
        dxl_addparam_result = groupSyncWrite.addParam(6, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()



        time.sleep(1)





        # even Leg Down
        # mid (ten)  (Down Motion)
        dxl_goal_position_ten_L_D += 500
        dxl_goal_position_ten_R_D -= 500
        # Leg 2
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D))]
        dxl_addparam_result = groupSyncWrite.addParam(12, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        # Leg 4
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_L_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_L_D))]
        dxl_addparam_result = groupSyncWrite.addParam(14, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()

        # Leg 6
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_ten_R_D)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_ten_R_D))]
        dxl_addparam_result = groupSyncWrite.addParam(16, param_goal_position)

        dxl_comm_result = groupSyncWrite.txPacket()
        #time.sleep(0.1)
        groupSyncWrite.clearParam()


        time.sleep(1)





#        # low_twen
#        # Leg 2
#        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_twen)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#        dxl_addparam_result = groupSyncWrite.addParam(22, param_goal_position)

#        dxl_comm_result = groupSyncWrite.txPacket()
#        #time.sleep(0.1)
#        groupSyncWrite.clearParam()

#        # Leg 4
#        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_twen)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#        dxl_addparam_result = groupSyncWrite.addParam(24, param_goal_position)

#        dxl_comm_result = groupSyncWrite.txPacket()
#        #time.sleep(0.1)
#        groupSyncWrite.clearParam()


#        # Leg 6
#        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_twen)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#        dxl_addparam_result = groupSyncWrite.addParam(26, param_goal_position)

#        dxl_comm_result = groupSyncWrite.txPacket()
#        #time.sleep(0.1)
#        groupSyncWrite.clearParam()


#        time.sleep(1)

    else:
        break

    




#    dxl_goal_position = center_data + 1000
#    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#    dxl_addparam_result = groupSyncWrite.addParam(26, param_goal_position)

#    dxl_comm_result = groupSyncWrite.txPacket()
#    time.sleep(1)
#    groupSyncWrite.clearParam()
    
#    dxl_goal_position = center_data - 1500
#    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#    dxl_addparam_result = groupSyncWrite.addParam(2, param_goal_position)
    
#    dxl_goal_position = center_data + 1000
#    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#    dxl_addparam_result = groupSyncWrite.addParam(12, param_goal_position)
    
#    dxl_goal_position = center_data - 1000
#    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#    dxl_addparam_result = groupSyncWrite.addParam(22, param_goal_position)
    
#    dxl_comm_result = groupSyncWrite.txPacket()
#    time.sleep(1)
#    groupSyncWrite.clearParam()
    
#    dxl_goal_position = center_data - 1500
#    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#    dxl_addparam_result = groupSyncWrite.addParam(4, param_goal_position)
    
#    dxl_goal_position = center_data + 1000
#    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#    dxl_addparam_result = groupSyncWrite.addParam(14, param_goal_position)
    
#    dxl_goal_position = center_data - 1000
#    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#    dxl_addparam_result = groupSyncWrite.addParam(24, param_goal_position)

#    dxl_comm_result = groupSyncWrite.txPacket()
#    time.sleep(1)
#    groupSyncWrite.clearParam()
    
#    dxl_goal_position = center_data - 1500
#    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#    dxl_addparam_result = groupSyncWrite.addParam(6, param_goal_position)
    
#    dxl_goal_position = center_data + 1000
#    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#    dxl_addparam_result = groupSyncWrite.addParam(16, param_goal_position)
    
#    dxl_goal_position = center_data - 1000
#    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
#    dxl_addparam_result = groupSyncWrite.addParam(26, param_goal_position)

#    dxl_comm_result = groupSyncWrite.txPacket()
#    time.sleep(1)
#    groupSyncWrite.clearParam()

    #    dxl_goal_position = center_data - 1500
    #    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)),      DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    #    dxl_addparam_result = groupSyncWrite.addParam(1, param_goal_position)

    #    dxl_goal_position = center_data + 1000
    #    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    #    dxl_addparam_result = groupSyncWrite.addParam(11, param_goal_position)

    #    dxl_goal_position = center_data - 1000
    #    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    #    dxl_addparam_result = groupSyncWrite.addParam(21, param_goal_position)

    #    dxl_comm_result = groupSyncWrite.txPacket()
    #    time.sleep(1)
    #    groupSyncWrite.clearParam()

    #    dxl_goal_position = center_data - 1500
    #    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)),      DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    #    dxl_addparam_result = groupSyncWrite.addParam(3, param_goal_position)

    #    dxl_goal_position = center_data + 1000
    #    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    #    dxl_addparam_result = groupSyncWrite.addParam(13, param_goal_position)

    #    dxl_goal_position = center_data - 1000
    #    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    #    dxl_addparam_result = groupSyncWrite.addParam(23, param_goal_position)

    #    dxl_comm_result = groupSyncWrite.txPacket()
    #    time.sleep(1)
    #    groupSyncWrite.clearParam()

    #    dxl_goal_position = center_data - 1500
    #    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)),      DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    #    dxl_addparam_result = groupSyncWrite.addParam(5, param_goal_position)

    #    dxl_goal_position = center_data + 1000
    #    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    #    dxl_addparam_result = groupSyncWrite.addParam(15, param_goal_position)

    #    dxl_goal_position = center_data - 1000
    #    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    #    dxl_addparam_result = groupSyncWrite.addParam(25, param_goal_position)

    #    dxl_comm_result = groupSyncWrite.txPacket()
    #    time.sleep(1)
    #    groupSyncWrite.clearParam()


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
