from pyPS4Controller.controller import Controller
from dynamixel_sdk import *
import INHERO_initial
import INHERO_initial2
import INHERO_forward
import INHERO_backward
import INHERO_leftmove
import INHERO_rightmove
import INHERO_cw
import INHERO_ccw
import INHERO_northeastmove
import INHERO_northwestmove
import INHERO_southwestmove
import INHERO_southeastmove
import INHERO_dance1
import INHERO_dance2
import INHERO_tiltmove
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

# Inverse Kinematics
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

Mode_1 = 0
Mode_2 = 0
Mode_3 = 0
Mode_4 = 0
Mode_5 = 0
Mode_6 = 0

# Controller Settings
class InheroJoy(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.mode = 0
        self.perform_action()
        print("State :", self.mode)

    def on_playstation_button_press(self):
       # global self.mode
        print("State :", self.mode)

    def on_playstation_button_release(self):
        pass

    def on_triangle_press(self):
        self.mode = 1
        self.perform_action()
    def on_triangle_release(self):
        pass

    def on_circle_press(self):
        self.mode = 2
        self.perform_action()
    def on_circle_release(self):
        pass

    def on_x_press(self):
        self.mode = 3
        self.perform_action()
    def on_x_release(self):
        pass

    def on_square_press(self):
        self.mode = 4
        self.perform_action()
    def on_square_release(self):
        pass

    def on_up_arrow_press(self):
        self.mode = 'UP'
        self.perform_action()
    def on_up_down_arrow_release(self):
        pass

    def on_down_arrow_press(self):
        self.mode = 'DOWN'
        self.perform_action()
    def on_up_down_arrow_release(self):
        pass

    def on_right_arrow_press(self):
        self.mode = 'RIGHT'
        self.perform_action()
    def on_left_right_arrow_release(self):
        pass

    def on_left_arrow_press(self):
        self.mode = 'LEFT'
        self.perform_action()
    def on_left_right_arrow_release(self):
        pass

    def on_L3_press(self):
        self.mode = 5
        self.perform.action()
    def on_L3_release(self):
        pass

    def on_R3_press(self):
        self.mode = 6
        self.perform.action()
    def on_R3_release(self):
        pass

    def on_L3_up(self,value):
        input_value = round(-value/3000)
        print("L3 up", input_vamlue)
    def on_L3_down(self,value):
        input_value = round(value/3000)
        print("L3 down", input_value)
    def on_L3_left(self,value):
        input_value = round(-value/3000)
        print("L3 left", input_value)
    def on_L3_right(self,value):
        input_value = round(value/3000)
        print("L3 right", input_value)
    def on_L3_x_at_rest(self):
        print("L3 X break")
    def on_L3_y_at_rest(self):
        print("L3 Y break")

    def perform_action(self):
        global Mode_1, Mode_2, Mode_3, Mode_4, Mode_5, Mode_6

        if self.mode == 0:
            print("INHERO Operating")
            INHERO_initial.Generating()
        elif self.mode == 1: # Mode 1 (Press triangle)
            Mode_1 = 1
            Mode_2 = 0
            Mode_3 = 0
            Mode_4 = 0
            Mode_5 = 0
            Mode_6 = 0
            INHERO_initial.Generating()
            print("Present Mode : 1")
        elif self.mode == 2: # Mode 2 (Press circle)
            Mode_1 = 0
            Mode_2 = 1
            Mode_3 = 0
            Mode_4 = 0
            Mode_5 = 0
            Mode_6 = 0
            INHERO_initial.Generating()
            print("Present Mode : 2")
        elif self.mode == 3: # Mode 3 (Press x)
            Mode_1 = 0
            Mode_2 = 0
            Mode_3 = 1
            Mode_4 = 0
            Mode_5 = 0
            Mode_6 = 0
            INHERO_initial.Generating()
            print("Present Mode : 3")
        elif self.mode == 4: # Mode 4 (Press square)
            Mode_1 = 0
            Mode_2 = 0
            Mode_3 = 0
            Mode_4 = 1
            Mode_5 = 0
            Mode_6 = 0
            INHERO_initial.Generating()
            print("Present Mode : 4")
        elif self.mode == 5: # Mode 5 (Press L3)
            Mode_1 = 0
            Mode_2 = 0
            Mode_3 = 0
            Mode_4 = 0
            Mode_5 = 1
            Mode_6 = 0
            INHERO_initial.Generating()
            print("Present Mode : 5")
        elif self.mode == 6: # Mode 6 (Press R3)
            Mode_1 = 0
            Mode_2 = 0
            Mode_3 = 0
            Mode_4 = 0
            Mode_5 = 0
            Mode_6 = 1
            INHERO_initial.Generating()
            print("Present Mode : 6")

        # Mode 1
        if self.mode == 'UP' and Mode_1 == 1:
            print("Forward")
            INHERO_forward.Generating()
        elif self.mode == 'DOWN' and Mode_1 == 1:
            print("Backward")
            INHERO_backward.Generating()
        elif self.mode == 'LEFT' and Mode_1 == 1:
            print("Left")
            INHERO_leftmove.Generating()
        elif self.mode == 'RIGHT' and Mode_1 == 1:
            print("Right")
            INHERO_rightmove.Generating()

        # Mode 2
        elif self.mode == 'UP' and Mode_2 == 1:
            print("Forward")
            INHERO_forward.Generating()
        elif self.mode == 'DOWN' and Mode_2 == 1:
            print("Backward")
            INHERO_backward.Generating()
        elif self.mode == 'LEFT' and Mode_2 == 1:
            print("Counter Clockwise")
            INHERO_ccw.Generating()
        elif self.mode == 'RIGHT' and Mode_2 == 1:
            print("Clockwise")
            INHERO_cw.Generating()

        # Mode 3
        elif self.mode == 'UP' and Mode_3 == 1:
            print("Northeast")
            INHERO_northeastmove.Generating()
        elif self.mode == 'DOWN' and Mode_3 == 1:
            print("Southwest")
            INHERO_southwestmove.Generating()
        elif self.mode == 'LEFT' and Mode_3 == 1:
            print("Northwest")
            INHERO_northwestmove.Generating()
        elif self.mode == 'RIGHT' and Mode_3 == 1:
            print("Southeast")
            INHERO_southeastmove.Generating()

        # Mode 4
        elif self.mode == 'UP' and Mode_4 == 1:
            print("Forward")
            INHERO_dance1.Generating()
        elif self.mode == 'DOWN' and Mode_4 == 1:
            print("Backward")
            INHERO_dance2.Generating()
        elif self.mode == 'LEFT' and Mode_4 == 1:
            print("-3 degree")
            INHERO_tiltmove.Generating1()
        elif self.mode == 'RIGHT' and Mode_4 == 1:
            print("+3 degree")
            INHERO_tiltmove.Generating2()

        # Mode 5
        elif self.mode == 'UP' and Mode_5 == 1:
            print("dance")
            INHERO_tiltmove.Generating3()
        elif self.mode == 'DOWN' and Mode_5 == 1:
            print("Wiggle")
            INHERO_tiltmove.Generating4()
        elif self.mode == 'LEFT' and Mode_5 == 1:
            print("-3 degree")
            INHERO_tiltmove.Generating1()
        elif self.mode == 'RIGHT' and Mode_5 == 1:
            print("+3 degree")
            INHERO_tiltmove.Generating2()

controller = InheroJoy(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()
