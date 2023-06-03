from pyPS4Controller.controller import Controller
from dynamixel_sdk import *
import INHERO_initial
import INHERO_initial2
import INHERO_forward
import INHERO_backward
import INHERO_leftmove
import INHERO_rightmove
import INHERO_northeast
import INHERO_northwest
import INHERO_southwest
import INHERO_southeast
import INHERO_dance1
import INHERO_dance2
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
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, 64, 1) # 모터 회전 가능하도록 설정
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 48, MIN_ANGLE) # 모터 최소 각도 설정
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, 52, MAX_ANGLE) # 모터 최대 각도 설정

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

    def on_x_press(self):
        self.mode = 1        
        self.perform_action()
    def on_x_release(self):
        pass

    def on_triangle_press(self):
        self.mode = 2
        self.perform_action()
    def on_triangle_release(self):
        pass

    def on_circle_press(self):
        self.mode = 3
        self.perform_action()
    def on_circle_release(self):
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

    def on_right_arrow_press(self):
        self.mode = 'DOWN'
        self.perform_action()
    def on_left_right_arrow_release(self):
        passIHR_MC_ccw

    def on_down_arrow_press(self):
        self.mode = 'RIGHT'
        self.perform_action()
    def on_up_down_arrow_release(self):
        pass

    def on_left_arrow_press(self):
        self.mode = 'LEFT'
        self.perform_action()
    def on_left_right_arrow_release(self):
        pass

    def on_L3_up(self,value):
        input_value = round(-value/3000)
        print("L3 up", input_value)
    def on_L3_down(self,value):
        input_value = round(value/3000)
        print("L3 down", input_value)
    def on_L3_left(self,value):
        input_value = round(-value/3000)
        print("L3 left", input_value)
    def on_L3_right(self,value):
        input_value = round(value/3000)
        print("L3 right", input_value)
    def on_L3_press(self):
        print("L3 press")
    def on_L3_release(self):
        print("L3 release")
    def on_L3_x_at_rest(self):
        print("L3 X break")
    def on_L3_y_at_rest(self):
        print("L3 Y break")

    def perform_action(self):
        if self.mode == 0:
            print("0 operate")
            INHERO_initial.Generating()
        elif self.mode == 'UP':
            print("Go")
            INHERO_forward.Generating()
        elif self.mode == 'DOWN':
            print("Back")
            INHERO_backward.Generating()
        elif self.mode == 'RIGHT':
            print("Clockwise")
            INHERO_cw.Generating()
        elif self.mode == 'LEFT':
            print("Counter Clockwise")
            INHERO_ccw.Generating()

controller = InheroJoy(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()
