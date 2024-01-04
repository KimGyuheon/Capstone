import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib.widgets import Slider
from math import pi, sin, cos, atan2, degrees, radians, sqrt

# Robot Geometry
CoxaLength = 60
FemurLength = 75
TibiaLength = 130

BodySideLength = 98.5 # 98.5
BodyCenterOffset1 = 64.84 # 64.5
BodyCenterOffset2 = 124.842 # 125

Offset = 140
Height = 90

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

# List of Transformation Matrix
TB_0 = [np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4))]
T0_1 = [np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4))]
T1_2 = [np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4))]
T2_3 = [np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4))]
TB_1 = [np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4))]
TB_2 = [np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4))]
TB_3 = [np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4))]

# List of Rotation Matrix
RotPos = [np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1))] 
RotAngle = [0, 0, 0]

# Initial Position
def Initial():
    
    for i in range (0, 3):
        
        PosX[i] = BodyCenterOffsetX[i] + Offset * cos(radians(FeetAngle[i]))
        PosY[i] = BodyCenterOffsetY[i] + Offset * sin(radians(FeetAngle[i]))
        PosZ[i] = -Height

    for i in range (3, 6):

        PosX[i] = BodyCenterOffsetX[i] + Offset * cos(radians(FeetAngle[i]))
        PosY[i] = BodyCenterOffsetY[i] + Offset * sin(radians(FeetAngle[i]))
        PosZ[i] = -Height
    
    return (PosX, PosY, PosZ)

# Walk Motion (홀수번 다리의 feet들은 sin파형을 그리며 보행)
def Input(dy):
    odleg = [0,2,4]
    evleg = [1,3,5]

    # Leg 1,3,5
    for i in odleg:
        PosX[i] = BodyCenterOffsetX[i] + Offset * cos(radians(FeetAngle[i]))
        PosY[i] = BodyCenterOffsetY[i] + Offset * sin(radians(FeetAngle[i])) + dy
        PosZ[i] = -Height
    
    # Leg 2,4,6
    for i in evleg:
        PosX[i] = BodyCenterOffsetX[i] + Offset * cos(radians(FeetAngle[i]))
        PosY[i] = BodyCenterOffsetY[i] + Offset * sin(radians(FeetAngle[i])) - dy
        PosZ[i] = 75 * sin(pi * (PosY[i] - (BodyCenterOffsetY[i] + Offset * sin(radians(FeetAngle[i])) - 50)) / 100) - Height

    return (PosX, PosY, PosZ)

# Input Rotation
def Rotation(PosX, PosY, PosZ, RotAngle):

    psi, theta, phi = RotAngle
    
    RotX = np.array([[1, 0, 0], [0, cos(radians(psi)), -sin(radians(psi))], [0, sin(radians(psi)), cos(radians(psi))]])
    RotY = np.array([[cos(radians(theta)), 0, sin(radians(theta))], [0, 1, 0], [-sin(radians(theta)), 0, cos(radians(theta))]])
    RotZ = np.array([[cos(radians(phi)), -sin(radians(phi)), 0], [sin(radians(phi)), cos(radians(phi)), 0], [0, 0, 1]])

    RotXY = np.dot(RotX, RotY)
    RotMatrix = np.dot(RotXY, RotZ)

    for i in range (6):
        RotPos[i] = np.dot(RotMatrix, np.array([PosX[i], PosY[i], PosZ[i]]))

    RotPosX = [RotPos[0][0], RotPos[1][0], RotPos[2][0], RotPos[3][0], RotPos[4][0], RotPos[5][0]]    
    RotPosY = [RotPos[0][1], RotPos[1][1], RotPos[2][1], RotPos[3][1], RotPos[4][1], RotPos[5][1]]
    RotPosZ = [RotPos[0][2], RotPos[1][2], RotPos[2][2], RotPos[3][2], RotPos[4][2], RotPos[5][2]]

    return (RotPosX, RotPosY, RotPosZ)

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

# Forward Kinematics
X = [np.zeros([4, 3]), np.zeros([4, 3]), np.zeros([4, 3]), np.zeros([4, 3]), np.zeros([4, 3]), np.zeros([4, 3])]

def FK(Theta = [CoxaAngle, FemurAngle, TibiaAngle]):
    
    for i in range(0, 6):
        TB_0[i] = np.array([[cos(radians(FeetAngle[i])), -sin(radians(FeetAngle[i])), 0, BodyCenterOffsetX[i]], 
                            [sin(radians(FeetAngle[i])), cos(radians(FeetAngle[i])), 0, BodyCenterOffsetY[i]], 
                            [0, 0, 1, 0], 
                            [0, 0, 0, 1]])
        T0_1[i] = np.array([[cos(radians(CoxaAngle[i])), 0, sin(radians(CoxaAngle[i])), CoxaLength * cos(radians(CoxaAngle[i]))], 
                            [sin(radians(CoxaAngle[i])), 0, -cos(radians(CoxaAngle[i])), CoxaLength * sin(radians(CoxaAngle[i]))], 
                            [0, 1, 0, 0], 
                            [0, 0, 0, 1]])
        T1_2[i] = np.array([[cos(radians(FemurAngle[i])), -sin(radians(FemurAngle[i])), 0, FemurLength * cos(radians(FemurAngle[i]))], 
                            [sin(radians(FemurAngle[i])), cos(radians(FemurAngle[i])), 0, FemurLength * sin(radians(FemurAngle[i]))],
                            [0, 0, 1, 0], 
                            [0, 0, 0, 1]])
        T2_3[i] = np.array([[cos(radians(TibiaAngle[i])), -sin(radians(TibiaAngle[i])), 0, TibiaLength * cos(radians(TibiaAngle[i]))],
                            [sin(radians(TibiaAngle[i])), cos(radians(TibiaAngle[i])), 0, TibiaLength * sin(radians(TibiaAngle[i]))], 
                            [0, 0, 1, 0], 
                            [0, 0, 0, 1]])

        TB_1[i] = np.dot(TB_0[i], T0_1[i])
        TB_2[i] = np.dot(TB_1[i], T1_2[i])
        TB_3[i] = np.dot(TB_2[i], T2_3[i])
    
    for i in range (6):
        
        X[i] = np.array([[TB_0[i][0][3], TB_0[i][1][3], TB_0[i][2][3]],
                         [TB_1[i][0][3], TB_1[i][1][3], TB_1[i][2][3]], 
                         [TB_2[i][0][3], TB_2[i][1][3], TB_2[i][2][3]], 
                         [TB_3[i][0][3], TB_3[i][1][3], TB_3[i][2][3]]])
        
    return (X)

def update(slider_val):
    
    dy = slider_val
    
    PosX, PosY, PosZ = Input(dy)
    theta = IK(PosX, PosY, PosZ)
    ang = FK(theta)
    
    #graphs = []
    for i in range(6):
        graphs[i].set_data(ang[i].T[0], ang[i].T[1])
        graphs[i].set_3d_properties(ang[i].T[2])
        graphs[i].set_linestyle('-')
        graphs[i].set_linewidth(2)
        graphs[i].set_marker('o')
        graphs[i].set_markerfacecolor('g')
        graphs[i].set_markeredgecolor('g')
        graphs[i].set_markersize('5')
        
    fig.canvas.draw_idle()

# Main
PosX, PosY, PosZ = Initial()
theta = IK(PosX, PosY, PosZ)
ang = FK(theta)

# Graph Setting
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim3d(-300, 300)
ax.set_ylim3d(-300, 300)
ax.set_zlim3d(-300, 300)
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")

plt.grid()

# Plotting Robot's Body
ax.plot3D([BodyCenterOffsetX_1, BodyCenterOffsetX_2, BodyCenterOffsetX_3, BodyCenterOffsetX_4, BodyCenterOffsetX_5, BodyCenterOffsetX_6, BodyCenterOffsetX_1], 
          [BodyCenterOffsetY_1, BodyCenterOffsetY_2, BodyCenterOffsetY_3, BodyCenterOffsetY_4, BodyCenterOffsetY_5, BodyCenterOffsetY_6, BodyCenterOffsetY_1], 
          [0, 0, 0, 0, 0, 0, 0], "-r*")

graphs = []
for i in range(6):
    
    graph, = plt.plot(ang[i][:, 0], ang[i][:, 1], ang[i][:, 2])
    graphs.append(graph)
    
    graphs[i].set_linestyle('-')
    graphs[i].set_linewidth(2)
    graphs[i].set_marker('o')
    graphs[i].set_markerfacecolor('g')
    graphs[i].set_markeredgecolor('g')
    graphs[i].set_markersize('5')

slider_pos = plt.axes([0.22, 0.04, 0.6, 0.03])
threshold_slider = Slider(slider_pos, 'y', -45.0, 45.0, 0.0)
threshold_slider.on_changed(update)

plt.show()
fig.canvas.draw()