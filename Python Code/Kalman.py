import numpy as np
from math import pi,sin,cos,radians
import matplotlib.pyplot as plt
from ir_test import user_input
from scipy import optimize


# Estimation parameter of EKF
Q = np.diag([0.7,0.7,0.022])
R = np.diag([5.09,12.01,0.03])

#Q = np.diag([0.1,0.1,0.1])
#R = np.diag([0.2,0.2,0.2])

TURN = 45
MOVE = 30
show_animation = True



#x[0] x
#x[1] y
#x[2] angle
#x[3] input
def motion_model(x,u):
    stdAngle = x.item(2)# + pi/2
    moveFlag = 0
    turnFlag = 0

    if u == 'FW':
        moveFlag = 1
    elif u == 'BW':
        moveFlag = -1
    elif u == 'RT':
        turnFlag = -1
    elif u == 'LT':
        turnFlag = 1


    newX = x.item(0) + MOVE*cos(stdAngle)*moveFlag
    newY = x.item(1) + MOVE*sin(stdAngle)*moveFlag
    newAngle = x.item(2) + radians(TURN*turnFlag)


    return np.matrix([newX, newY, newAngle])

def observation_model(x):
    a,b = user_input(x.item(0),x.item(1),x.item(2))
    return np.matrix([a,b,radians(x.item(2))])


def jacobF(x):
    '''
    # Jacobian of Motion Model
    stdAngle = x[2] + pi / 2
    moveFlag = 0
    turnFlag = 0

    if x[3] == 'FW':
        moveFlag = 1
    elif x[3] == 'BW':
        moveFlag = -1
    elif x[3] == 'RT':
        turnFlag = -1
    elif x[3] == 'LT':
        turnFlag = 1
    '''

    jF = np.matrix([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]])

    return jF


def jacobH(x):
    # Jacobian of Observation Model
    jH =  np.matrix([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]])

    return jH


def ekf_estimation(xEst, PEst, z, u):

    #  Predict
    xPred = motion_model(xEst, u)

    jF = jacobF(xPred)
    PPred = jF * PEst * jF.T + Q

    #  Update
    jH = jacobH(xPred)
    zPred = observation_model(xPred)
    print xPred



    y = z - zPred
    print y

    S = jH * PPred * jH.T + R
    K = PPred * jH.T * np.linalg.inv(S)
    print K
    xEst = xPred + K*y
    PEst = (np.eye(len(xEst)) - K * jH) * PPred

    print xEst

    print '-----------------'
    return xEst, PEst






def main():

    time = 0.0
    xTrue1 = [[0,0],
             [25,0],
             [55,0],
             [50,0],
             [50, 0],
             [50,-32],
             [50,-65],
             [51,-62],
             [49,-70]]

    xTrue2 = [[0,0],
              [30,0],
              [58,1],
              [55,1],
              [53,0],
              [45,-30],
              [42,-65],
              [55,-60],
              [48,-65]]

    angleTrue1 = [0, 0, 0, 45, 90, 90, 90, 45, 4]

    angleTrue2 = [0, 0, 3, 48, 90, 90, 90, 45, -3]

    measurement1 = [[265,279],
                   [265,253],
                   [263,225],
                   [310,318],
                   [224,276],
                   [223,306],
                   [219,321],
                   [270,336],
                   [201,230]]

    measurement2 = [[268,278],
                    [264,253],
                    [267,224],
                    [307,318],
                    [221,279],
                    [225,310],
                    [231,340],
                    [256,328],
                    [203,240]]

    angleMeasure1 = [0, -2, -6, 31, 66, 64, 62, 37, 3]

    angleMeasure2 = [0, -9, -4, 26, 67, 78, 66, 30, -9]

    # State Vector [x y yaw v]'
    xKamal = np.matrix(np.zeros((3, 1)))
    P = np.eye(3)
    u = ['','FW','FW','LT','LT','BW','BW','RT','RT']


    # State Vector [x y yaw v]'
    xEst = np.matrix([0,0,0])
    PEst = np.eye(3)

    # history
    hxEst = [[0],[0]]
    hxTrue = [[0],[0]]

    for i in range(1,9):

        measurement = [measurement1[i][0],measurement1[i][1],radians(angleMeasure1[i])]


        xEst, PEst = ekf_estimation(xEst, PEst, np.matrix(measurement), u[i])

        # store data history
        hxEst[0].append(xEst.item(0))
        hxEst[1].append(xEst.item(1))

        hxTrue[0].append(xTrue1[i][0])
        hxTrue[1].append(xTrue1[i][1])


        hxTrue.append(xTrue1[i])

        if show_animation:
            plt.cla()

            plt.plot(hxTrue[0],
                     hxTrue[1], "-b")

            plt.plot(hxEst[0],
                     hxEst[1], "-r")

            plt.axis('equal')
            plt.grid(True)
            plt.pause(0.5)

if __name__ == '__main__':
    main()
