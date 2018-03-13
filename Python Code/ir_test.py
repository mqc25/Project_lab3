from sympy import pi
from sympy import sin, cos, atan
#Dimension of the box
boxX = 600
boxY = 605
#Parameter:
#x: position in the x direction of the center of the car
#y: position in the x direction of the center of the car
#theta(center of mass): the angle of head of the car compare to x-axis (need to add -pi/2 to theta if used for the right sensor)
#offset(center of mass): how far from the sensor to the center of the car (3 for right sensor, 4.9 for front sensor)
#theta: the angle of head of the car compare to x-axis (need to add -0.76*pi to theta if used for the right sensor)
#offset(centter at 2 wheels: how far from the sensor to the center of the car (5 for right sensor, 2.5 for front sensor)

def ir_estimator(x, y, theta,offset):
    range = 0.0
    Xsensor = 0.0
    Ysensor = 0.0
    wall = 0

#check the input angle then calculate the coordinate of the sensors
    if(0 <= theta and theta <= pi/2):
        Xsensor = float (x + (offset * cos(theta)))
        Ysensor = float (y + (offset * sin(theta)))
    elif(pi/2 < theta and theta <= pi):
        Xsensor = float(x - (offset * sin(theta-(pi/2))))
        Ysensor = float(y + (offset * cos(theta-(pi/2))))
    elif(-pi/2 <= theta and theta < 0):
        Xsensor = float(x + (offset * cos(-theta)))
        Ysensor = float(y - (offset * sin(-theta)))
    else:
        Xsensor = float(x - (offset * sin(-(pi/2)-theta)))
        Ysensor = float(y - (offset * cos(-(pi/2)-theta)))
#divide the box into 4 small rectangulars with:
    X1 = (boxX / 2) - Xsensor
    X2 = boxX - X1
    Y1 = (boxY / 2) - Ysensor
    Y2 = boxY - Y1
#determine at which wall the car is facing and
#calculate the expected sensors reading
    if (-atan(Y2 / X1) <= theta and theta <= atan(Y1 / X1)):
        wall = 1
        range = X1 / cos(theta)
    elif(atan(Y1/X1)<= theta and theta<= pi/2 + atan(X2/Y1)):
        wall = 2
        if(atan(Y1/X1)<=theta and theta <= pi/2):
            range = Y1 / cos((pi/2)-theta)
        else:
            range = Y1 / cos(theta-(pi/2))
    elif((-pi/2)-atan(X2/Y2) <= theta and theta <= -atan(Y2/X1)):
        wall = 3
        if((-pi/2) <= theta and theta <= -atan(Y2/X1)):
            range = Y2/cos((pi/2)+theta)
        else:
            range = Y2/cos (-theta-(pi/2))
    else:
        wall = 4
        if((pi/2)+atan(X2/Y1) <= theta and theta <= pi):
            range = X2/cos(pi-theta)
        else:
            range = X2/cos(pi+theta)
    return float(range)

def user_input (Xr, Yr, theta):
    # convert x, y of middle of the wheels to center of mass
    x = 0.0;
    y = 0.0;
    if (0 <= theta and theta <= pi / 2):
        x = float(Xr - (2.5 * cos(theta)))
        y = float(Yr - (2.5 * sin(theta)))
    elif (pi / 2 < theta and theta <= pi):
        x = float(Xr + (2.5 * sin(theta - (pi / 2))))
        y = float(Yr - (2.5 * cos(theta - (pi / 2))))
    elif (-pi / 2 <= theta and theta < 0):
        x = float(Xr - (2.5 * cos(-theta)))
        y = float(Yr + (2.5 * sin(-theta)))
    else:
        x = float(Xr + (2.5 * sin(-(pi / 2) - theta)))
        y = float(Yr + (2.5 * cos(-(pi / 2) - theta)))

    sensorFront =  ir_estimator(x, y, theta-pi/2, 3)
    sensorRear =  ir_estimator(x, y, theta, 4.9)

    return sensorRear,sensorFront


