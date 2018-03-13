import time
from math import sin, cos, pi, sqrt, radians

from shapely.geometry import Point as PT
from shapely.geometry.polygon import Polygon as Poly

from Draw_Map import Drawing, rotate
from RRT import RRT

DEBUG = False
TURN_ANGLE = 45
MOVE_DISTANCE = 30
MAP_SIZE = [600, 600]
FPS = 5

border = [[0, 0], [0, MAP_SIZE[1]], MAP_SIZE, [MAP_SIZE[0], 0]]

SAFE_DISTANCE = 65

x = []
y = []




class Point:
    def __init__(self, x, y):
        self.x = int(x)
        self.y = int(y)

    def add(self, x, y):
        self.x = int(self.x + int(x))
        self.y = int(self.y + int(y))


def distance(src, dest):
    return sqrt((src.x - dest.x) ** 2 + (src.y - dest.y) ** 2)


def genOrientation(src, angle):
    # orientation = [[src.x-45,src.y+30],[src.x+45,src.y+30],[src.x+45,src.y-70],[src.x-45,src.y-70]]
    orientation = [[src.x - 45, src.y - 70], [src.x - 45, src.y + 30], [src.x + 45, src.y + 30],
                   [src.x + 45, src.y - 70]]
    orientation = rotate(orientation, angle, [src.x, src.y])
    polygon = Poly(orientation)

    return polygon


def validMovement(src, angle):
    validPoint = []
    polyCover = []
    stdAngle = radians(angle) + pi / 2.

    for i in range(8):
        curAngle = stdAngle + i * radians(TURN_ANGLE)
        x = src.x + MOVE_DISTANCE * cos(curAngle)
        y = src.y + MOVE_DISTANCE * sin(curAngle)
        newPoint = Point(x, y)
        validPoint.append(newPoint)
        polyCover.append(genOrientation(newPoint, curAngle * 180 / pi))

    return validPoint, polyCover


def turnFromToAngle(src, dest):
    turnAngle = dest - src
    turnAngle = (turnAngle + 180) % 360 - 180
    turn = int(turnAngle / TURN_ANGLE)
    action = []
    if turn > 0:
        for i in range(turn):
            action.append('LT')
    else:
        for i in range(-turn):
            action.append('RT')
    return action

travelNode = []
def goFromTo(src, angle, dest, obsList):

    curLocation = src
    curAngle = angle
    action = []
    while (distance(curLocation, dest) > MOVE_DISTANCE):

        validPoint, polyCover = validMovement(curLocation, curAngle)
        minDistance = 99999
        nearestPoint = None

        for k in range(8):

            x = validPoint[k].x
            y = 600 - validPoint[k].y
            r = 10
            drawing.canvas.create_oval(x - r, y - r, x + r, y + r, fill='violet')

            if checkObs(validPoint[k], obsList):
                x = validPoint[k].x
                y = 600 - validPoint[k].y
                r = 10
                drawing.canvas.create_oval(x - r, y - r, x + r, y + r, fill='red')
                continue

            tempDis = distance(validPoint[k], dest)

            if (tempDis < (minDistance )):
                for i in travelNode:
                    '''
                    if distance(validPoint[k],i) < MOVE_DISTANCE - 10:
                        print "here"
                    '''
                    continue
                nearestPoint = validPoint[k]
                minDistance = tempDis
            '''
            elif (tempDis < (minDistance + 3)):
                for i in travelNode:
                    if distance(validPoint[k], i) < MOVE_DISTANCE/2:
                        continue
                if (k == 0):
                    nearestPoint = validPoint[k]
                    minDistance = tempDis
            '''
        if minDistance == 99999:
            continue
        travelNode.append(nearestPoint)
        index = validPoint.index(nearestPoint)

        if (index < 4):
            for i in range(index):
                action.append('LT')
                curAngle += TURN_ANGLE

        # elif (index == 4):
        #    action.append('BW')

        else:
            for i in range(8 - index):
                action.append('RT')
                curAngle -= TURN_ANGLE

        action.append('FW')
        print action
        curLocation = nearestPoint
        x = nearestPoint.x
        y = 600 - nearestPoint.y
        r = 10
        drawing.canvas.create_oval(x-r, y-r, x+r, y+r,fill='yellow')
        drawing.tk.update()
        time.sleep(0.15)

    return action


def move(drawing, action):
    for act in action:
        if act == 'LT':
            drawing.drawing(0, TURN_ANGLE, FPS)
        elif act == 'RT':
            drawing.drawing(0, -TURN_ANGLE, FPS)
        elif act == 'FW':
            drawing.drawing(MOVE_DISTANCE, 0, FPS)
        elif act == 'BW':
            drawing.drawing(-MOVE_DISTANCE, 0, FPS)

    return action


def checkObs(center, obsList):
    '''
    for i in obsList:
        if poly.intersects(i):
            return True
    return False
    '''
    pt = PT(center.x, center.y)
    for i in obsList:
        if pt.distance(i) < SAFE_DISTANCE:
            return True
    return False





def createPolygon(obs):
    buffer = 10
    polygon = Poly([(obs[0][0] - buffer, obs[0][1] - buffer), (obs[1][0] - buffer, obs[1][1] + buffer),
                    (obs[2][0] + buffer, obs[2][1] + buffer), (obs[3][0] + buffer, obs[3][1] - buffer)])

    return polygon


dest = Point(450, 500)
src = Point(300,250)
obsList = []
drawing = Drawing(MAP_SIZE, [src.x, src.y, 0])


obs1 = [[250, 400], [250, 450], [350, 450], [350, 400]]
#obs2 = [[250, 500], [250, 550], [350, 550], [350, 500]]
#obs2 = [[450, 400], [450, 450], [550, 450], [550, 400]]
#obs3 = [[50, 400], [50, 450], [150, 450], [150, 400]]
#obs4 = [[150, 50], [150, 100], [250, 100], [250, 50]]
'''
border1= [[0, 0], [0, MAP_SIZE[1]], [5, MAP_SIZE[1]+5], [5,0]]
border2= [[0, MAP_SIZE[1]], MAP_SIZE, [MAP_SIZE[0]-5, MAP_SIZE[1]-5], [0, MAP_SIZE[1]-5]]
border3= [[MAP_SIZE[0], 0], [MAP_SIZE[0], MAP_SIZE[1]], [MAP_SIZE[0]-5, MAP_SIZE[1]+5], [MAP_SIZE[0]-5,0]]
border4= [[0, 0], [MAP_SIZE[0],0], [MAP_SIZE[0], 5], [0, 5]]


obsList.append(createPolygon(border1))
obsList.append(createPolygon(border2))
obsList.append(createPolygon(border3))
obsList.append(createPolygon(border4))
'''
obsList.append(createPolygon(obs1))
'''
obsList.append(createPolygon(obs2))
obsList.append(createPolygon(obs3))
obsList.append(createPolygon(obs4))
drawing.drawObs(obs4)

drawing.drawObs(obs2)
drawing.drawObs(obs3)
'''
drawing.drawObs(obs1)
drawing.tk.update()
#time.sleep(20)
rrt = RRT(drawing, src, dest, MAP_SIZE=[600, 600], obsList=obsList)
path = rrt.Planning(animation=True)

curLocation = src
print path
for i in reversed(path):
    newLocation = Point(i[0], i[1], )
    if distance(curLocation, newLocation) < MOVE_DISTANCE:
        continue
    action = goFromTo(curLocation, drawing.car.angle, newLocation, obsList)
    move(drawing, action)
    curLocation = Point(drawing.car.position[0],drawing.car.position[1])
    print action

action = goFromTo(curLocation, drawing.car.angle, dest, obsList)
move(drawing, action)
curLocation = Point(drawing.car.position[0],drawing.car.position[1])
time.sleep(0.5)
# move(drawing, action)
act = turnFromToAngle(drawing.car.angle, 360)
move(drawing, act)

drawing.tk.mainloop()
