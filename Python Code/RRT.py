
from shapely.geometry import Point,LineString
from shapely.geometry.polygon import Polygon as Poly
import matplotlib.pyplot as plt
from random import randint, uniform
import math
import time
from math import pi,sqrt,sin,cos,atan2
from Draw_Map import Drawing
show_animation = False

MAP_SIZE =[600,600]
SAFE_BUFFER = 35
MOVE_DISTANCE = 150
class Node:
    def __init__(self,x,y,parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, drawing, src, dest, obsList,
                 MAP_SIZE, expandDis=MOVE_DISTANCE, maxNode = 200, goalSampleRate=10):
        """
        Setting Parameter

        start:Start Position [x,y]
        end:Goal Position [x,y]
        obsList:obstacle Positions [[[x1,y1],[x2,y2],[x3.y3]],...]

        """
        self.lineList = []
        self.drawing = drawing
        self.nodeList = []
        self.start = Node(src.x,src.y)
        self.end = Node(dest.x,dest.y)
        self.map_size = MAP_SIZE
        self.expandDis = expandDis
        self.maxNode = maxNode
        self.goalSampleRate = goalSampleRate
        self.obsList = obsList

    def distance(self,src,dest):
        return sqrt( (src.x - dest.x)**2 + (src.y - dest.y)**2 )

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [self.distance(node,rnd) for node in nodeList]
        return dlist.index(min(dlist))

    def Planning(self, animation=True):
        u"""
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList.append(self.start)


        while len(self.nodeList) < self.maxNode:
            # Random Sampling
            rnd = []
            if randint(0,100) < self.goalSampleRate:
                rnd = self.end
            else:
                rnd = Point(randint(0,self.map_size[0]), randint(0,self.map_size[1]))

            # Find nearest node
            nearestIndex = self.GetNearestListIndex(self.nodeList, rnd)
            nearestNode = self.nodeList[nearestIndex]
            if self.distance(nearestNode,rnd) < MOVE_DISTANCE:
                continue
            # expand tree


            i = 1
            theta = math.atan2(rnd.y - nearestNode.y, rnd.x - nearestNode.x)
            prevNode = nearestNode

            while(i*self.expandDis < self.distance(nearestNode,rnd)):
                newX = nearestNode.x + i*self.expandDis * math.cos(theta)
                newY = nearestNode.y + i*self.expandDis * math.sin(theta)

                # check border
                if newX < SAFE_BUFFER or newX > self.map_size[0] - SAFE_BUFFER  or newY < SAFE_BUFFER or newY > self.map_size[1] - SAFE_BUFFER:
                    break

                newParent = self.nodeList.index(prevNode)
                newNode = Node(newX,newY,newParent)



                # check if point in safe distance away from obstacle
                tempPt = Point(newNode.x,newNode.y)
                stopFlag = False
                for poly in self.obsList:
                    if tempPt.distance(poly) < SAFE_BUFFER+30:
                        stopFlag= True
                        break
                if stopFlag:
                    break

                # check if line between point intersect obstacle
                tempLine = [[newNode.x,newNode.y],[prevNode.x,prevNode.y]]
                line = LineString(tempLine)
                for poly in self.obsList:

                    if poly.intersects(line):
                        stopFlag = True
                        break
                if stopFlag:
                    break

                if newNode in self.nodeList:
                    continue

                displayLine = [[newNode.x,self.map_size[1] - newNode.y],[prevNode.x,self.map_size[1]- prevNode.y]]
                self.lineList.append(self.drawing.canvas.create_line(displayLine,width = 3))
                prevNode = newNode
                self.nodeList.append(newNode)
                i = i + 1

            if self.distance(self.nodeList[-1], self.end) < MOVE_DISTANCE:
                break
            if animation:
                self.drawing.tk.update()


            # check goal


        index = self.GetNearestListIndex(self.nodeList,self.end)
        self.end.parent = index


        path = [[self.end.x, self.end.y]]
        lastIndex = self.end.parent
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent

        path.append([self.start.x, self.start.y])

        curLocation = [self.start.x,self.start.y]
        for i in reversed(path):
            displayLine = [[i[0], self.map_size[1] - i[1]], [curLocation[0], self.map_size[1] - curLocation[1]]]
            self.lineList.append(self.drawing.canvas.create_line(displayLine, width=3, fill='green'))
            curLocation = i
        self.drawing.tk.update()
        return path


def createPolygon(obs):
    buffer = 10
    polygon = Poly([(obs[0][0] - buffer, obs[0][1] - buffer), (obs[1][0] - buffer, obs[1][1] + buffer),
                    (obs[2][0] + buffer, obs[3][1] + buffer), (obs[3][0] + buffer, obs[3][1] - buffer)])
    return polygon


