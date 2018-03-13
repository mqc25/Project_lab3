from Tkinter import *
import time, random
from math import sin,cos,radians,pi

DEBUG = False


def rotate(points, angle, center):
    angle = radians(angle)
    cos_val = cos(angle)
    sin_val = sin(angle)
    cx, cy = center
    new_points = []
    for x_old, y_old in points:
        x_old -= cx
        y_old -= cy
        x_new = float(x_old) * cos_val - float(y_old) * sin_val
        y_new = x_old * sin_val + y_old * cos_val
        new_points.append([x_new + cx, y_new + cy])
    return new_points



def adjustDisplay(x,mapSize):
    y = [x[0], mapSize - x[1]]
    return y

class Car:
    def __init__(self,x,y,theta,mapSizeY):
        self.mapSizeY = mapSizeY
        self.position = [x,y]
        self.angle = theta
        self.orientation = [[x-45,y+30],[x+45,y+30],[x+45,y-70],[x-45,y-70]]
        self.displayPosition = adjustDisplay(self.position,mapSizeY)
        self.displayOrientation = [adjustDisplay(self.orientation[0], mapSizeY),
                                   adjustDisplay(self.orientation[1], mapSizeY),
                                   adjustDisplay(self.orientation[2], mapSizeY),
                                   adjustDisplay(self.orientation[3], mapSizeY)]


    def adjust(self):
        self.displayPosition = adjustDisplay(self.position, self.mapSizeY)
        self.displayOrientation = [adjustDisplay(self.orientation[0], self.mapSizeY),
                                   adjustDisplay(self.orientation[1], self.mapSizeY),
                                   adjustDisplay(self.orientation[2], self.mapSizeY),
                                   adjustDisplay(self.orientation[3], self.mapSizeY)]
    def moveStraight(self,distance):
        dx = distance*cos(radians(self.angle) + pi/2)
        dy = distance*sin(radians(self.angle) + pi/2)
        self.position = [self.position[0] + dx,
                         self.position[1] + dy]

        self.orientation = [[self.position[0]-45,self.position[1]+30],
                            [self.position[0]+45,self.position[1]+30],
                            [self.position[0]+45,self.position[1]-70],
                            [self.position[0]-45,self.position[1]-70]]

        self.orientation = rotate(self.orientation,self.angle,self.position)
        self.adjust()

    def moveTurn(self,angle):

        self.angle = self.angle + angle
        self.orientation = [[self.position[0] - 45, self.position[1] + 30],
                            [self.position[0] + 45, self.position[1] + 30],
                            [self.position[0] + 45, self.position[1] - 70],
                            [self.position[0] - 45, self.position[1] - 70]]
        self.orientation = rotate(self.orientation, self.angle, self.position)
        self.adjust()

class Drawing:
    def __init__(self,mapSize, carInitPosition):
        self.tk = Tk()
        self.mapSizeY = mapSize[1]
        self.canvas = Canvas(self.tk, width=mapSize[0], height=mapSize[1])
        self.canvas.pack()
        self.car = Car(carInitPosition[0],carInitPosition[1],carInitPosition[2],mapSize[1])

        self.bgImg = self.canvas.create_rectangle(0, 0, mapSize[0], mapSize[1], fill='white')  # draws background
        self.carImg = self.canvas.create_polygon(self.car.displayOrientation, fill='blue')
        self.headImg = self.canvas.create_oval(self.car.displayPosition[0]-10, self.car.displayPosition[1]-10,
                                               self.car.displayPosition[0]+10, self.car.displayPosition[1]+10, fill='black')

        for i in range(0, mapSize[0], 50):
            self.canvas.create_line([(i, 0), (i, mapSize[1])], tag='grid_line')

            # Creates all horizontal lines at intevals of 100
        for i in range(0, mapSize[1], 50):
            self.canvas.create_line([(0, i), (mapSize[0], i)], tag='grid_line')

        border = [[0, 0], [0, mapSize[1]]]
        self.canvas.create_line(border, width=10, fill='red')
        border = [[0, mapSize[1]], [mapSize[0], mapSize[1]]]
        self.canvas.create_line(border, width=10, fill='red')
        border = [[mapSize[0], mapSize[1]], [mapSize[0], 0]]
        self.canvas.create_line(border, width=10, fill='red')
        border = [[mapSize[0], 0], [0,0]]
        self.canvas.create_line(border, width=10, fill='red')



        self.obsList = []

        self.tk.update()


    def update_map(self,distance,angle):
        self.canvas.create_polygon(self.car.displayOrientation, outline='blue',fill='')

        self.car.moveTurn(angle)
        self.car.moveStraight(distance)


        self.canvas.delete(self.carImg)
        self.carImg = self.canvas.create_polygon(self.car.displayOrientation, fill='blue')
        self.canvas.delete(self.headImg)
        self.headImg = self.canvas.create_oval(self.car.displayPosition[0] - 10, self.car.displayPosition[1] - 10,
                                               self.car.displayPosition[0] + 10, self.car.displayPosition[1] + 10, fill='black')


        self.tk.update()


    def drawing(self,distance,angle,fps):
        for i in range(fps):
            self.update_map(float(distance)/float(fps), float(angle)/float(fps))
            time.sleep(0.1/float(fps))

    def drawObs(self,position):
        displayPos = []
        for i in position:
            i[1] = self.mapSizeY - i[1]
            displayPos.append(i)
        obs = self.canvas.create_polygon(displayPos,fill= 'red')
        self.obsList.append(obs)
        self.tk.update()
'''
mapSize = [600,600]
carInitPosition = [300,300,0]
tk = Tk()
canvas = Canvas(tk, width=mapSize[0], height=mapSize[1])
canvas.pack()
car = Car(carInitPosition[0],carInitPosition[1],carInitPosition[2])
bgImg = canvas.create_rectangle(0, 0, mapSize[0], mapSize[1], fill='white')  # draws background
carImg = canvas.create_polygon(car.orientation, fill='blue')
headImg = canvas.create_oval(car.head[0]-10, car.head[1]-10, car.head[0]+10, car.head[1]+10, fill='black')

tk.update()

    
car.moveTurn(50)
car.moveStraight(50)

print carImg
canvas.delete(carImg)
carImg = canvas.create_polygon(car.orientation, fill='blue')
canvas.delete(headImg)
headImg = canvas.create_oval(car.head[0] - 10, car.head[1] - 10,
                                      car.head[0] + 10, car.head[1] + 10, fill='black')
tk.update()
tk.mainloop()
'''