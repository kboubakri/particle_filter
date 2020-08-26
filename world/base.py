#!/usr/bin/python2.7

from random import randint
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import keyboard
from numpy import cos,sin
import time

PERIOD = 0.2 # in s

class Robot:
    def __init__(self,x,y,theta):
        self._x = x
        self._y = y
        self._theta = theta
        self._delta_w = 0.1
        self._delta_v = 1
        self._w = 0

    def inverse_transform(self,v):
        self._theta = self._w
        self._x = self._x + v*cos(self._theta)*PERIOD
        self._y = self._y + v*sin(self._theta)*PERIOD

    def move(self,orientation):
        v = self._delta_v if orientation else -self._delta_v
        self.inverse_transform(v)
        # print(self._x)
        # print(self._y)
        # print(self._theta)

    def change_orientation(self,orientation):
        delta_w = self._delta_w if orientation else -self._delta_w
        self._w = self._w + delta_w
        print(self._w)

class DistanceSensor:
    def __init__(self,pos,landmarks_list):
        self._x = pos._x
        self._y = pos._y
        # self._theta = pos._theta
        self._landmarks_list = landmarks_list

    def compute_distances(self):
        dx = []
        dy = []
        for i in range(len(self._landmarks_list._x)):
            dx.append(self._x - self._landmarks_list._x[i])
            dy.append(self._y - self._landmarks_list._y[i])
        return (dx,dy)

    # Define as utils
    def compute_pos_in_world(self,dx,dy):
        dx_w = []
        dy_w = []
        for xp,yp in dx,dy:
            dx_w.append(xp*cos(0)-yp*sin(0)+self._x)
            dx_w.append(xp*sin(0)+yp*cos(0)+self._x)


class StaticLandmarks:
    def __init__(self,x,y):
        self.update_position(x,y)


    def update_position(self,x,y):
        self._x = x
        self._y = y

class GraphicalInterface:
    def __init__(self,robot,landmarks):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title('Application of the PF on unycle robot model')
        self._trajectory, = ax.plot([0], [0],'r',label='Ground truth')  # empty line
        ax.legend()
        # self._trajectory2, = ax.plot([0], [0])  # empty line
        self._robot = robot
        self._xtraj = [self._robot._x]
        self._ytraj = [self._robot._y]
        # self._xtraj2 = [self._robot._x+1]
        # self._ytraj2 = [self._robot._y+1]
        self._landmarks = landmarks
        self.cid = self._trajectory.figure.canvas.mpl_connect('key_press_event', self)
        plt.scatter(self._landmarks._x,self._landmarks._y,s=50)

    def __call__(self,event):
        if keyboard.is_pressed('right'):
            print("right")
            self._robot.move(1)
        if keyboard.is_pressed('left'):
            print("left")
            self._robot.move(0)
        if keyboard.is_pressed('up'):
            print("up")
            self._robot.change_orientation(1)
        if keyboard.is_pressed('down'):
            print("down")
            self._robot.change_orientation(0)
        self._xtraj.append(self._robot._x)
        self._ytraj.append(self._robot._y)
        # self._xtraj2.append(self._robot._x+1)
        # self._ytraj2.append(self._robot._y+1)
        self._trajectory.set_data(self._xtraj,self._ytraj)
        # self._trajectory2.set_data(self._xtraj2,self._ytraj2)
        self._trajectory.figure.canvas.draw()
        # self._trajectory2.figure.canvas.draw()
        plt.draw()


class Base:
    def __init__(self):
        self._static_landmarks = StaticLandmarks([randint(0,20) for i in range(5)],[randint(0,20) for i in range(5)])
        self._robot = Robot(randint(0,20),randint(0,20),0)
        self._graphical_interface = GraphicalInterface(self._robot,self._static_landmarks)
        self._distance_sensor = DistanceSensor(self._robot,self._static_landmarks)
        self._distance_sensor.compute_distances()

    def run(self):
        while not keyboard.is_pressed('q'):
            plt.show()
            plt.close()


if __name__ == "__main__":
    # execute only if run as a script
    world_base = Base()
    world_base.run()
