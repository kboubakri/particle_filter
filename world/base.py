#!/usr/bin/python2.7

from random import randint
from utils import Position,Particle
from robot import Robot,DistanceSensor
from particleFilter import Particle, ParticleFilter
from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import keyboard
from numpy import cos,sin
# import time

PERIOD = 1 # in s
DIM_OF_WORLD = [20,20]
NB_OF_LANDMARKS = 10
NB_OF_PARTICLES = 50

class StaticLandmarks:
    # Define at the beginning, they are not supposed to be moved
    def __init__(self,x,y):
        self._X = x
        self._Y = y

class GraphicalInterfaceHandler:
    def __init__(self,robot_pos,landmarks,particles):
        # Plot initialization
        self.initialize_plot()
        # Memorized variables
        self._xtraj,self._ytraj = [robot_pos._x],[robot_pos._y]
        self._dead_reck_x,self._dead_reck_y = [robot_pos._x],[robot_pos._y]
        x_array = [p._position._x for p in particles]
        y_array = [p._position._y for p in particles]
        # Plotted elements
        self._trajectory, = self._ax.plot([0], [0],'r',label='Ground truth')
        self._dead_reckoning, = self._ax.plot([0], [0],'g',label='Dead reckoning')
        self._particles = self._ax.scatter(x_array,y_array,label="particles")
        plt.scatter(landmarks._X,landmarks._Y,s=50,marker='D',label="Static landmarks")
        self._robot = self._ax.scatter(robot_pos._x,robot_pos._y,marker='X',label="Robot")
        self._ax.legend()

    def initialize_plot(self):
         fig = plt.figure()
         self._ax = fig.add_subplot(111)
         self._ax.set_title('Application of the PF on unycle robot model')
         self._ax.set_xlim([0,DIM_OF_WORLD[0]])
         self._ax.set_ylim([0,DIM_OF_WORLD[1]])

    def update_GI(self,ideal_pos,noisy_pos,particles):
         # Update memorized values
         self._xtraj.append(ideal_pos._x)
         self._ytraj.append(ideal_pos._y)
         self._dead_reck_x.append(noisy_pos._x)
         self._dead_reck_y.append(noisy_pos._y)
         x_array = [p._position._x for p in particles]
         y_array = [p._position._y for p in particles]
         # Set values
         self._trajectory.set_data(self._xtraj,self._ytraj)
         self._dead_reckoning.set_data(self._dead_reck_x,self._dead_reck_y)
         self._particles.remove()
         self._robot.remove()
         # Draw
         self._particles = self._ax.scatter(x_array,y_array,label="particles")
         self._robot = self._ax.scatter(noisy_pos._x,noisy_pos._y,marker='X',label="Robot")
         self._trajectory.figure.canvas.draw()
         self._dead_reckoning.figure.canvas.draw()
         plt.draw()

class Base:
    def __init__(self):
        self._static_landmarks = StaticLandmarks([randint(0,DIM_OF_WORLD[0]) for i in range(NB_OF_LANDMARKS)],[randint(0,DIM_OF_WORLD[1]) for i in range(NB_OF_LANDMARKS)])
        self._robot = Robot(randint(0,DIM_OF_WORLD[0]),randint(0,DIM_OF_WORLD[1]),0)
        self._distance_sensor = DistanceSensor(self._robot,self._static_landmarks,0.1)
        self._particle_filter = ParticleFilter(self._robot._noisy_position)
        self._GI_handler = GraphicalInterfaceHandler(self._robot._position,self._static_landmarks,self._particle_filter._particles)
        self._has_move = False
        # Make the Gi event reactive
        self.cid = self._GI_handler._trajectory.figure.canvas.mpl_connect('key_press_event', self)

    def __call__(self,event):
        self.check_key()
        if self._has_move == True:
            # Update sensor info
            noisy_dist_function = self._distance_sensor.compute_noisy_distance()
            # Update the particles'position upon actuator state
            self._particle_filter.prediction(self._robot._v,self._robot._w)
            # Compute particles'weight
            self._particle_filter.update_weights(noisy_dist_function,self._static_landmarks)
            # Resample
            self._particle_filter.resample_particles()
        # Update the graphical interface
        self._GI_handler.update_GI(self._robot._position,self._robot._noisy_position,self._particle_filter._particles)
        self._has_move = False

    def check_key(self):
        if keyboard.is_pressed('right'):
            self._robot.move(1)
            self._has_move = True
        if keyboard.is_pressed('left'):
            self._has_move = True
            self._robot.move(0)
        if keyboard.is_pressed('up'):
            self._robot.change_orientation(1)
        if keyboard.is_pressed('down'):
            self._robot.change_orientation(0)

    def run(self):
        while not keyboard.is_pressed('q'):
            plt.show()
            plt.close()


if __name__ == "__main__":
    # execute only if run as a script
    world_base = Base()
    world_base.run()
