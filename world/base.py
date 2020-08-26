#!/usr/bin/python2.7

from random import randint
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import keyboard
from numpy import cos,sin
import time

PERIOD = 0.2 # in s
DIM_OF_WORLD = [20,20]
NB_OF_LANDMARKS = 2
NB_OF_PARTICLES = 50

class Position:
    def __init__(self,x,y,theta):
        self._x = x
        self._y = y
        self._theta = theta

    def predict_position(self,v,w):
        self._theta = w
        self._x = self._x + v *cos(self._theta)*PERIOD
        self._y = self._y + v *sin(self._theta)*PERIOD

    def add_noise(self,noise):
        self._x += noise
        self._y +=  noise
        self._theta += noise

    def compute_noisy_position(self,mean,standard_deviation):
        s = np.random.normal(mean,standard_deviation)
        self.add_noise(s)

    def print_pos(self):
        print("x: "+str(self._x) +" y: "+str(self._y))

class Robot:
    def __init__(self,x,y,theta):
        self._position = Position(x,y,theta)
        self._noisy_position = Position(x,y,theta)
        self._delta_w = 0.1
        self._delta_v = 1
        self._w = 0
        self._v = 0

    def move(self,orientation):
        self._v = self._delta_v if orientation else -self._delta_v
        self._position.predict_position(self._v,self._w)
        self._noisy_position.predict_position(self._v,self._w)
        self._noisy_position.compute_noisy_position(0,0.01)

    def change_orientation(self,orientation):
        delta_w = self._delta_w if orientation else -self._delta_w
        self._w = self._w + delta_w

class DistanceSensor:
    def __init__(self,robot,landmarks_list):
        self._robot = robot
        # self._theta = pos._theta
        self._landmarks_list = landmarks_list

    def compute_distances(self):
        dx = []
        dy = []
        for i in range(len(self._landmarks_list._X)):
            dx.append(self._robot._noisy_position._x - self._landmarks_list._X[i])
            dy.append(self._robot._noisy_position._y - self._landmarks_list._Y[i])
        return (dx,dy)

class Particle:
    def __init__(self,position,weight):
        self._position = position
        self._weight = weight

class ParticleFilter:
    def __init__(self,robot_pos):
        # self._particles = [Particle(Position(randint(0,20),randint(0,20),0),0) for i in range(NB_OF_PARTICLES)]
        self._particles = [Particle(Position(robot_pos._x,robot_pos._y,0),1) for i in range(NB_OF_PARTICLES)]

    def get_particles_x(self):
        x_array = []
        for p in self._particles:
            x_array.append(p._position._x)
            print(p._position._x)
        return x_array

    def get_particles_y(self):
        y_array = []
        for p in self._particles:
            y_array.append(p._position._y)
            print(p._position._y)
        return y_array

    def get_particles_weight(self):
        return [p._weight for p in self._particles]

    def prediction(self,v,w):
        for particle in self._particles:
            particle._position.predict_position(v,w)
            particle._position.compute_noisy_position(0,0.001)

    def update_weights(self,dx_to_landmarks,dy_to_landmarks,landmarks):
        measurement_uncertainty = [0.1,0.1] # in x and y
        sum_weights = 0.0

        for particle in self._particles:
            particle._weight = 1.0
            for landmark in range(len(dx_to_landmarks)):
                # Compute the [dx,dy] between the real position of the landmark
                # And the estimation according to the position of the particle
                # Magic sensor --> the landmark is identified
                x_landmark_world = particle._position._x - dx_to_landmarks[landmark]
                y_landmark_world = particle._position._y - dy_to_landmarks[landmark]
                estimated_pos_landmark_world = Position(x_landmark_world,y_landmark_world,0)
                estimated_pos_landmark_world.print_pos()
                print("landmark = " + str(landmarks._X[landmark]) + " , " + str(landmarks._Y[landmark]))
                dx = landmarks._X[landmark] - estimated_pos_landmark_world._x
                print("dx " +str(dx))
                dy = landmarks._Y[landmark] - estimated_pos_landmark_world._y
                w = np.exp(-0.5*((dx/10)**2 + (dy/10)**2))/2*np.pi*measurement_uncertainty[0]*measurement_uncertainty[1]
                particle._weight = particle._weight*w
                sum_weights += particle._weight

    def resample_particles(self):
        # Normalize weights
        weights = self.get_particles_weight()
        weights = weights/np.sum(weights)
        self._particles = np.random.choice(self._particles,NB_OF_PARTICLES,p=weights)

class StaticLandmarks:
    def __init__(self,x,y):
        self._X = x
        self._Y = y

class Base:
    def __init__(self):
        self._static_landmarks = StaticLandmarks([randint(0,DIM_OF_WORLD[0]) for i in range(NB_OF_LANDMARKS)],[randint(0,DIM_OF_WORLD[1]) for i in range(NB_OF_LANDMARKS)])
        self._robot = Robot(randint(0,DIM_OF_WORLD[0]),randint(0,DIM_OF_WORLD[1]),0)
        self._distance_sensor = DistanceSensor(self._robot,self._static_landmarks)
        self._particle_filter = ParticleFilter(self._robot._noisy_position)
        self._has_move = False
        self.initialize_plot()

    def initialize_plot(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title('Application of the PF on unycle robot model')
        self._trajectory, = ax.plot([0], [0],'r',label='Ground truth')  # empty line
        self._dead_reckoning, = ax.plot([0], [0],'g',label='Dead reckoning')  # empty line
        x_array = [p._position._x for p in self._particle_filter._particles]
        y_array = [p._position._y for p in self._particle_filter._particles]
        plt.scatter(x_array,y_array,label="particles")
        self.cid = self._trajectory.figure.canvas.mpl_connect('key_press_event', self)
        plt.scatter(self._static_landmarks._X,self._static_landmarks._Y,s=50,label="Static landmarks")
        self._xtraj = [self._robot._position._x]
        self._ytraj = [self._robot._position._y]
        self._dead_reck_x = [self._robot._noisy_position._x]
        self._dead_reck_y = [self._robot._noisy_position._y]
        ax.legend()

    def __call__(self,event):
        self.check_key()
        if self._has_move == True:
            # Update sensor info
            dx_to_landmarks, dy_to_landmarks = self._distance_sensor.compute_distances()
            # Update the particles'position upon actuator state
            self._particle_filter.prediction(self._robot._v,self._robot._w)
            # Compute particles'weight
            self._particle_filter.update_weights(dx_to_landmarks,dy_to_landmarks,self._static_landmarks)
            # Resample
            self._particle_filter.resample_particles()
        # Update the graphical interface
        self.update_GI()
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

    def update_GI(self):
        self._xtraj.append(self._robot._position._x)
        self._ytraj.append(self._robot._position._y)
        self._dead_reck_x.append(self._robot._noisy_position._x)
        self._dead_reck_y.append(self._robot._noisy_position._y)
        self._trajectory.set_data(self._xtraj,self._ytraj)
        self._dead_reckoning.set_data(self._dead_reck_x,self._dead_reck_y)
        x_array = [p._position._x for p in self._particle_filter._particles]
        y_array = [p._position._y for p in self._particle_filter._particles]
        plt.scatter(x_array,y_array,label="particles")
        self._trajectory.figure.canvas.draw()
        self._dead_reckoning.figure.canvas.draw()
        plt.draw()


    def run(self):
        while not keyboard.is_pressed('q'):
            plt.show()
            plt.close()


if __name__ == "__main__":
    # execute only if run as a script
    world_base = Base()
    world_base.run()
