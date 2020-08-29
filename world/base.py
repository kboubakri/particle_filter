#!/usr/bin/python2.7

from random import randint
from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
from matplotlib.animation import FuncAnimation
import keyboard
from numpy import cos,sin
import time

PERIOD = 1 # in s
DIM_OF_WORLD = [20,20]
NB_OF_LANDMARKS = 10
NB_OF_PARTICLES = 50

class Position:
    def __init__(self,x,y,theta):
        self._x = x
        self._y = y
        self._theta = theta

    def predict_position(self,v,w,speeds_std=[0,0]):
        # Update heading
        self._theta = w*PERIOD + np.random.randn()*speeds_std[1]
        self._theta %= 2*np.pi
        #Update position
        dist = v*PERIOD + np.random.randn()*speeds_std[0]
        self._x += dist *cos(self._theta)
        self._y += dist *sin(self._theta)

    def print_pos(self):
        print("x: "+str(self._x) +" y: "+str(self._y))

class Robot:
    def __init__(self,x,y,theta):
        self._position = Position(x,y,theta)
        self._noisy_position = Position(x,y,theta)
        self._delta_w = 1
        self._delta_v = 1
        self._w = 0
        self._v = 0

    def move(self,orientation):
        self._v = self._delta_v if orientation else -self._delta_v
        self._position.predict_position(self._v,self._w)
        self._noisy_position.predict_position(self._v,self._w,[0.1,0.01])
        # self._noisy_position.compute_noisy_position(0,0.01)

    def change_orientation(self,orientation):
        delta_w = self._delta_w if orientation else -self._delta_w
        self._w = self._w + delta_w

class DistanceSensor:
    def __init__(self,robot,landmarks_list,std_sens):
        self._std_sens = std_sens
        self._robot = robot
        # self._theta = pos._theta
        self._landmarks_list = landmarks_list

    def compute_noisy_distance(self):
        noisy_distances = []
        for landmark in range(NB_OF_LANDMARKS):
            dx = self._robot._noisy_position._x - self._landmarks_list._X[landmark]
            dy = self._robot._noisy_position._y - self._landmarks_list._Y[landmark]
            noisy_distances.append(np.sqrt(dx**2+dy**2))
        return noisy_distances

class Particle:
    def __init__(self,position,weight):
        self._position = position
        self._weight = weight

class ParticleFilter:
    def __init__(self,robot_pos):
        self._particles = [Particle(Position(randint(0,20),randint(0,20),0),0) for i in range(NB_OF_PARTICLES)] # Random initialization
        # self._particles = [Particle(Position(robot_pos._x,robot_pos._y,0),1) for i in range(NB_OF_PARTICLES)] # First position initialization

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

    def set_particles_weight(self,new_weights):
        for i in range(NB_OF_PARTICLES):
            self._particles[i]._weight = new_weights[i]

    def prediction(self,v,w):
        i = 0
        for particle in self._particles:
            print("particle " +str(i)+" before ")
            particle._position.print_pos()
            particle._position.predict_position(v,w,[0.3,0.1])
            print("after ")
            particle._position.print_pos()
            i +=1

    def update_weights(self,noisy_dist_function,landmarks):
        measurement_uncertainty = [0.1,0.1] # in x and y
        sum_weights = 0.0

        for particle in self._particles:
            particle._weight = 1.0
            for landmark in range(NB_OF_LANDMARKS):
                # Compute the [dx,dy] between the real position of the landmark
                # And the estimation according to the position of the particle
                # Magic sensor --> the landmark is identified
                dx = particle._position._x - landmarks._X[landmark]
                dy = particle._position._y - landmarks._Y[landmark]
                distance = np.sqrt(dx**2+dy**2)
                particle._weight *= stats.norm(distance,0.2).pdf(noisy_dist_function[landmark])
            particle._weight += 1.e-300 # avoid round-off to zero

    def stratified_resample(self,weights):
        N = NB_OF_PARTICLES
        # Position in the weight's spectre
        positions = (np.random.random(N) + range(N))/N
        indexes = np.zeros(N,'i')
        cumulative_sum = np.cumsum(weights)
        i,j = 0,0
        while i < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        return indexes

    def resample_particles(self):
        # Normalize weights
        weights = self.get_particles_weight()
        weights = weights/np.sum(weights)

        print("weights = " + str(weights))

        # Stratified resampling
        chosen_indexes = self.stratified_resample(weights)

        # Resample according to indexes
        old_particles_list = self._particles
        for i in range(NB_OF_PARTICLES):
            self._particles[i]._position = deepcopy(old_particles_list[chosen_indexes[i]]._position)

    def estimate_position(self):
        sum_x = 0.0
        sum_y = 0.0
        for p in range(NB_OF_PARTICLES):
            sum_x += self._particles[p]._position._x
            sum_y += self._particles[p]._position._y
        return(Position(sum_x/NB_OF_PARTICLES,sum_y/NB_OF_PARTICLES,0))


class StaticLandmarks:
    # Define at the beginning, they are not supposed to be moved
    def __init__(self,x,y):
        self._X = x
        self._Y = y

class Base:
    def __init__(self):
        self._static_landmarks = StaticLandmarks([randint(0,DIM_OF_WORLD[0]) for i in range(NB_OF_LANDMARKS)],[randint(0,DIM_OF_WORLD[1]) for i in range(NB_OF_LANDMARKS)])
        self._robot = Robot(randint(0,DIM_OF_WORLD[0]),randint(0,DIM_OF_WORLD[1]),0)
        self._distance_sensor = DistanceSensor(self._robot,self._static_landmarks,0.1)
        self._particle_filter = ParticleFilter(self._robot._noisy_position)
        self._has_move = False
        self.initialize_plot()

    def initialize_plot(self):
        fig = plt.figure()
        self._ax = fig.add_subplot(111)
        self._ax.set_title('Application of the PF on unycle robot model')
        self._trajectory, = self._ax.plot([0], [0],'r',label='Ground truth')  # empty line
        self._dead_reckoning, = self._ax.plot([0], [0],'g',label='Dead reckoning')  # empty line
        self._ax.set_xlim([0,DIM_OF_WORLD[0]])
        self._ax.set_ylim([0,DIM_OF_WORLD[1]])
        x_array = [p._position._x for p in self._particle_filter._particles]
        y_array = [p._position._y for p in self._particle_filter._particles]
        self._ps = self._ax.scatter(x_array,y_array,label="particles")
        self.cid = self._trajectory.figure.canvas.mpl_connect('key_press_event', self)
        plt.scatter(self._static_landmarks._X,self._static_landmarks._Y,s=50,marker='D',label="Static landmarks")
        self._xtraj = [self._robot._position._x]
        self._ytraj = [self._robot._position._y]
        self._dead_reck_x = [self._robot._noisy_position._x]
        self._dead_reck_y = [self._robot._noisy_position._y]
        self._ax.legend()

    def __call__(self,event):
        self.check_key()
        if self._has_move == True:
            # Print positions
            print("\n\nrobot ")
            self._robot._position.print_pos()
            # Update sensor info
            noisy_dist_function = self._distance_sensor.compute_noisy_distance()
            # Update the particles'position upon actuator state
            self._particle_filter.prediction(self._robot._v,self._robot._w)
            # Compute particles'weight
            self._particle_filter.update_weights(noisy_dist_function,self._static_landmarks)
            # Resample
            self._particle_filter.resample_particles()
            # Estimate
            print("Estimate")
            self._particle_filter.estimate_position().print_pos()
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
        self._ps.remove()
        self._ps = self._ax.scatter(x_array,y_array,label="particles")
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
