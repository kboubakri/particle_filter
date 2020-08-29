from utils import Position
from random import randint
import numpy as np
from scipy import stats
from copy import deepcopy

class Particle:
    def __init__(self,position,weight):
        self._position = position
        self._weight = weight

# Inspired of the following SIR implementation :
# https://share.cocalc.com/share/7557a5ac1c870f1ec8f01271959b16b49df9d087/Kalman-and-Bayesian-Filters-in-Python/12-Particle-Filters.ipynb?viewer=share
class ParticleFilter:
    def __init__(self,robot_pos,nb_of_particles):
        self._nb_of_particles = nb_of_particles # This value is not expected to be changed during the filtering
        self._particles = [Particle(Position(randint(0,20),randint(0,20),0),0) for i in range(self._nb_of_particles)] # Random initialization
        # self._particles = [Particle(Position(robot_pos._x,robot_pos._y,0),1) for i in range(NB_OF_PARTICLES)] # First position initialization

    def get_particles_weight(self):
        return [p._weight for p in self._particles]

    def set_particles_weight(self,new_weights):
        for i in range(self._nb_of_particles):
            self._particles[i]._weight = new_weights[i]

    def prediction(self,v,w,std_actuators):
        for particle in self._particles:
            particle._position.predict_position(v,w,std_actuators)

    def update_weights(self,noisy_dist_function,landmarks):
        measurement_uncertainty = [0.1,0.1] # in x and y
        sum_weights = 0.0

        for particle in self._particles:
            particle._weight = 1.0
            for landmark in range(len(landmarks._X)):
                # Compute the [dx,dy] between the real position of the landmark
                # And the estimation according to the position of the particle
                # Magic sensor --> the landmark is identified and all landmarks seen
                dx = particle._position._x - landmarks._X[landmark]
                dy = particle._position._y - landmarks._Y[landmark]
                distance = np.sqrt(dx**2+dy**2)
                particle._weight *= stats.norm(distance,0.2).pdf(noisy_dist_function[landmark])
            particle._weight += 1.e-300 # avoid round-off to zero

    def stratified_resample(self,weights):
        N = self._nb_of_particles
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

        # Stratified resampling
        chosen_indexes = self.stratified_resample(weights)

        # Resample according to indexes
        old_particles_list = self._particles
        for i in range(self._nb_of_particles):
            self._particles[i]._position = deepcopy(old_particles_list[chosen_indexes[i]]._position)

    def estimate_position(self):
        sum_x = 0.0
        sum_y = 0.0
        N = self._nb_of_particles
        for p in range(N):
            sum_x += self._particles[p]._position._x
            sum_y += self._particles[p]._position._y
        return(Position(sum_x/N,sum_y/N,0))