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

    def update_weights(self,noisy_dist_function,landmarks,std_sensor):
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
                particle._weight *= stats.norm(distance,std_sensor).pdf(noisy_dist_function[landmark])
            particle._weight += 1.e-300 # avoid round-off to zero

    # def stratified_resample(self,weights):
    #     N = self._nb_of_particles
    #     # Positions in the weight's spectre
    #     positions = (np.random.random(N) + range(N))/N
    #     indexes = np.zeros(N,'i')
    #     cumulative_sum = np.cumsum(weights)
    #     i,j = 0,0
    #     while i < N:
    #         if positions[i] < cumulative_sum[j]:
    #             indexes[i] = j
    #             i += 1
    #         else:
    #             j += 1
    #     return indexes

    def systematic_resample(self,weights):
        N = self._nb_of_particles
        positions = (np.arange(N) + np.random.random())/N
        indexes = np.zeros(N,'i')
        cumulative_sum = np.cumsum(weights)
        i, j = 0, 0
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
        # chosen_indexes = self.stratified_resample(weights)
        chosen_indexes = self.systematic_resample(weights)

        # Resample according to indexes
        old_particles_list = self._particles
        for i in range(self._nb_of_particles):
            self._particles[i]._position = deepcopy(old_particles_list[chosen_indexes[i]]._position)

    def estimate_position(self):
        """
            Compute the barycenter of all the particles
        """
        N = self._nb_of_particles

        x_array = [p._position._x for p in self._particles]
        y_array = [p._position._y for p in self._particles]

        covariance = np.cov(x_array,y_array)

        return(Position(np.sum(x_array)/N,np.sum(y_array)/N,0),covariance)
