from utils import Position
import numpy as np
from numpy import sqrt

class Robot:
    def __init__(self,x,y,theta,std_actuators):
        self._std_actuators = std_actuators
        self._position = Position(x,y,theta)
        self._noisy_position = Position(x,y,theta)
        self._delta_w = .3
        self._delta_v = 1
        self._w = 0
        self._v = 0

    def move(self,direction):
        """
            Get the robot's direction from key arrows and move the robot accordingly
        """
        self._v = self._delta_v if direction else -self._delta_v
        self._position.predict_position(self._v,self._w)
        self._noisy_position.predict_position(self._v,self._w,self._std_actuators)

    def change_orientation(self,orientation):
        """
            Get the robot's angular velocity from key arrows and orient the robot accordingly
        """
        delta_w = self._delta_w if orientation else -self._delta_w
        self._w = self._w + delta_w

class DistanceSensor:
    def __init__(self,robot,landmarks_list,std_sens):
        self._std_sens = std_sens   # Same error along x and y axis
        self._robot = robot
        self._landmarks_list = landmarks_list

    def compute_noisy_distance(self):
        """
            Compute the distance between the robot and each landmark
            Mimic
        """
        noisy_distances = []
        for landmark in range(len(self._landmarks_list._X)):
            dx = self._robot._noisy_position._x - self._landmarks_list._X[landmark] + np.random.randn()*self._std_sens
            dy = self._robot._noisy_position._y - self._landmarks_list._Y[landmark] + np.random.randn()*self._std_sens
            noisy_distances.append(sqrt(dx**2+dy**2))
        return noisy_distances
