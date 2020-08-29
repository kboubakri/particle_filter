from utils import Position
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

    def move(self,orientation):
        self._v = self._delta_v if orientation else -self._delta_v
        self._position.predict_position(self._v,self._w)
        self._noisy_position.predict_position(self._v,self._w,self._std_actuators)

    def change_orientation(self,orientation):
        delta_w = self._delta_w if orientation else -self._delta_w
        self._w = self._w + delta_w

class DistanceSensor:
    def __init__(self,robot,landmarks_list,std_sens):
        self._std_sens = std_sens
        self._robot = robot
        self._landmarks_list = landmarks_list

    def compute_noisy_distance(self):
        noisy_distances = []
        for landmark in range(len(self._landmarks_list._X)):
            dx = self._robot._noisy_position._x - self._landmarks_list._X[landmark]
            dy = self._robot._noisy_position._y - self._landmarks_list._Y[landmark]
            noisy_distances.append(sqrt(dx**2+dy**2))
        return noisy_distances
