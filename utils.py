import numpy as np
from numpy import cos,sin

PERIOD = 1 # in s

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
