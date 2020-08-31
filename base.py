#!/usr/bin/python2.7
from robot import Robot,DistanceSensor
from particleFilter import Particle, ParticleFilter

from random import randint
import matplotlib.pyplot as plt
import keyboard

DIM_OF_WORLD = [20,20]
NB_OF_LANDMARKS = 10
NB_OF_PARTICLES = 50
STD_ACTUATORS = [0.1,0.1]
STD_SENSOR = 0.2

class StaticLandmarks:
    """
        Define a list of static landmarks using their (x,y) coordinates
        The landmarks are not meant to be moved
    """
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
        self._particles = self._ax.scatter(x_array,y_array,c='blue',label="particles")
        plt.scatter(landmarks._X,landmarks._Y,c='orange',s=50,marker='D',label="Static landmarks")
        self._robot = self._ax.scatter(robot_pos._x,robot_pos._y,c='purple',marker='X',label="Robot")
        self._ax.legend()

    def initialize_plot(self):
         """ Initialize the subplot for the GI """
         fig = plt.figure()
         self._ax = fig.add_subplot(111)
         self._ax.set_title('Application of the particle filter for an unicycle robot')
         self._ax.set_xlim([0,DIM_OF_WORLD[0]])
         self._ax.set_ylim([0,DIM_OF_WORLD[1]])

    def update_GI(self,ideal_pos,noisy_pos,particles):
         """
            Get the new ideal and noisy position of the robot and each particles
            and use it to update the plotting of the dead reckoning and ground truth
             trajectories as well as the particles' position
         """
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
         self._particles = self._ax.scatter(x_array,y_array,c='blue',label="particles")
         self._robot = self._ax.scatter(noisy_pos._x,noisy_pos._y,c='purple',marker='X',label="Robot")
         self._trajectory.figure.canvas.draw()
         self._dead_reckoning.figure.canvas.draw()
         plt.draw()


class Base:
    def __init__(self):
        self._has_move = False # Check if left or right arrow has been pressed
        # Class' instancies
        self._static_landmarks = StaticLandmarks([randint(1,DIM_OF_WORLD[0]-1) for i in range(NB_OF_LANDMARKS)],[randint(1,DIM_OF_WORLD[1]-1) for i in range(NB_OF_LANDMARKS)])
        self._robot = Robot(randint(1,DIM_OF_WORLD[0]-1),randint(1,DIM_OF_WORLD[1]-1),0,STD_ACTUATORS)
        self._distance_sensor = DistanceSensor(self._robot,self._static_landmarks,STD_SENSOR)
        self._particle_filter = ParticleFilter(self._robot._noisy_position, NB_OF_PARTICLES)
        self._GI_handler = GraphicalInterfaceHandler(self._robot._position,self._static_landmarks,self._particle_filter._particles)
        # Make the Gi event reactive
        self.cid = self._GI_handler._trajectory.figure.canvas.mpl_connect('key_press_event', self)

    # Event sensitive function, called when a key is pressed
    def __call__(self,event):
        """
            Event sensitve function. Called anytime a key is pressed
            Used to call the filter's function, update the robot's positions
            and to plot the new values in the GI
        """
        self.check_key()    # Update the robot position w/respect to the pressed key
        if self._has_move == True:
            # Update sensor info (with noise)
            noisy_dist_function = self._distance_sensor.compute_noisy_distance()
            # Update the particles'position upon actuator state
            self._particle_filter.prediction(self._robot._v,self._robot._w,STD_ACTUATORS)
            # Compute particles'weight
            self._particle_filter.update_weights(noisy_dist_function,self._static_landmarks,STD_SENSOR)
            # Resample particules
            self._particle_filter.resample_particles()
            # Estimate position (barycenter)
            estimated_positon,covariance = self._particle_filter.estimate_position()
            # Print output
            # self.print_out(self._robot._noisy_position,estimated_positon,covariance)
            # Update the graphical interface
            self._GI_handler.update_GI(self._robot._position,self._robot._noisy_position,self._particle_filter._particles)
            self._has_move = False

    def check_key(self):
        """
            Check if one of the arrows key has been pressed and update the robot's
            position accordingly
        """
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

    def print_out(self,real_pos,estimated_pos,covariance):
        print("\nrealistic robot position :")
        real_pos.print_pos()
        print("estimated position :")
        estimated_pos.print_pos()
        print("with covariance = " + str(covariance))

    def run(self):
        """
            Loop until 'q' is pressed.
            The __call__ function is dealing with the event handling
        """
        while not keyboard.is_pressed('q'):
            plt.show()
            plt.close()


if __name__ == "__main__":
    # execute only if run as a script
    world_base = Base()
    world_base.run()
