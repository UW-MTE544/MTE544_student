# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

import numpy as np


class planner:
    def __init__(self, type_, trajectory="parabola"):

        self.type=type_
        self.trajectory=trajectory

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    def trajectory_planner(self):
        #if the trajectory is parabola, return a list of points along the parabola from (0,0) to (1,1) note the parabola was sized down to fit in the room
        if self.trajectory == "parabola":
            x_values = np.linspace(0.0, 1, 51)
            return [[float(x), float(x**2)] for x in x_values]
        #if the trajectory is sigmoid, return a list of points along the sigmoid from (0,0) to (2.5,1)
        elif self.trajectory == "sigmoid":
            x_values = np.linspace(0.0, 2.5, 51)
            return [[float(x), float(2 / (1 + np.exp(-2 * x)) - 1)] for x in x_values]
        #if the input trajectory is not recognized, raise an error
        raise ValueError("Unknown trajectory type: {}".format(self.trajectory))

