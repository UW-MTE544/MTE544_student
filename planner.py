import numpy as np
#  Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_):

        self.type=type_


    def plan(self, goalPoint=[-1.0, -1.0]):

        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)

        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]

        # parabola 
        x = np.linspace(0, 1.5, 15)
        y = x ** 2

        # sigmoid function
        # x = np.linspace(0, 2.5, 25)
        # y = 2 / (1 + np.exp(-2 * x)) - 1

        return [[x_val, y_val] for x_val, y_val in zip(x, y)]


