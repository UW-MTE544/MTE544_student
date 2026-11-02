from math import sin,cos, atan2,atan
import math

POINT_PLANNER=0; TRAJECTORY_PLANNER=1; SPIRAL_4TUNE=2

PARABOLA=0; SIGMOID=1

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self):
        
        if self.type==POINT_PLANNER:
            return self.point_planner()
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self):
        x=-1.0; y=-1.0; theta=0.0
        return x, y, theta


    def trajectory_planner(self):
        TRAJECTORY_TYPE=SIGMOID
        degree_rad_conversion=3.14/180.0
        if TRAJECTORY_TYPE == PARABOLA:
            
            path = [[ (x/10.0) ,(x/10.0)**2] for x in range(0,20)]
            # rotate the path by theta degrees
            theta = 60.0 * degree_rad_conversion
            return [[x*cos(theta) - y*sin(theta),
                    x*sin(theta)  + y*cos(theta)] for x,y in path]
        else:
            path = [[ -(x/10.0) , -1/( 1 + math.exp(-(x/10)))] for x in range(0,30)]
            # rotate the path by theta degrees
            theta = 60.0 * degree_rad_conversion
            return [[x*cos(theta) - y*sin(theta),
                    x*sin(theta)  + y*cos(theta)] for x,y in path]
