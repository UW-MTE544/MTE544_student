import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    # def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2): PID5
    # def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=1.2, kai=0.2): PID6
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        # TODONE Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller (do in lab)

        #set chosen controller (PID), gains, and set file name for logging the errors
        self.PID_linear=PID_ctrl(PID, klp, klv, kli, filename_="pid_sigmoid_linear.csv")
        self.PID_angular=PID_ctrl(PID, kap, kav, kai, filename_="pid_sigmoid_angular.csv")

    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)


        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # TODONE Part 5: Add saturation limits for the robot linear and angular velocity
        # Values selected from turtlebot 4 documentation 
        # may need to be adjusted if negative values are used for the linear and angular velocity.

        #limited linear velocity to be between 0 and 0.10 m/s, and angular velocity to be between -1.0 and 1.0 rad/s to ease tuning
        linear_vel = float(np.clip(linear_vel, 0.0, 0.10))
        angular_vel = float(np.clip(angular_vel, -1.0, 1.0))
        
        # Slow forward motion while heading away from the target.
        linear_vel *= max(0.0, float(np.cos(e_ang)))
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.6, klv=0.3, kli=0.3, kap=1.2, kav=0.3, kai=0.2):
        
        super().__init__(klp, klv, kli, kap, kav, kai)
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # TODONE Part 5: Add saturation limits for the robot linear and angular velocity
        # Values selected from turtlebot 4 documentation 
        linear_vel = float(np.clip(linear_vel, 0.0, 0.10))
        angular_vel = float(np.clip(angular_vel, -1.0, 1.0))
        
        # Slow forward motion while heading away from the target.
        linear_vel *= max(0.0, float(np.cos(e_ang)))
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
