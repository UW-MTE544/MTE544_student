import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensors, kalmanFilter

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, SPIRAL_4TUNE, planner
from controller import controller, trajectoryController


class decision_maker(Node):
    
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_profile=qos_publisher)
        
        
        publishing_period=1/rate

        self.reachThreshold=0.01


        # TODO Part 3: use the Kalman Filter
        self.localizer=localization(...)
        
        if motion_type==POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)      
            self.planner=planner(POINT_PLANNER)

        
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)      
            self.planner=planner(TRAJECTORY_PLANNER)
        elif motion_type==SPIRAL_4TUNE:
            self.controller=None
            self.planner=None
            self.linearVelocity = 0
        
        else:
            print("Error! you don't have this type of planner", file=sys.stderr)
            return

        self.motion_type=motion_type
        self.goal=self.planner.plan() if self.motion_type != SPIRAL_4TUNE else None
        

        self.create_timer(publishing_period, self.timerCallback)



    def timerCallback(self):
        
        spin_once(self.localizer)

        if self.localizer.getPose() is  None:
            print("waiting for odom msgs ....")
            return
        
        
        vel_msg=Twist()

        if self.motion_type == SPIRAL_4TUNE:
            self.linearVelocity += 0.01 if self.linearVelocity < 1.0 else 0.0
            vel_msg.linear.x=self.linearVelocity
            vel_msg.angular.z=1.0
            self.publisher.publish(vel_msg)
            return

        
        if type(self.goal) == list:
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal[-1]) <self.reachThreshold else False
        else: 
            reached_goal=True if calculate_linear_error(self.localizer.getPose(), self.goal) <self.reachThreshold else False


        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            raise SystemExit
        
        velocity, yaw_rate = self.controller.\
            vel_request(self.localizer.getPose(), self.goal, True)

        
        vel_msg.linear.x=velocity
        vel_msg.angular.z=yaw_rate
        
        self.publisher.publish(vel_msg)


import argparse
def main(args=None):
    
    
    init()
    
    
    if args.motion == "point":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=POINT_PLANNER)
    elif args.motion == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=TRAJECTORY_PLANNER)
    elif args.motion == "spiral":
        DM=decision_maker(Twist, "/cmd_vel", 10, motion_type=SPIRAL_4TUNE)
    else:
        print("invalid motion type", file=sys.stderr)
        return



    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")




if __name__=="__main__":
    
    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="spiral")
    args = argParser.parse_args()

    main(args)
