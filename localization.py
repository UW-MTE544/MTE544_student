import sys
import time

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

USE_SIM = False

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # TODONE Part 3: Define the QoS profile for the simulation (Turtlebot 3 Burger).
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        if USE_SIM:
            # Match the simulator's /odom publisher and keep the latest message.
            odom_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        else:
            # Robot 11 advertises BEST_EFFORT / VOLATILE odometry.
            odom_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        self.last_received = None
        
        if localizationType == rawSensor:
        # TODONE Part 3: subscribe to the position sensor topic (Odometry)
        #incoming /odom messages invoke the odom_callback function to update the robot's pose
            self.create_subscription(odom, "/odom", self.odom_callback, odom_qos)
        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg):
        # TODONE Part 3: Read x,y, theta, and record the stamp
        #pose = [x, y, yaw, ROS timestamp], yaw extracted from the quaternion in the pose_msg
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion([pose_msg.pose.pose.orientation.x,
                                           pose_msg.pose.pose.orientation.y,
                                           pose_msg.pose.pose.orientation.z,
                                           pose_msg.pose.pose.orientation.w]),
                    pose_msg.header.stamp ]
        
        self.last_received = time.monotonic()

        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])
    
    def getPose(self):
        return self.pose

# TODONE Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py

if __name__ == "__main__":
    init()
    node=localization()
    spin(node)
    