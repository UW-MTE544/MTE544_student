import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin, shutdown

rawSensor = 0
class localization(Node):

    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")

        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3



        odom_qos= QoSProfile(depth = 10)

        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None

        if localizationType == rawSensor:
             self.odom_subscriber = self.create_subscription(odom, "/odom", self.odom_callback, odom_qos)
        # TODO Part 3: subscribe to the position sensor topic (Odometry)
        # ...

        else:
            print("This type doesn't exist", sys.stderr)


    def odom_callback(self, pose_msg):

        # TODO Part 3: Read x,y, theta, and record the stamp

        self.odom_initialized = True

        ## this was taken from our LAB 1 code
        x_pos = pose_msg.pose.pose.position.x
        y_pos = pose_msg.pose.pose.position.y

        orientation = pose_msg.pose.pose.orientation

        quat_values = [orientation.x, orientation.y, orientation.z, orientation.w]

        yaw = euler_from_quaternion(quat_values)

        stamp = Time.from_msg(pose_msg.header.stamp).nanoseconds


        self.odom_logger.log_values ([x_pos, y_pos, yaw, stamp])
        self.pose=[x_pos, y_pos, yaw, stamp]


        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])

    def getPose(self):
        return self.pose

# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py

if __name__ == "__main__":
    init()
    node = localization()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()



