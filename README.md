# LAB 3 - Localization

## Introduction

Welcome to LAB 3 of the mobile robotics course! 
In this lab, participants will gain experience with utilizing sensor data to perform localization via state estimation.

In this lab, you will implement the Extended Kalman Filter (EKF). 

By the end of this lab, participants will be able to:
- Use an Extended Kalman Filter to localize in real-time a mobile robot using sensor data.

*Part 1* and *Part 2* are the same as in the previous labs they are here just for your convenience.


#### The summary of what you should learn is as following:
- You will learn how to derive the needed functions and matrices and perform localization using an EKF by using IMU and wheel encoders (as odom).

**NOTE** this Lab builds on top of Lab 2. A complete solution to Lab 2 is provided within this lab so that even if you did not conclude Lab 2's implementation, you can still work on Lab 3. You are welcome to replace some of the code with your own development from Lab 2.

Check ```rubrics.md``` for the grading scheme of this lab.

### NOTES for pre-lab activities
Given the limited time in the lab, it is highly recommended to go through this manual and start (or complete) your implementation before the lab date, by working on your personal setup (VMWare, remote desktop, lent laptop), and using simulation for testing when needed to verify that your codes are working before coming into the lab. For simulation, refer to `tbt3Simulation.md` in the `main` branch.

During the 3 hours in the lab, you want to utilize this time to test your code, work with the actual robot, get feedback from the TAs, and acquire the in-lab marks (check `rubrics.md` in the same branch).

While in-lab, you are required to use the Desktop PCs and **NOT** your personal setup (VMWare, remote desktop, lent laptop). So, make sure that you have your modified files, either online or on a USB, with you to try it out in-lab. 

### Pre-lab deliverable
A first version of the completed code is to be submitted **24 hours before the group's lab section** (e.g. groups on the Wednesday section must submit by Tuesday at 3 PM), along with a list of doubts/questions to be solved during the in-person lab section (optional, if needed). The code does not have to be perfect but should be the best attempt and commented. The in-person lab is intended primarily for **testing, troubleshooting, and receiving TA support**, rather than implementing the code from scratch. 

The following penalties apply:

- **5 marks** for failing to submit the preliminary code by the deadline.
- **10 additional marks** for failing to demonstrate and explain a complete working simulation, including **all in-lab components**, within the first 20 minutes of the scheduled lab session.

Students are encouraged to post on Piazza and reach out to the TAs for support ahead (at least 48-72h ahead is recommended) of their scheduled lab session to successfully complete the pre-lab deliverables.

## Part 1 - connecting to the robot (no marks)
Open the [connectToUWtb4s.md](https://github.com/aalghooneh/MTE544_student/blob/main/connectToUWtb4s.md) markdown file in the main branch, and follow along. Read and follow each step carefully.

## Part 2 - Robot teleop (no marks)

In this part, you will learn to play with the robot; you will get to undock it from the charger and then move it around by keyboard.  
When you want to dock it again, It should be able to find it only when it is in less than ~0.5 meter around it. Note, that it doesn't
necessarily goes to the dock that it was undocked from, it will just find the next available dock.

The undock command goes through a [action server](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html).

```
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
```
You robot should undock.
If not, revisit *Part 1* - Connect to robot via VPN, and make sure the VPN terminal is still running and that you can still see the robot's topics. If you suspect the vpn isn't working, make sure you terminate it, and then run again.

Next run the teleop command to be able to move the robot manually around.

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

See the prompt for help on the keys. 

To dock the robot, use:

```
ros2 action send_goal /dock irobot_create_msgs/action/Dock {}
```

## NOTE: when you open a new terminal, you need to source again and set the domain ID, or you will not see the topics:

- Source the .bashrc file: source ~/robohub/turtlebot4/configs/.bashrc
- Declare ros2 domain: export ROS_DOMAIN_ID=X (X being the number of your robot)

## Odometry reset
For this lab, you may want to reset the odometry so that when you start your path, the odometry starts from the (0, 0, 0) state.
Use this command to reset the odometry on the physical TurtleBot4 (in simulation, by simply restarting the simulation, you can reset the odometry):

```
ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose

```

## Part 3 - Implement the Extended Kalman Filter (EKF) (40 marks)
For the implementation of the EKF, you will use the following sensor data to localize the robot:
- IMU;
- Odometry (as we do not have direct access to wheel encoders on TurtleBot4).
  
The structure of the code is basically the same as the one from LAB-2, but the localization is based on EKF instead of using raw data. See comment in ```decisions.py```.

The estimation algorithm is implemented in ```kalman_filter.py``` and is used in ```localization.py```, which is then used by ```decisions.py```.

The algorithm of the EKF implemented in ```kalman_filter.py``` is as follows (note that the notation is a bit different than the lecture slides):

```
Prediction step:
x = f(x, u) // This is the motion model function
P = A*P*A' + Q // note that Q is the covariance matrix of the states
```

```
Update step:

S = C*P*C' + R // note that R is the covariance matrix of the measurements
K = P*C'*inv(S)
Y_bar = z - h(x) // h is the measurement function
x = x + K*Y_bar
P = (1 - K*C)*P
```

For the estimation, you can use:
- As state vector ```x = [x,y,th,w,v,vdot]```
- As measurements, data from odometry and IMU ```z = [v,w,ax,ay]```. 

Derive the necessary equations and matrices for the process model and measurement model to complete the EKF implementation.

For this part:
- Follow the comments in ```kalman_filter.py```to implement the EKF with all necessarily quantities and matrices.
- Comment the code in ```kalman_filter.py```, explaining what the code does and what is the role of each matrix and function in the code; 
- Follow the comments in ```localization.py``` to complete the implementation of the EKF for the robots using IMU and odom by deriving all necessary matrices and functions in ```initKalmanfilter``` and complete the steps in ```fusion_callback```.

## Part 4 - Test the EKF implementation (20 marks)
After you've finished the implementation of the EKF, you can proceed with testing your localization:
- Spiral motion (use this motion to tune your EKF);
- The point controller;
If you want, in addition, you can also test with the other trajectories provided (or the ones you implemented in Lab-2 - this is optional).

In testing your code, make sure to try different values for the covariance matrices and log the data accordingly. How do they influence the estimation?

You can start with Q = 0.5 and R = 0.5 (multiplied by the correct sizes of Identity matrices to create the covariance matrices). Try to increase and decrease the value of Q without modifying R to observe the effect of modifying the state covariance. Then do the same for R, try to increase and then decrease the value of R without modifying Q to see the effect of modifying the measurement covariance. The range for the variations of Q and R should be in the range of decimals and units. 

Perform at least two variations of Q and two variations of R for only spiral motion to put in your written report (a total of 4 combinations).
Perform the final tuning with your point controller and put the results in your written report.

Follow the comments in ```localization.py``` to complete the data logging, the headers are there.

These data (robot_pose.csv) will be needed for the plots to be reported in the written report. Plot estimates vs measurements. You're free to adapt the plotting script for the required plots.

**Show the results of your EKF for both the spiral and the point to a TA to score half of the marks for this part.**


## Conclusions - Written report (40 marks)
You can do this part in the lab (time allowing) or at home. **Make sure you have the proper data saved**.

Please prepare a written report containing on the front page:
- Names (Family Name, First Name) of all group members;
- Student ID of all group members;
- Station number and robot number.

In a maximum of 3 pages (excluding the front page), report the performance of the EKF. This report should contain the following:

* Provide your understanding of the algorithm provided for the EKF and report on your derivations of the matrices, including the process model and measurement model. 
* Results of the EKF implementation, compare the measured and estimated. Perform this comparison considering different cases of the covariance matrices as specified in Part 4. The plots should have, title, label name for the axis, legends, different shapes/colors for each line, and grids. 

## Submission

Submit the report on Crowdmark and the code on Dropbox (LEARN) in the corresponding folder. Only one submission per group on each platform is needed:
- **Report**: one single pdf, do not put the pdf inside the zip file, submit it as an independent file to Crowdmark, remember to add all group members to a group before submitting;
- **Code**: make sure to have commented your code! Submit one single zip file with everything (including the csv files obtained from the data log) to Dropbox on LEARN in the folder corresponding to your section.

Good luck!
