# LAB 2 - Closed loop control of mobile robots

## Introduction

Welcome to LAB 2 of the mobile robotics course! Here, you start shaping the overall structure of your mobile robot stack. Please check again the architecture provided in the README.md in the main branch and leave that open on your web browser. 

In this lab, Participants will gain hands-on experience with mobile robots and acquire knowledge about their basic functionalities. By the end of this lab, participants will be able to:
- Use the position data in a controller;
- Control a mobile robot using a tunable PID controller.
- Implement pure pursuit controller


#### The summary of what you should learn is as following:

1. You will learn how to properly read and log position sensors using ros 2.
2. You will write a PID controller class that can properly calculate derivate, and integral of your position sensor.
3. You use the PID controller to perform two trajectories with the mobile robot while logging the error. Then, a report should be prepared comparing the P and PID controller in agility, accuracy, and overshoot.


### NOTES for pre-lab activities
Given the limited time in the lab, it is highly recommended to go through this manual and start (or complete) your implementation before the lab date, by working on your personal setup (VMWare, remote desktop, lent laptop), and using simulation for testing when needed to verify that your codes are working before coming into the lab. For simulation, refer to `tbt3Simulation.md` in the `main` branch.

During the 3 hours in the lab, you want to utilize this time to test your code, work with the actual robot, get feedback from the TAs, and acquire the in-lab marks (check `rubrics.md` in the same branch).

While in-lab, you are required to use the Desktop PCs and **NOT** your personal setup (VMWare, remote desktop, lent laptop). So, make sure that you have your modified files, either online or on a USB, with you to try it out in-lab. 

### Pre-lab deliverable
A first version of the completed code is to be submitted **24 hours before the group's lab section** (e.g. groups on the Wednesday section must submit by Tuesday at 3 PM), along with a list of doubts/questions to be solved during the in-person lab section (optional, if needed). The in-person lab is not meant for implementation but for testing and getting help from the TAs. 

Failure to submit will result in a penalty of 5 marks on the lab report. The code does not have to be fully correct, but it should be (at least almost) complete and meaningful with appropriate comments.

### Gain tuning
In this lab, you will be tuning PID gains for the controller. Remember that gains tuned for the robot in simulation may not work for the real robot, so you do not need to achieve perfect tracking in simulation, just use it to check the working status of your code and whether the outputs are reasonable with just a few tuning.

In the lab, on the physical robot, tune to achieve reasonable results, and to obtain sufficient plots to demonstrate your tuning process (see sections below and what is needed to report in Conclusion).

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

## Part 3 - Read the position sensor, and log data (15 marks)

In this lab, you will implement a closed-loop controller to drive the robot by completing different parts of the code: the controller, the planner, and the localization. You can refer to the architecture to see how they are connected together.

- The controller is implemented using different control laws. You will start with a proportional controller (P) and then extend to proportional, derivative, integral (PID).
- The planner generates the desired path/destination for the robot to follow, this can be as simple as a point planner, or a trajectory. At this stage, you will only implement predefined points/trajectories assuming the environment is perfect, we will see in LAB-4 how to implement optimal paths considering the environment (e.g. obstacles, walls).
- The localization tells you where the robot is so that you can move the robot along the desired paths and read the data to feed back to your controller. At this stage, you will simply use the odometry to determine the position of the robot. In LAB-3 we will see how you can improve localization with sensor information (state estimation).

To start with, you need to read the position sensor and log the data so it can be used in the controller.

Follow the comments in ```utilities.py```, ```localization.py``` and ```decisions.py```.

## Part 4 - Write a P controller (20 marks)
Start with a simple case and write a P controller only. Remember the control law of a P-controller, you need the proportional gain and the error of the system you are regulating, in this case, they are the linear and angular movement of the robot.
To implement the controller, you will need to:
- Compute both the linear and angular errors; follow the comments in ```utilities.py```;
- Use the error to implement the control law of the P-controller; follow the comments in ```pid.py```;
- Log your errors to evaluate the performance of your controller and to tune the gains; follow the comments in ```pid.py```;
- Add saturation limits for the robot's linear and angular velocity; follow the comments in ```controller.py```;

  For real robot: Check out maximum linear and angular velocity from [Turtlebot 4 Specifications](https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html#hardware-specifications).

  For simulation: Check out maximum linear and angular velocity from [Turtlebot3 Burger Specifications](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)
- Send the velocities to the robot to move the robot; follow the comments in ```decisions.py```.

Now, you can test your P-controller with the point planner before proceeding:
- Make sure that the VPN terminal is still running, i.e., ```Tunnel is ready```, and that you can see the robot's topic list;
- Remember to source your ```.bashrc``` file and set ```ROS_DOMAIN_ID```, in case you did not set up a permenant environment;
- Undock your robot if needed, and use the teleop node to drive your robot to a free space;
- Run: ```python3 decision.py --motion point``` and robot should move to the corresponding point specified in ```planner.py```.

## Part 5 - Upgrade to PID (20 marks)
Now that you have implemented a P-controller, proceed with the extension to include the derivative and integral components. You will need to:
- Implement the error derivative and integral; follow the comments in ```pid.py```;
- Implement the control laws for PD, PI, PID; follow the comments in ```pid.py```;
- Add saturation limits (just like in part 4) for the robot's linear and angular velocity; follow the comments in ```controller.py```;
- Test each controller, i.e., P, PD, PI, and PID, using the point planner; follow the comments in ```controller.py```;
- Log your errors to evaluate the performance of your controllers (focus on the tuning for P and PID) and to plot them in your report, see the Conclusion section below on reporting;
- Plot robot pose and errors using the ```plot_errors.py``` for each controller.
- Tune your code based on the plots; follow the comments in ```decisions.py```.

## Part 6 - Perform trajectories and log your error (20 marks)
Implement more complex trajectories to test the performance of your controllers, follow the comments in ```planner.py```, and ```controller.py```. Cover these two trajectories, while logging the error. 

* Parabola: $y = x^2$ for $x \in [0.0, 1.5]$
* Sigmoid: $\sigma(x) = {2}/({1 + e^{-2x}}) - 1$ for $x \in [0.0, 2.5]$ 

For real robots, you will need to tune your generated trajectory to make sure it fits into the classroom space.
Choose one of the controllers among P, PI, PD, PID (you need to provide a reason) and tune your gains to obtain a good performance in terms of tracking:
Run: ```python3 decision.py --motion trajectory```

Process the errors and visualize the plots to see how your controller is performing. Do a post-process plot with the logged data. You can use the ```plot_errors.py``` file (adapt and modify accordingly).

Try: You can also log your data with a bag file and save it so that you can use it after the lab is over. The bag file allows you to replay all the topics as if you ran the robot again. 
To record a bag file (you can also check [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)):
- In a terminal, run ```ros2 bag record <topic_name>```, replace ```<topic_name>``` with the list of topics you want to record.
- To stop recording, simply CTRL+C con the bag recording terminal.
- To check the info of the bag you recorded ```ros2 bag info <bag_folder_name>```.
- To play a bag file, in a terminal, run ```ros2 bag play <bag_folder_name>```.
Note that when playing the bag file, it will play only for the duration for which you recorded it. If you want it to loop, you can add the option ```--loop``` to the bag play command.

*NOTE* do not save all the topics in your bag file, as it would be extremely large, select the ones that you actually need.

**IMPORTANT!! Before you leave, DELETE all of your codes, files, etc.**

## Conclusions - Written report (25 marks)
You can do this part in the lab (time allowing) or at home. Make sure you have the proper data saved.

Please prepare a written report containing on the front page:
- Names (Family Name, First Name) of all group members;
- Student ID of all group members;
- Station number and robot number.

In a maximum of 3 pages (excluding the front page), report a comparison between the P-controller and the PID one. This report should only have two sections:

* Section 1 - the plot of the logged error for the trajectories. The plot should have, title, label name for axis, legends, different shapes/colors for each error, and grids.
  * The plots you need to provide are as follows (for those in curly brackets, it is best to have them in one figure):
    * The plot of P, PID controller for point controller, in which you have for all states for {e-t, edot-t}, and {x-t, y-t, th-t}, and {x-y}, {e-edot}.
    * For your final chosen controller for part 6, provide the reason and the plot of the parabola and sigmoid trajectories {x-y} only, and if you had the plot of the linear and angular error that was recorded during the trajectory motion execution, then nice to provide that but it is not necessary.
* Section 2 - A table comparing the P and PID controllers in agility, accuracy, and overshoot numerically **for the point controller only**. You should find the metrics for these three quantities from automatic control concepts/previous control courses. 

## Submission

Submit the report and the code on Dropbox (LEARN) in the corresponding folder. Only one submission per group is needed:
- **Report**: one single pdf, **do not** put the pdf inside the zip file, submit it as an independent file;
- **Code**: make sure to have commented your code! Submit one single zip file with everything (including the csv files obtained from the data log).


Good luck!
