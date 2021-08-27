# F1Tenth-Lab3
This is a repo for F1Tenth Lab 3 Assignment which implements PID controller according to the desired distance to the wall at the left of the vehicle.

<img src="wall_follow.gif"/>

## Dependeices for Running Locally
* ROS Melodic
    * Ubuntu [click here for installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)
* F1TENTH Simulator
    * [click here for installation instructions](https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/index.html)

## Basic Run Instructions

1. Clone this repo into catkin_ws/src.
2. Launch it: `roslaunch wall_follow wall_follow_nav.launch`.
3. Press 'N' key to activate navigation mode in the running terminal.

## PID Parameters Tuning Iteration

1. Firstly, Kp needs to be much higher than Kd and Ki. Kd can help eliminate overshoot while little Ki can helps in overall response settles settles in shorter time.
2. Kp too high will lead to consistent overshoot, while too low will result to sluggish steering.
3. High Kd dosent affect much in my case. but if it is too low, the overshoot problem will be obvious.
4. High Ki will lead to overshoot.
5. With the current PID values, if angle between two laser scan is 60 degree, my car cannot turn at the bottom right corner as it straight detected the left wall after T-junction, so I changed it to 55 degree to solve the problem.
6. Slow down vehicle speed plays a important role during large steering input, as the pid controller can have enough time for localisation and calculation. 

## Problems and further modifications

1. The overall driving experience is still not smooth, especially at the bottom part of the map. Further tuning of PID values is needed.