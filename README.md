# Map my world
Robotic Software Engineering Localization

Adaptive Monte Carlo Localization Project

To run the project, use the following command in the terminal:

$ cd home/robond/catkin_ws/

$ catkin_make

$ source devel/setup.bash

$ roslaunch my_robot world.launch

In a second terminal, run the following:

$ cd home/robond/catkin_ws/

$ source devel/setup.bash

$ roslaunch my_robot amcl.launch

In a third terminal, run the following:

$ cd home/robond/catkin_ws/

$ source devel/setup.bash

$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

To complete this project, I referenced the two following repositories for its directory structure and AMCL parameter tuning:

1.) https://github.com/rfzeg/udacity_bot

2.) https://github.com/DanielsKraus/Robotics-Software-Engineer

3.) https://github.com/huuanhhuynguyen/RoboND-Where-Am-I/blob/master/my_robot/launch/amcl.launch

4.) https://github.com/jinchaolu/RoboND-Term1-P3-Where-Am-I/blob/master/catkin_ws/src/my_robot/launch/amcl.launch 