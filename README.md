# ROS-Wheeled_Robot

To Run The project follow the following steps:

Updating Dependencies

       cd ~/catkin_ws/
       rosdep update
       rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic

Installing Controllers For Robot

       sudo apt-get update
       sudo apt-get install ros-noetic-ros-controllers

Building tutorial package

       cd ~/catkin_ws
       catkin build tutorial
       source devel/setup.bash

Launch the robot

       roslaunch tutorial gazebo.launch

To start the robot

       cd ~/catkin_ws
       source devel/setup.bash
       rosrun tutorial my_publisher
