To navigate a robot with a rgb-d sensor and a voxel layer for obstacle detection

## Cloning the repo
From home directory create /catkin_ws/src folder and run the following command
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/utra-robosoccer/Tutorials-2020.git
```

## Updating Dependencies
```
cd ~/catkin_ws/
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic
```

## Installing Controllers For Robot
```
sudo apt-get update
sudo apt-get install ros-noetic-ros-controllers
```


## Building tutorial package
First build the project and source the setup file so that the system knows where to look for your build files
```
cd ~/catkin_ws
catkin build tutorial
source devel/setup.bash
```

## Launch the robot
```
roslaunch tutorial gazebo.launch
```


## Commands used during tutorial
useful tip: press tab to auto-complete words as you type commands
Open a new terminal to run commands for the robot

```
cd ~/catkin_ws
source devel/setup.bash
```
To run the obstacle detection node run this command
```
rosrun tutorial my_publisher
```

To send a command to the left arm
```
rostopic pub /left_arm_controller/command std_msgs/Float64 "data: 1.0"
```

To see the sensor data
```
rqt
```
    
