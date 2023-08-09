# Drone simulation
- inspired by [link](https://github.com/NovoG93/sjtu-drone)
# Step
- [ ] empty env
    - [ ] delete world files and stay with a default world
- [ ] put camera to drone
    - [ ] edit sdf model
- [ ] put a QR code in the environment
    - [ ] needs a bit of research. put QR code on a plane or on a side of a cube
- [ ] detect QR code and print its relative position
    - [ ] run help script
- [ ] write a motion control to follow QR code and keep a certain distance from it.
    - [ ] set a target position and rotation from middle of QR code.
    - [ ] give target position and rotation to drone and tell it to go there. organize it in two PID problems: position and rotation.
    - [ ] first move QR without rotating
    - [ ] then just rotate QR
    - [ ] then mix
- [ ] for result, put drone in different angles and positions and measure PnP error.
# ROS Package Details

I created this package by editing the sjtu Drone simulation in GitHub repo [link](https://github.com/NovoG93/sjtu-drone). The original repo had some bugs. After fixing them, I added a laser sensor to the drone and implemented ```drone_navigator``` node. I tested this package on my system, which is utilized with **ROS Noetic (Ubuntu 20.04)** and **Gazebo version 11.10.2**.

## Drone Details
![rosgraph](./img/drone.png)

Drone has only two sensors: 
1. Laser Scanner (/drone/laser)
2. IMU (/drone/imu)

The rqt_graph for this package is provided below:

![rosgraph](./img/rosgraph.png)

## Downloading and running
Clone this repository, install dependencies and build following the commands below:
```
cd ~/catkin_ws
rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO 
catkin_make
source devel/setup.bash
```
For running, execute the command below.
```
roslaunch sjtu_drone start.launch
```
