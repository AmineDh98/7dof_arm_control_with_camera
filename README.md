
# sayens
Optic table environment description with a 7dof robotic arm and a camera


First clone the repository **kinova-ros** from this [link](https://github.com/Kinovarobotics/kinova-ros).

Run ```sudo apt-get install ros-noetic-moveit*```
Then do build your catkin workspace.



Then clone this repository and build the catkin_ws and run ```roslaunch sayens display.launch``` 

to move the robot run ```rosrun kinova_control move_robot.py j2n7s300```