#!/bin/bash
sed -i '1s/^.*$//' ~/catkin_ws/src/sayens/src/move_to_initial_pose.py
sed -i '1c #!/usr/bin/env python3' ~/catkin_ws/src/sayens/src/move_to_initial_pose.py
rosrun sayens move_to_initial_pose.py