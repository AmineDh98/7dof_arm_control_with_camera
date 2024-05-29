#!/bin/bash
sed -i '1s/^.*$//' ~/catkin_ws/src/sayens/src/compute_target_pose.py
sed -i '1i#!/usr/bin/env python3' ~/catkin_ws/src/sayens/src/compute_target_pose.py
rosrun sayens compute_target_pose.py


