#!/bin/bash
sed -i '1s/^.*$//' ~/catkin_ws/src/sayens/src/compute_target_pose_tf.py
sed -i '1c #!/usr/bin/env python3' ~/catkin_ws/src/sayens/src/compute_target_pose_tf.py
rosrun sayens compute_target_pose_tf.py


