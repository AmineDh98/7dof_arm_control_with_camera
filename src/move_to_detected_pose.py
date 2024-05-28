#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
import numpy as np
from kin import *  # Assuming you have these functions
import math


# Global variables for resolved-rate motion control
d = np.array([-275.5, 0, -410, 13.3, -311.1, 0, -263.8])
q = np.array([0, -2.356194, 0.0, 0.785398, -1.570796, -1.570796, -1.570796], dtype=float)
alpha = np.array([math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi])
a = np.array([0, 0, 0, 0, 0, 0, 0])

revolute = [True, True, True, True, True, True, True]

# Desired end-effector position
# sigma_d = np.array([-164, 240, 540])
sigma_d = np.array([0, 400, 300])
sigma_d=np.array([-sigma_d[0], sigma_d[1], -sigma_d[2]])
goal_pose = Point()
goal_pose.x = sigma_d[0]
goal_pose.y = sigma_d[1]
goal_pose.z = sigma_d[2]
print(sigma_d)

# Control gains (you may need to adjust these)
K = np.diag([3, 3, 3])
abc = [0, 0, 0, 0, 0, 0, 0]

# Define publishers for each joint
pub_joint1 = rospy.Publisher('/j2s7s300/joint_1_position_controller/command', Float64, queue_size=10)
pub_joint2 = rospy.Publisher('/j2s7s300/joint_2_position_controller/command', Float64, queue_size=10)
pub_joint3 = rospy.Publisher('/j2s7s300/joint_3_position_controller/command', Float64, queue_size=10)
pub_joint4 = rospy.Publisher('/j2s7s300/joint_4_position_controller/command', Float64, queue_size=10)
pub_joint5 = rospy.Publisher('/j2s7s300/joint_5_position_controller/command', Float64, queue_size=10)
pub_joint6 = rospy.Publisher('/j2s7s300/joint_6_position_controller/command', Float64, queue_size=10)
pub_joint7 = rospy.Publisher('/j2s7s300/joint_7_position_controller/command', Float64, queue_size=10)

# Callback function to process joint states and control the arm
def joint_states_callback(msg):
    global d, q, a, alpha, revolute, sigma_d, K, abc
    
    # Extract current joint positions
    current_joint_positions = np.array(msg.position[0:7], dtype=float)
    abc = msg.position[0:7]

    # Update robot
    T = kinematics(d, current_joint_positions, a, alpha)
    J = jacobian(T, revolute)
    J1 = J[0:3, :]
    T = np.array(T)    
    sigma = np.array([T[-1][0][-1], T[-1][1][-1], T[-1][2][-1]])
    err = (sigma_d - sigma).reshape((3, 1)) 
    dq1 = np.linalg.pinv(J1) @ (K @ err)
    dq2 = dq1[:, 0] 
    abc += dt * dq2 
    
    # Publish joint positions
    pub_joint1.publish(abc[0])
    pub_joint2.publish(abc[1])
    pub_joint3.publish(abc[2])
    pub_joint4.publish(abc[3])
    pub_joint5.publish(abc[4])
    pub_joint6.publish(abc[5])
    pub_joint7.publish(abc[6])

    # Publish goal position
    pub_goal_pose.publish(goal_pose)
    print("current position = ", sigma)
    if np.linalg.norm(err) < 10:   
        print("Position reached")
        rospy.signal_shutdown("Position reached")
      
# Initialize the ROS node
rospy.init_node('control_arm_node')

# Set the rate (adjust as needed)
dt = 0.1

pub_goal_pose = rospy.Publisher('goal_pose', Point, queue_size=10)
# Subscribe to the joint states topic
rospy.Subscriber('/j2s7s300/joint_states', JointState, joint_states_callback)

# Spin to keep the node alive
rospy.spin()
