#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
import numpy as np
from kin import *  # Functions define kinematics and jacobians
import math

class RobotController:
    def __init__(self):
        # Initialize node
        rospy.init_node('initial_node')

        # Global variables for resolved-rate motion control
        self.d = np.array([-275.5, 0, -410, 13.3, -311.1, 0, -263.8])
        self.q = np.array([0, -2.356194, 0.0, 0.785398, -1.570796, -1.570796, -1.570796], dtype=float)
        self.alpha = np.array([math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi])
        self.a = np.array([0, 0, 0, 0, 0, 0, 0])
        self.revolute = [True, True, True, True, True, True, True]
        self.dt = 0.1

        # Desired end-effector position
        self.sigma_d = np.array([0, 0, 1220])
        self.sigma_d = np.array([-self.sigma_d[0], self.sigma_d[1], -self.sigma_d[2]])
        self.goal_pose = Point()
        self.goal_pose.x = self.sigma_d[0]
        self.goal_pose.y = self.sigma_d[1]
        self.goal_pose.z = self.sigma_d[2]
        print(self.sigma_d)

        # Control gains
        self.K = np.diag([3, 3, 3])
        self.abc = [0, 0, 0, 0, 0, 0, 0]
        self.goal_reached = False  # Flag to indicate when the goal is reached

        # Define publishers for each joint
        self.pub_joint1 = rospy.Publisher('/j2s7s300/joint_1_position_controller/command', Float64, queue_size=10)
        self.pub_joint2 = rospy.Publisher('/j2s7s300/joint_2_position_controller/command', Float64, queue_size=10)
        self.pub_joint3 = rospy.Publisher('/j2s7s300/joint_3_position_controller/command', Float64, queue_size=10)
        self.pub_joint4 = rospy.Publisher('/j2s7s300/joint_4_position_controller/command', Float64, queue_size=10)
        self.pub_joint5 = rospy.Publisher('/j2s7s300/joint_5_position_controller/command', Float64, queue_size=10)
        self.pub_joint6 = rospy.Publisher('/j2s7s300/joint_6_position_controller/command', Float64, queue_size=10)
        self.pub_joint7 = rospy.Publisher('/j2s7s300/joint_7_position_controller/command', Float64, queue_size=10)

        self.pub_goal_pose = rospy.Publisher('goal_pose', Point, queue_size=10)

        # Subscribe to the joint states topic
        rospy.Subscriber('/j2s7s300/joint_states', JointState, self.joint_states_callback)

        # Spin to keep the node alive
        rospy.spin()

    def joint_states_callback(self, msg):
        if self.goal_reached:
            return  # If the goal is reached, do nothing

        # Extract current joint positions
        current_joint_positions = np.array(msg.position[0:7], dtype=float)
        self.abc = msg.position[0:7]

        # Update robot
        T = kinematics(self.d, current_joint_positions, self.a, self.alpha)
        J = jacobian(T, self.revolute)
        J1 = J[0:3, :]
        T = np.array(T)
        sigma = np.array([T[-1][0][-1], T[-1][1][-1], T[-1][2][-1]])
        err = (self.sigma_d - sigma).reshape((3, 1))
        dq1 = np.linalg.pinv(J1) @ (self.K @ err)
        dq2 = dq1[:, 0]
        self.abc += self.dt * dq2

        # Publish joint positions
        self.pub_joint1.publish(self.abc[0])
        self.pub_joint2.publish(self.abc[1])
        self.pub_joint3.publish(self.abc[2])
        self.pub_joint4.publish(self.abc[3])
        self.pub_joint5.publish(self.abc[4])
        self.pub_joint6.publish(self.abc[5])
        self.pub_joint7.publish(self.abc[6])

        # Publish goal position
        self.pub_goal_pose.publish(self.goal_pose)
        print("current position = ", sigma)
        if np.linalg.norm(err) < 10:
            print("Position reached")
            self.goal_reached = True
            rospy.signal_shutdown("Position reached")

if __name__ == '__main__':
    controller = RobotController()
