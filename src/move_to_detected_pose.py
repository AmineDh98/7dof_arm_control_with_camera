#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
from kin import *  # Functions define kinematics and jacobians
import math
from sayens.srv import TargetPose, TargetPoseRequest

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

        # Control gains
        self.K = np.diag([3, 3, 3])
        self.abc = [0, 0, 0, 0, 0, 0, 0]
        self.goal_reached = False  # Flag to indicate when the goal is reached
        self.pose_received = False  # Flag to indicate when the pose is received

        # Desired end-effector position (initially not set)
        self.sigma_d = None
        self.goal_pose = Point()

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

        try:
            # Wait for the service to become available
            rospy.loginfo("Waiting for target_pose_service...")
            rospy.wait_for_service('target_pose_service')
            rospy.loginfo("target_pose_service available.")
            # Create a service proxy
            self.target_pose_service = rospy.ServiceProxy('target_pose_service', TargetPose)
            self.get_target_pose()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            rospy.signal_shutdown("Service call failed")
        except rospy.ROSInterruptException:
            rospy.loginfo("Node shutdown before completing initialization.")
            return

        # Spin to keep the node alive
        rospy.spin()

    def get_target_pose(self):
        # Call the service to get the target pose
        rospy.loginfo("Calling target_pose_service...")
        request = TargetPoseRequest()
        response = self.target_pose_service(request)
        target_pose = response.target_pose.pose

        # Update the desired end-effector position
        self.sigma_d = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
        self.goal_pose.x = self.sigma_d[0]
        self.goal_pose.y = self.sigma_d[1]
        self.goal_pose.z = self.sigma_d[2]
        self.pose_received = True  # Set the flag indicating the pose has been received
        rospy.loginfo("Received target pose: %s", self.sigma_d)

    def distance(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)
    def joint_states_callback(self, msg):
        if not self.pose_received or self.goal_reached:
            return  # If the goal is reached or pose not received, do nothing
        if (self.distance(self.sigma_d,[0,0,0])>984 or self.sigma_d[2]>0):
            rospy.loginfo("Position is not reachable")
            rospy.signal_shutdown("Position is not reachable")
            return
        # Extract current joint positions
        current_joint_positions = np.array(msg.position[0:7], dtype=float)
        self.abc = msg.position[0:7]

        # Update robot
        T = kinematics(self.d, current_joint_positions, self.a, self.alpha)
        J = jacobian(T, self.revolute)
        J1 = J[0:3, :]
        T = np.array(T)
        sigma = np.array([T[-1][0][-1], T[-1][1][-1], T[-1][2][-1]])
        

        # Check if any of the joints collide with the table
        sigma_list = [
            np.array([T[-1][0][-1], T[-1][1][-1], T[-1][2][-1]]),
            np.array([T[1][0][-1], T[1][1][-1], T[1][2][-1]]),
            np.array([T[2][0][-1], T[2][1][-1], T[2][2][-1]]),
            np.array([T[3][0][-1], T[3][1][-1], T[3][2][-1]]),
            np.array([T[4][0][-1], T[4][1][-1], T[4][2][-1]]),
            np.array([T[5][0][-1], T[5][1][-1], T[5][2][-1]]),
            np.array([T[6][0][-1], T[6][1][-1], T[6][2][-1]])
        ]

        
        if any(sigma[2] > 0 for sigma in sigma_list):
            rospy.loginfo("Robot collided")
            rospy.signal_shutdown("Robot collided, killing node...")
            return


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
        rospy.loginfo("current position = %s", sigma)
        if np.linalg.norm(err) < 30:
            rospy.loginfo("Position reached")
            self.goal_reached = True
            rospy.signal_shutdown("Position reached")

if __name__ == '__main__':
    controller = RobotController()
