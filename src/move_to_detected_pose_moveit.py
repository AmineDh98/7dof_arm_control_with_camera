#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
from sayens.srv import TargetPose, TargetPoseRequest
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys

class RobotController:
    def __init__(self):
        # Initialize node
        rospy.init_node('initial_node')

        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        # Define publisher for goal pose
        self.pub_goal_pose = rospy.Publisher('goal_pose', Point, queue_size=10)

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
        self.goal_pose = Point(self.sigma_d[0], self.sigma_d[1], self.sigma_d[2])

        # Move the robot to the target pose
        self.move_to_pose(target_pose)

        # Publish goal position
        self.pub_goal_pose.publish(self.goal_pose)
        rospy.loginfo("Received target pose: %s", self.sigma_d)

    def move_to_pose(self, pose):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pose.position.x
        target_pose.position.y = pose.position.y
        target_pose.position.z = pose.position.z
        target_pose.orientation.x = pose.orientation.x
        target_pose.orientation.y = pose.orientation.y
        target_pose.orientation.z = pose.orientation.z
        target_pose.orientation.w = pose.orientation.w

        self.group.set_pose_target(target_pose)

        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        rospy.loginfo("Current pose: %s", current_pose)

        # Check if the goal is reached
        if self.is_goal_reached(current_pose, target_pose):
            rospy.loginfo("Position reached")
            rospy.signal_shutdown("Position reached")

    def is_goal_reached(self, current_pose, target_pose):
        position_tolerance = 0.01  # meters
        orientation_tolerance = 0.01  # radians

        position_error = np.linalg.norm([
            current_pose.position.x - target_pose.position.x,
            current_pose.position.y - target_pose.position.y,
            current_pose.position.z - target_pose.position.z
        ])

        orientation_error = np.linalg.norm([
            current_pose.orientation.x - target_pose.orientation.x,
            current_pose.orientation.y - target_pose.orientation.y,
            current_pose.orientation.z - target_pose.orientation.z,
            current_pose.orientation.w - target_pose.orientation.w
        ])

        return position_error < position_tolerance and orientation_error < orientation_tolerance

if __name__ == '__main__':
    controller = RobotController()
