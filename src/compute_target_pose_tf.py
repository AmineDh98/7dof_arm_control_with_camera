#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sayens.srv import TargetPose, TargetPoseResponse
import tf.transformations as tf_trans
import tf

class ComputeTargetPose:
    def __init__(self):
        rospy.init_node('compute_target_pose')

        self.tf_listener = tf.TransformListener()

        self.service = rospy.Service('target_pose_service', TargetPose, self.handle_compute_target_pose)
        rospy.loginfo("ComputeTargetPose service ready.")

    def get_transform(self, target_frame, source_frame):
        try:
            self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr(f"Could not get transform from {source_frame} to {target_frame}")
            return None, None

    def handle_compute_target_pose(self, req):
        robot_to_table_trans, robot_to_table_rot = self.get_transform('table', 'root')
        camera_to_table_trans, camera_to_table_rot = self.get_transform('table', 'camera')
        target_to_camera_trans, target_to_camera_rot = self.get_transform('camera', 'xy_platform')

        if not (robot_to_table_trans and camera_to_table_trans and target_to_camera_trans):
            rospy.logerr("One or more transforms could not be obtained")
            return None

        robot_to_table_pose = self.get_pose_from_tf(robot_to_table_trans, robot_to_table_rot)
        camera_to_table_pose = self.get_pose_from_tf(camera_to_table_trans, camera_to_table_rot)
        target_to_camera_pose = self.get_pose_from_tf(target_to_camera_trans, target_to_camera_rot)

        final_pose = self.compute_final_pose(robot_to_table_pose, camera_to_table_pose, target_to_camera_pose)

        response = TargetPoseResponse()
        response.target_pose = final_pose
        return response

    def get_pose_from_tf(self, trans, rot):
        pose = PoseStamped()
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        return pose

    def compute_final_pose(self, robot_to_table, camera_to_table, target_to_camera):
        robot_to_table_matrix = tf_trans.compose_matrix(translate=[robot_to_table.pose.position.x, robot_to_table.pose.position.y, robot_to_table.pose.position.z],
                                                        angles=tf_trans.euler_from_quaternion([robot_to_table.pose.orientation.x, robot_to_table.pose.orientation.y, robot_to_table.pose.orientation.z, robot_to_table.pose.orientation.w]))
        camera_to_table_matrix = tf_trans.compose_matrix(translate=[camera_to_table.pose.position.x, camera_to_table.pose.position.y, camera_to_table.pose.position.z],
                                                         angles=tf_trans.euler_from_quaternion([camera_to_table.pose.orientation.x, camera_to_table.pose.orientation.y, camera_to_table.pose.orientation.z, camera_to_table.pose.orientation.w]))
        target_to_camera_matrix = tf_trans.compose_matrix(translate=[target_to_camera.pose.position.x, target_to_camera.pose.position.y, target_to_camera.pose.position.z],
                                                          angles=tf_trans.euler_from_quaternion([target_to_camera.pose.orientation.x, target_to_camera.pose.orientation.y, target_to_camera.pose.orientation.z, target_to_camera.pose.orientation.w]))

        rospy.loginfo("robot_to_table_matrix = %s", robot_to_table_matrix)
        rospy.loginfo("camera_to_table_matrix = %s", camera_to_table_matrix)
        rospy.loginfo("target_to_camera_matrix = %s", target_to_camera_matrix)

        robot_to_table_matrix_inv = np.linalg.inv(robot_to_table_matrix)

        final_transform = tf_trans.concatenate_matrices(robot_to_table_matrix_inv, camera_to_table_matrix, target_to_camera_matrix)
        translation = tf_trans.translation_from_matrix(final_transform)
        rotation = tf_trans.quaternion_from_matrix(final_transform)

        final_pose = PoseStamped()
        final_pose.pose.position.x = translation[0] * (-1000)
        final_pose.pose.position.y = translation[1] * 1000
        final_pose.pose.position.z = translation[2] * (-1000)
        final_pose.pose.orientation.x = rotation[0]
        final_pose.pose.orientation.y = rotation[1]
        final_pose.pose.orientation.z = rotation[2]
        final_pose.pose.orientation.w = rotation[3]

        rospy.loginfo("final_pose = [%f, %f, %f]", final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z)

        return final_pose

if __name__ == '__main__':
    try:
        ComputeTargetPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
