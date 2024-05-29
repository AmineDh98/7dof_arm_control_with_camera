#!/usr/bin/env python3







import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sayens.srv import TargetPose, TargetPoseResponse
import tf.transformations as tf_trans

class ComputeTargetPose:
    def __init__(self):
        rospy.init_node('compute_target_pose')

        self.robot_to_table = self.get_transform_params('/robot_to_table')
        self.camera_to_table = self.get_transform_params('/camera_to_table')
        self.target_to_camera = self.get_transform_params('/target_to_camera')
        
        self.service = rospy.Service('target_pose_service', TargetPose, self.handle_compute_target_pose)
        rospy.loginfo("ComputeTargetPose service ready.")

    def get_transform_params(self, prefix):
        params = {}
        params['x'] = rospy.get_param(f'~{prefix}/x', 0.0)
        params['y'] = rospy.get_param(f'~{prefix}/y', 0.0)
        params['z'] = rospy.get_param(f'~{prefix}/z', 0.0)
        params['roll'] = rospy.get_param(f'~{prefix}/roll', 0.0)
        params['pitch'] = rospy.get_param(f'~{prefix}/pitch', 0.0)
        params['yaw'] = rospy.get_param(f'~{prefix}/yaw', 0.0)
        print(params['x'], ' ',params['y'], ' ' ,params['z'])
        return params

    def handle_compute_target_pose(self, req):
        robot_to_table_pose = self.get_pose_from_params(self.robot_to_table)
        camera_to_table_pose = self.get_pose_from_params(self.camera_to_table)
        target_to_camera_pose = self.get_pose_from_params(self.target_to_camera)

        final_pose = self.compute_final_pose(robot_to_table_pose, camera_to_table_pose, target_to_camera_pose)

        response = TargetPoseResponse()
        response.target_pose = final_pose
        return response

    def get_pose_from_params(self, params):
        quaternion = tf_trans.quaternion_from_euler(params['roll'], params['pitch'], params['yaw'])
        pose = PoseStamped()
        pose.pose.position.x = params['x']
        pose.pose.position.y = params['y']
        pose.pose.position.z = params['z']
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    def compute_final_pose(self, robot_to_table, camera_to_table, target_to_camera):

        
        robot_to_table_matrix = tf_trans.compose_matrix(translate=[robot_to_table.pose.position.x, robot_to_table.pose.position.y, robot_to_table.pose.position.z],
                                                        angles=tf_trans.euler_from_quaternion([robot_to_table.pose.orientation.x, robot_to_table.pose.orientation.y, robot_to_table.pose.orientation.z, robot_to_table.pose.orientation.w]))
        camera_to_table_matrix = tf_trans.compose_matrix(translate=[camera_to_table.pose.position.x, camera_to_table.pose.position.y, camera_to_table.pose.position.z],
                                                         angles=tf_trans.euler_from_quaternion([camera_to_table.pose.orientation.x, camera_to_table.pose.orientation.y, camera_to_table.pose.orientation.z, camera_to_table.pose.orientation.w]))
        target_to_camera_matrix = tf_trans.compose_matrix(translate=[target_to_camera.pose.position.x, target_to_camera.pose.position.y, target_to_camera.pose.position.z],
                                                          angles=tf_trans.euler_from_quaternion([target_to_camera.pose.orientation.x, target_to_camera.pose.orientation.y, target_to_camera.pose.orientation.z, target_to_camera.pose.orientation.w]))

        print("robot_to_table_matrix = ", robot_to_table_matrix)
        print("camera_to_table_matrix = ", camera_to_table_matrix)
        print("target_to_camera_matrix = ", target_to_camera_matrix)

        robot_to_table_matrix_inv = np.linalg.inv(robot_to_table_matrix)

        final_transform = tf_trans.concatenate_matrices(robot_to_table_matrix_inv, camera_to_table_matrix, target_to_camera_matrix)
        translation = tf_trans.translation_from_matrix(final_transform)
        rotation = tf_trans.quaternion_from_matrix(final_transform)

        final_pose = PoseStamped()
        final_pose.pose.position.x = translation[0]*(-1000)
        final_pose.pose.position.y = translation[1]*1000
        final_pose.pose.position.z = translation[2]*(-1000)
        final_pose.pose.orientation.x = rotation[0]
        final_pose.pose.orientation.y = rotation[1]
        final_pose.pose.orientation.z = rotation[2]
        final_pose.pose.orientation.w = rotation[3]

        print("final_pose = ", final_pose.pose.position.x, ' ', final_pose.pose.position.y, ' ', final_pose.pose.position.z)

        return final_pose

if __name__ == '__main__':
    try:
        ComputeTargetPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
