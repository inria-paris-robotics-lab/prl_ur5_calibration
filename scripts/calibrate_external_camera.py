#!/usr/bin/env python
import os
import rospy
from tf import transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

from prl_ur5_calibration.utils import visp_meas_filter

class CameraCalibration:
    def __init__(self, sample_nb, camera_name, tracker_node):
        self.sample_nb = sample_nb
        self.camera_name= camera_name
        self.tracker_node = tracker_node

    def get_marker(self):
        while not rospy.is_shutdown():
            success, pose, _ = visp_meas_filter(self.sample_nb, self.tracker_node)
            if success:
                return pose
            rospy.logwarn("Tracking not stable enough, retrying...")

    def run(self):
        # Get marker transformation
        pose = self.get_marker()

        # Inverse the transformation from camera-to-marker to marker-to-camera (a.k.a origin-to-camera)
        quat = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        trans = pose.position.x, pose.position.y, pose.position.z

        quat_mat = transformations.quaternion_matrix(quat)
        trans_mat = transformations.translation_matrix(trans)

        concat_mat = transformations.concatenate_matrices(trans_mat, quat_mat)

        res_mat = transformations.inverse_matrix(concat_mat)
        marker_T_camera = res_mat
        marker_initial_pos = [-0.45, 0.0, 0.008]
        marker_initial_quat = [0.0, 0.0, 0.0, 1.0]

        init_quat_mat = transformations.quaternion_matrix(marker_initial_quat)
        init_trans_mat = transformations.translation_matrix(marker_initial_pos)

        world_T_marker = transformations.concatenate_matrices(init_trans_mat, init_quat_mat)

        world_T_camera = np.matmul(world_T_marker, marker_T_camera)

        res_trans = transformations.translation_from_matrix(world_T_camera)
        res_quat = transformations.quaternion_from_matrix(world_T_camera)

        # User-friendly print the result
        pretty_str = make_pretty_str(self.sample_nb, self.camera_name, res_trans, res_quat)
        rospy.loginfo(pretty_str)

    def done(self):
        # Kill the visp_auto_tracker node as it's not needed anymore
        rospy.loginfo(F"Done calibrating > Killing {self.tracker_node} node")
        os.system(F"rosnode kill {self.tracker_node}")

def make_pretty_str(sample_nb, camera_name, trans, rot):
    euler = euler_from_quaternion(rot)
    return F"""\n
############################################
# Generated from calibrate_external_camera #
############################################
# For camera : {camera_name}
# (Filtered on {sample_nb} samples)

  pose:
    x: {trans[0]:.6f}
    y: {trans[1]:.6f}
    z: {trans[2]:.6f}
    roll:  {euler[0]:.3f}
    pitch: {euler[1]:.3f}
    yaw:   {euler[2]:.3f}

    (degrees :
    roll:  {euler[0]*180/3.1415926:.1f} deg
    pitch: {euler[1]*180/3.1415926:.1f} deg
    yaw:   {euler[2]*180/3.1415926:.1f} deg
    )
"""

if __name__ == "__main__":
    rospy.init_node("calibrate_external_camera", anonymous=True, log_level=rospy.INFO)

    camera_name = rospy.get_param("~camera_name") # Name of the camera for print purposes
    tracker_node = rospy.get_param("~tracker_node") # Get the name of visp_auto_tracker node
    sample_nb = rospy.get_param("~sample_nb") # Number of samples for averaging the pose

    run_loop = rospy.get_param("~run_loop", False) # Stop after one measurment or not

    # Run calibration
    calibration = CameraCalibration(sample_nb, camera_name, tracker_node)
    while not rospy.is_shutdown():
       calibration.run()
       if not run_loop:
           break
    calibration.done()

