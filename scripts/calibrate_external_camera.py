#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from tf import transformations

from prl_ur5_calibration.utils import visp_meas_filter

class CameraCalibration:
    def __init__(self, camera_name, tracker_node):
        self.camera_name= camera_name
        self.tracker_node = tracker_node

    def get_marker(self):
        while not rospy.is_shutdown():
            success, pose = visp_meas_filter(1, self.tracker_node)
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

        res_trans = transformations.translation_from_matrix(res_mat)
        res_quat = transformations.quaternion_from_matrix(res_mat)

        # User-friendly print the result
        msg = make_msg(self.camera_name, res_trans, res_quat)
        rospy.logwarn(msg)

    def done(self):
        # Kill the visp_auto_tracker node as it's not needed anymore
        rospy.loginfo(F"Done calibrating > Killing {self.tracker_node} node")
        os.system(F"rosnode kill {self.tracker_node}")

def make_msg(camera_name, trans, rot):
    return F"""\n
############################################
# Generated from calibrate_external_camera #
############################################
# For camera : {camera_name}
  position:
    x: {trans[0]}
    y: {trans[1]}
    z: {trans[2]}
  orientation:
    x: {rot[0]}
    y: {rot[1]}
    z: {rot[2]}
    w: {rot[3]}
"""

if __name__ == "__main__":
    rospy.init_node("calibrate_external_camera", anonymous=True)

    camera_name = rospy.get_param("~camera_name") # Name of the camera for print purposes
    tracker_node = rospy.get_param("~tracker_node") # Get the name of visp_auto_tracker node

    # Run calibration
    calibration = CameraCalibration(camera_name, tracker_node)
    calibration.run()
    calibration.done()