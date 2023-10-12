import rospy
import numpy as np
import pinocchio as pin
from std_msgs.msg import Int8
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point, Quaternion

def transform_to_se3(trans):
    return pin.XYZQUATToSE3([trans.translation.x, trans.translation.y, trans.translation.z,
                             trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])

def input_accept(prompt):
    choice = input(prompt + " [y]/n: ")
    if(choice == "" or choice.lower()[0] == 'y'):
        return True
    return False

def compute_barycenter(pose_list, fp_iter=100, callback=None):
    """
    bi invariant barycenter
    """
    guess = pose_list[0]
    if callback is not None:
        callback(guess)

    for _ in range(fp_iter):
        guess =  pin.exp(
            pin.Motion(
                np.stack([pin.log(p * guess.inverse()).np for p in pose_list], axis=0).mean(axis=0)
            )
        ) * guess
        if callback is not None:
            callback(guess)
    return guess

def compute_covariance(barycenter, pose_list):
    N = len(pose_list)
    logs_l_riem = np.stack([
        np.concatenate([
            pin.log3((barycenter.inverse() * p).rotation),
            (barycenter.inverse() * p).translation
        ], axis=0)
        for p in pose_list
    ], axis=0)
    V_l_riem = np.einsum('ib,ic->bc', logs_l_riem, logs_l_riem) / (N-1)
    return V_l_riem

def visp_meas_filter(n, tracker_topic):
    """
    Wait for n consecutive measures in Tracking Mode and return the last one.

    (Ensure that the measure is not a false positive)

    Parameters:
    -----------
        n (int): number of consecutive measurements to wait for.
        tracker_topic (str): prefix topic to subscribe to.

    Returns:
    --------
        success (bool): True if the average was successful, False otherwise.
        pose (Pose): the last measure of the n consecutives.
        stamp (Time): Timestamp of the last measure.
    """
    pose = None
    for i in range(n):
        status = rospy.wait_for_message(tracker_topic + "/status", Int8)
        if(status.data != 3): # Not tracking
            return False, None, None
        if i == n-1:
            transf_marker = rospy.wait_for_message(tracker_topic + "/object_position", PoseStamped)
            pose = transf_marker.pose
            stamp = transf_marker.header.stamp
    return True, pose, stamp
