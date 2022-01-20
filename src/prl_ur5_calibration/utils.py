import rospy
import numpy as np
import pinocchio as pin
from std_msgs.msg import Int8
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point, Quaternion

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
    """
    pose = None
    for i in range(n):
        status = rospy.wait_for_message(tracker_topic + "/status", Int8)
        if(status.data != 3): # Not tracking
            return False, None
        if i == n-1:
            transf_marker = rospy.wait_for_message(tracker_topic + "/object_position", PoseStamped)
            pose = transf_marker.pose
    return True, pose
