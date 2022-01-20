import rospy
import numpy as np
import pinocchio as pin
from std_msgs.msg import Int8
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point, Quaternion


def average_poses(poses):
    """
    Compute the average of a list of poses.

    Parameters:
    -----------
        poses (Pose[]): list of poses to average.

    Returns:
    --------
        pose (Pose): average of the poses.
    """
    vw_sum = np.zeros(6)
    for pose in poses:
        se3 = pin.XYZQUATToSE3([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        vw = pin.log6(se3).np
        vw_sum += vw
    vw_avg = vw_sum / len(poses)
    se3_avg = pin.exp6(vw_avg)
    pose_avg = pin.SE3ToXYZQUAT(se3_avg)
    return Pose(position = Point(*pose_avg[:3]), orientation = Quaternion(*pose_avg[3:]))

def visp_meas_filter(n, tracker_topic):
    """
    Compute the average of n consecutive measurements from the tracker topic.

    Measures are considered valid if the tracker status is Tracking Mode (3).
    If one measure of the batch is not valid, the the method fails and returns (False, None)

    Average is computed as follows:
    wMo_avg = exp6(sum(log6(wMo_k))/n)))

    Parameters:
    -----------
        n (int): number of consecutive measurements to average.
        tracker_topic (str): prefix topic to subscribe to.

    Returns:
    --------
        success (bool): True if the average was successful, False otherwise.
        pose (Pose): average of the n consecutive measurements.
    """
    poses = []
    for i in range(n):
        status = rospy.wait_for_message(tracker_topic + "/status", Int8)
        if(status.data != 3): # Not tracking
            return False, None
        transf_marker = rospy.wait_for_message(tracker_topic + "/object_position", PoseStamped)
        poses.append(transf_marker.pose)
    pose_avg = average_poses(poses)
    return True, pose_avg
