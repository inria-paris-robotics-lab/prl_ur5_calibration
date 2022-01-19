#!/usr/bin/env python
import rospy
import pickle
import rospkg

from std_msgs.msg import Int8
from geometry_msgs.msg import Transform, Vector3, Quaternion, PoseStamped, Pose, Point, Quaternion
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
from visp_hand2eye_calibration.msg import TransformArray
import tf
from tqdm import tqdm

import numpy as np
import pinocchio as pin

def average_consecutive_meas(n, tracker_topic):
    vw_sum = np.zeros(6)
    for i in range(n):
        status = rospy.wait_for_message(tracker_topic + "/status", Int8)
        if(status.data != 3): # Not tracking
            return False, None
        transf_marker = rospy.wait_for_message(tracker_topic + "/object_position", PoseStamped)
        pose = transf_marker.pose
        position_orientation = [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        vw = pin.log6(pin.XYZQUATToSE3(position_orientation)).np
        vw_sum += vw
    vw_avg = vw_sum / n
    se3_avg = pin.exp6(vw_avg)
    pose_avg = pin.SE3ToXYZQUAT(se3_avg)
    return True, Pose(position = Point(*pose_avg[:3]), orientation = Quaternion(*pose_avg[3:]))
class Calibration:
    def __init__(self, poses):
        # Transformations
        self.tf_listener = tf.TransformListener()

        # Configure the planning and command pipeline
        from prl_hpp.ur5 import robot, planner, commander_left_arm
        self.planner = planner
        self.robot = robot
        self.commander_left_arm = commander_left_arm

        self.planner.lock_grippers()
        self.planner.lock_right_arm()
        self.planner.set_velocity_limit(0.25)
        self.planner.set_acceleration_limit(0.25)
        self.planner.set_planning_timeout(10.0)
        self.commander_left_arm.start_trajecotry()

        # Calibration topics
        rospy.loginfo("Waiting for service compute_effector_camera_quick...")
        rospy.wait_for_service('compute_effector_camera_quick')
        self.srv_calibrate = rospy.ServiceProxy('compute_effector_camera_quick', compute_effector_camera_quick)

        # Poses
        self.poses = poses

    def getTransforms(self, from_f, to_f):
        now = rospy.Time.now()
        self.tf_listener.waitForTransform(from_f, to_f, now, rospy.Duration(4.0))
        (trans,rot) = self.tf_listener.lookupTransform(from_f, to_f, now)

        return Transform(translation=Vector3(*trans), rotation=Quaternion(*rot))

    def get_marker(self):
        for _ in range(50): # Tries
            success, pose = average_consecutive_meas(30, "/visp_auto_tracker")
            if success:
                return pose
            rospy.sleep(0.2)
        rospy.logwarn("Failed to get marker")
        return None

    def go_at_pose(self, pose):
        try:
            path = self.planner.make_gripper_approach(self.robot.left_gripper_name, *pose, approach_distance = 0)
            # self.planner.pp(path.id)
            rospy.loginfo("Press enter to execute...")
            input("(Press enter to execute...)")
            self.commander_left_arm.execute_path(path)
        except:
            rospy.loginfo("Failed to plan path")
            return False
        return True


    def run(self):
        # self.set_gripper("open")

        camera_object_list = []
        world_effector_list = []

        for pose in tqdm(self.poses):
            success = self.go_at_pose(pose)
            if not success:
                continue

            rospy.sleep(1.0)

            camera_object = self.get_marker()
            if not camera_object:
                continue

            world_effector = self.getTransforms("/left_base_link", "/left_tool")

            world_camera = self.getTransforms("/left_base_link", "/left_camera_color_optical_frame")
            # Todo: check that world_camera > camera_obect ~= O

            camera_object_list.append(camera_object)
            world_effector_list.append(world_effector)

        calibration = self.srv_calibrate(TransformArray(transforms=camera_object_list), TransformArray(transforms=world_effector_list))

        return calibration

if __name__ == "__main__":
    rospy.init_node("calibration")

    # Read calibration poses
    poses_filepath = rospkg.RosPack().get_path("prl_ur5_calibration") + "/files/poses_calibration.p"
    poses_file = open(poses_filepath, "rb")
    poses = pickle.load(poses_file)
    poses_file.close()

    # Run calibration
    calibration = Calibration(poses)
    calibration.run()