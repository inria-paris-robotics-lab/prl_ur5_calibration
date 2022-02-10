#!/usr/bin/env python
from copy import deepcopy
import rospy
import pickle
import rospkg

from geometry_msgs.msg import Transform, Vector3, Quaternion, Quaternion
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
from visp_hand2eye_calibration.msg import TransformArray
import tf
from tqdm import tqdm

from prl_ur5_calibration.utils import visp_meas_filter
from prl_pinocchio.tools.utils import compare_poses, quaternion_to_euler
import pinocchio as pin
import numpy as np

def transform_to_se3(trans):
    return pin.XYZQUATToSE3([trans.translation.x, trans.translation.y, trans.translation.z,
                             trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])

class Calibration:
    def __init__(self):
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

    def _getTransforms(self, from_f, to_f):
        latest_t = self.tf_listener.getLatestCommonTime(from_f, to_f)
        (trans, rot) = self.tf_listener.lookupTransform(from_f, to_f, latest_t)

        return Transform(translation=Vector3(*trans), rotation=Quaternion(*rot))

    def _get_marker(self):
        for _ in range(50): # Tries
            success, pose = visp_meas_filter(30, "/visp_auto_tracker")
            if success:
                return pose
            rospy.sleep(0.2)
        rospy.logwarn("Failed to get marker")
        return None

    def _go_at_pose(self, pose):
        assert not rospy.is_shutdown()
        try:
            path = self.planner.make_gripper_approach(self.robot.left_gripper_name, *pose, approach_distance = 0)
            # self.planner.pp(path.id)
            self.commander_left_arm.execute_path(path)
        except Exception as e:
            rospy.logwarn("Failed to plan path")
            rospy.logwarn(e.args)
            return False
        return True

    def get_measures(self, poses, logfile, recover=False):
        # self.set_gripper("open")
        meas_log = []
        poses = deepcopy(poses)

        # Recover measure to not re-do it
        recover_cnt = 0
        if recover:
            with open(logfile, "rb") as f:
                recovered_data = pickle.load(f)

            for meas in recovered_data:
                for p in poses:
                    if meas[0] == p: # The pose was in the recover file
                        meas_log.append(meas)
                        poses.remove(p)
                        recover_cnt += 1
                        break

            rospy.logwarn(F"Recovered {recover_cnt} measures (over the {len(recovered_data)} previously saved). {len(poses)} measures left to do.")

        # Loop over the remaining poses
        for pose in tqdm(poses):
            # Save all the measures taken so far, in case of crash
            with open(logfile, "wb") as f:
                pickle.dump(meas_log, f)

            # Add a none element at the end of the log. Will be replaced by the real value (if success)
            meas_log += [[pose, None, None]]

            # Plan and go at pose
            success = self._go_at_pose(pose)
            if not success:
                continue

            # Wait for the robot to be at the exact position
            is_at_pose = False
            rospy.logwarn("Wait for the robot to be exactly at the pose...")
            while(not rospy.is_shutdown()):
                rospy.sleep(1.2)
                gripper_pose = self.robot.get_frame_pose(self.robot.get_gripper_link(self.robot.left_gripper_name))
                is_at_pose = compare_poses(pose[0]+pose[1], gripper_pose, threshold=0.004)
                if is_at_pose:
                    rospy.logwarn("At pose")
                    break

            # Get the marker position
            camera_object = self._get_marker()
            if not camera_object:
                continue
            camera_object = Transform(translation = camera_object.position, rotation = camera_object.orientation)

            # Get the effector position
            world_effector = self._getTransforms("/prl_ur5_base", "/left_tool")

            # Check rough position estimation
            effector_camera = self._getTransforms("/left_tool", "/left_camera_color_optical_frame")
            wMe = transform_to_se3(world_effector)
            eMc = transform_to_se3(effector_camera)
            cMo = transform_to_se3(camera_object)
            wMo = wMe * eMc * cMo

            trans_err_vect = wMo.translation
            rot_err_vect = pin.log3(wMo.rotation)
            trans_err = np.linalg.norm(trans_err_vect)
            rot_err = np.linalg.norm(rot_err_vect)

            rospy.logwarn(F"Marker pose:\n{wMo}")
            if(trans_err > 0.05 or rot_err > 0.2):
                rospy.logerr("Marker measured too far away from origin: point discarded"
                            +F"\n(translation error {trans_err} (wrt 0.05), rotation error {rot_err} (wrt 0.2))")
                continue

            # Save values
            meas_log[-1] = (pose, camera_object, world_effector)

        # Save all the measures
        with open(logfile, "wb") as f:
            pickle.dump(meas_log, f)

    def compute_calibration(self, logfile):
        camera_object_list = []
        world_effector_list = []

        log_data = pickle.load(open(logfile, "rb"))
        for meas in log_data:
            if meas[1] is not None:
                camera_object_list.append(meas[1])
                world_effector_list.append(meas[2])

        # Compute the calibration
        calibration = self.srv_calibrate(TransformArray(transforms=camera_object_list), TransformArray(transforms=world_effector_list))

        # Convert results in same frames as configuration file
        eMc = transform_to_se3(calibration.effector_camera)
        bsMe = transform_to_se3(self._getTransforms("/left_camera_bottom_screw_frame", "/left_tool"))
        bsMc = bsMe * eMc

        wMo_exact = pin.XYZQUATToSE3([0,0,0.005, 0,0,0,1])
        wMo_meas = transform_to_se3(calibration.world_object)
        w_oldMw_new = wMo_meas * wMo_exact.inverse()

        wMsl = transform_to_se3(self._getTransforms("/prl_ur5_base", "/stand_link"))
        slMbl = transform_to_se3(self._getTransforms("/stand_link", "/left_base_link"))
        slMbl_new = wMsl.inverse() * w_oldMw_new * wMsl * slMbl

        # Print the results
        rospy.logwarn("\narm_pose:" + self.str_pretty_pose(slMbl_new) + "\ncamera_pose:" + self.str_pretty_pose(bsMc))

    def str_pretty_pose(self, se3):
        xyzquat = pin.SE3ToXYZQUAT(se3)
        euler = quaternion_to_euler(list(xyzquat[3:]))
        return F"""
        x: {xyzquat[0]}
        y: {xyzquat[1]}
        z: {xyzquat[2]}
        roll: {euler[0]}
        pitch: {euler[1]}
        yaw: {euler[2]}"""



if __name__ == "__main__":
    rospy.init_node("calibration")

    # Read calibration poses
    poses_filepath = rospkg.RosPack().get_path("prl_ur5_calibration") + "/files/poses_calibration.p"
    poses_file = open(poses_filepath, "rb")
    poses = pickle.load(poses_file)
    poses_file.close()

    # Prepare pickle log file for measures
    measures_filepath = rospkg.RosPack().get_path("prl_ur5_calibration") + "/files/measures.p"

    # Parameters
    recover = rospy.get_param("~recover") # Recover measures from a previous run if possible
    compute_only = rospy.get_param("~compute_only", False) # Only read from the existing measure file and do the computation

    # Run calibration
    calibration = Calibration()
    if not compute_only:
        calibration.get_measures(poses, measures_filepath, recover)
    calibration.compute_calibration(measures_filepath)