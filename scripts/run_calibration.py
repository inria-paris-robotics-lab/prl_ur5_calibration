#!/usr/bin/env python
from copy import deepcopy
import rospy
import pickle
import rospkg
import os

from geometry_msgs.msg import Transform, Vector3, Quaternion, Quaternion
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
from visp_hand2eye_calibration.msg import TransformArray
import tf
from tqdm import tqdm

from prl_ur5_calibration.utils import input_accept, compute_barycenter, compute_covariance, visp_meas_filter, transform_to_se3
from prl_pinocchio.tools.utils import euler_to_quaternion
import pinocchio as pin
import numpy as np
import yaml

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
        self.planner.set_planning_timeout(45.0)
        self.planner.set_collision_margin(0.03)
        self.commander_left_arm.start_trajectory()

        # Calibration topics
        rospy.loginfo("Waiting for service compute_effector_camera_quick...")
        rospy.wait_for_service('compute_effector_camera_quick')
        self.srv_calibrate = rospy.ServiceProxy('compute_effector_camera_quick', compute_effector_camera_quick)

    def _getTransforms(self, from_f, to_f, timestamp):
        self.tf_listener.waitForTransform(from_f, to_f, timestamp, rospy.Duration(4.0))
        (trans, rot) = self.tf_listener.lookupTransform(from_f, to_f, timestamp)

        return Transform(translation=Vector3(*trans), rotation=Quaternion(*rot))

    def _get_marker(self):
        if input_accept(f"Measure marker pose ?"):
            success, pose, stamp = visp_meas_filter(30, "/visp_auto_tracker")
            if success:
                return pose, stamp
        rospy.logwarn("Failed to get marker")
        return None, None

    def _go_at_pose(self, pose):
        while not rospy.is_shutdown():
            try:
                path = self.planner.make_gripper_approach(self.robot.left_gripper_name, pose, approach_distance = 0)
                if(input_accept(f"Plan found (ID: {path.id}) ! Play ?")):
                    break
                print('Replanning...')
            except Exception as e:
                # import traceback
                # traceback.print_exc()
                rospy.logwarn(e.args)
                if(not input_accept(f"Failed to find a path. Replan ?")):
                    return False
        self.commander_left_arm.execute_path(path)
        return True

    def get_measures(self, marker_pose, pose_filelist, measure_dirpath, recover=False):
        # self.set_gripper("open")

        # Exact marker pose in world frame
        worldMobject_exact = pin.XYZQUATToSE3(marker_pose["xyz"] + euler_to_quaternion(marker_pose["rpy"]))

        # Add a collision box on the object
        self.planner.v.loadObstacleModel(rospkg.RosPack().get_path("prl_ur5_calibration") + "/files/models/marker.urdf", "marker_collision", guiOnly=False)
        self.planner.v.moveObstacle("marker_collision/bounding_box_0", marker_pose["xyz"] + euler_to_quaternion(marker_pose["rpy"]), guiOnly=False)
        self.planner.display(self.robot.get_meas_q())

        # Loop over the remaining poses
        for pose_filepath in tqdm(pose_filelist):
            # Extract name of the pose
            shortfile = pose_filepath.split('/')[-1]
            assert shortfile.endswith(".pkl")
            assert shortfile.startswith("pose_")
            pose_name = shortfile[5:-4] # remove the '.pkl' and take what is after "pose_"

            measure_filepath = measure_dirpath + "meas_" + pose_name + ".pkl"

            # Skip if recover is one
            if(recover and os.path.isfile(measure_filepath)):
                rospy.logwarn(f"Pose {pose_name} recovered. Skipping this measurment.")

            with open(pose_filepath, 'rb') as f:
                pose = pickle.load(f)

            # Plan and go at pose
            success = self._go_at_pose(pose)
            if not success:
                continue

            # Wait for the robot to be at pose
            rospy.sleep(3.5)

            # Get the marker position
            optical_object, stamp = self._get_marker()
            if not optical_object:
                continue
            optical_object = Transform(translation = optical_object.position, rotation = optical_object.orientation)

            # Get the effector position
            shoulder_effector = self._getTransforms("/left_base_link", "/left_tool", stamp)

            # Check rough position estimation
            world_shoulder = self._getTransforms("/prl_ur5_base", "/left_base_link", stamp)
            effector_optical = self._getTransforms("/left_tool", "/left_camera_color_optical_frame", stamp)
            worldMshoulder = transform_to_se3(world_shoulder)
            shoulderMeffector = transform_to_se3(shoulder_effector)
            effectorMoptical = transform_to_se3(effector_optical)
            opticalMobject = transform_to_se3(optical_object)
            worldMobject = worldMshoulder * shoulderMeffector * effectorMoptical * opticalMobject

            objectMobject_err = worldMobject_exact.inverse() * worldMobject

            trans_err_vect = objectMobject_err.translation
            rot_err_vect = pin.log3(objectMobject_err.rotation)
            trans_err = np.linalg.norm(trans_err_vect)
            rot_err = np.linalg.norm(rot_err_vect)

            rospy.logwarn(F"Marker pose:\n{worldMobject}")
            if(trans_err > 0.05 or rot_err > 0.2):
                rospy.logerr(F"Marker measured far away from expected pose: point (n°{pose_name}) kept anyway)\n"
                            +F"(translation error {trans_err} (wrt 0.05), rotation error {rot_err} (wrt 0.2))")

            # Save value
            meas = (pose, optical_object, shoulder_effector)
            with open(measure_filepath, 'wb') as f:
                pickle.dump(meas, f)

    def compute_calibration(self, marker_pose, measure_filelist):
        optical_object_list = []
        shoulder_effector_list = []

        meas_data = []
        for measure_filepath in measure_filelist:
            with open(measure_filepath, 'rb') as f:
                meas_data.append(pickle.load(f))

        for meas in meas_data:
            if meas[1] is not None:
                optical_object_list.append(meas[1])
                shoulder_effector_list.append(meas[2])

        # Compute the calibration
        calibration = self.srv_calibrate(TransformArray(transforms=optical_object_list), TransformArray(transforms=shoulder_effector_list))

        # Convert results in same frames as configuration file
        effectorMoptical = transform_to_se3(calibration.effector_camera)
        opticalMscrews = transform_to_se3(self._getTransforms("/left_camera_color_optical_frame", "/left_camera_bottom_screw_frame", rospy.Time(0)) )
        effectorMscrews = effectorMoptical * opticalMscrews

        worldMobject_exact = pin.XYZQUATToSE3(marker_pose["xyz"] + euler_to_quaternion(marker_pose["rpy"]))

        # Compute the barycenter of all the measurments to estimate the arm position in the world frame
        shoulderMobject_list = [ transform_to_se3(shoulder_effector) * effectorMoptical * transform_to_se3(optical_object) for shoulder_effector, optical_object in zip(shoulder_effector_list, optical_object_list)]
        shoulderMobject_meas = compute_barycenter(shoulderMobject_list)

        standMworld = transform_to_se3(self._getTransforms("/stand_link", "/prl_ur5_base", rospy.Time(0)))

        standMshoulder = standMworld * worldMobject_exact * shoulderMobject_meas.inverse()

        # Compute precision of the calibration
        worlMobject_list = [standMworld.inverse() * standMshoulder * shoulderMobject for shoulderMobject in shoulderMobject_list]
        worldMobject_bary = compute_barycenter(worlMobject_list)
        worldMobject_cov = compute_covariance(worldMobject_bary, worlMobject_list)
        worldMobject_std_dev = np.sqrt(worldMobject_cov.diagonal())
        object_pos_std = np.linalg.norm(worldMobject_std_dev[:3])
        object_rot_std = np.linalg.norm(worldMobject_std_dev[3:])

        # Print the results
        rospy.loginfo(F"""
arm_pose: {self.str_pretty_pose(standMshoulder)}
camera_pose: {self.str_pretty_pose(effectorMscrews)}
marker (wrt world) std deviation:
position: {object_pos_std} m
rotation: {object_rot_std} rad
                      """)


    def str_pretty_pose(self, se3):
        trans = se3.translation
        euler = pin.rpy.matrixToRpy(se3.rotation)
        return F"""
        x: {trans[0]}
        y: {trans[1]}
        z: {trans[2]}
        roll: {euler[0]}
        pitch: {euler[1]}
        yaw: {euler[2]}"""



if __name__ == "__main__":
    rospy.init_node("run_calibration", anonymous=True, log_level=rospy.INFO)

    abs_path = rospkg.RosPack().get_path("prl_ur5_calibration") + "/"

    # Read configuration file
    config_filepath = abs_path + rospy.get_param("~config_file")
    with open(config_filepath, "r") as cfg_file:
        cfg = yaml.safe_load(cfg_file)

    # Read calibration poses
    poses_dirpath = abs_path + cfg["output_paths"]["poses_export"]
    poses_filelist = os.listdir(poses_dirpath)
    poses_filelist = [poses_dirpath + pose_filepath for pose_filepath in poses_filelist]

    marker_pose = cfg["marker_pose"]

    # Prepare pickle log file for measures
    measure_dirpath = abs_path + cfg["output_paths"]["measures"]

    # Parameters
    recover = rospy.get_param("~recover") # Recover measures from a previous run if possible
    compute_only = rospy.get_param("~compute_only", False) # Only read from the existing measure file and do the computation

    # Run data collection
    calibration = Calibration()
    if not compute_only:
        calibration.get_measures(marker_pose, poses_filelist, measure_dirpath, recover)

    # Run calibration
    measure_filelist = os.listdir(measure_dirpath)
    measure_filelist = [measure_dirpath + measure_filepath for measure_filepath in measure_filelist]
    calibration.compute_calibration(marker_pose, measure_filelist)