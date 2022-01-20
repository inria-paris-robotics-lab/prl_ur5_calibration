#!/usr/bin/env python
import rospy
import pickle
import rospkg

from geometry_msgs.msg import Transform, Vector3, Quaternion, Quaternion
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
from visp_hand2eye_calibration.msg import TransformArray
import tf
from tqdm import tqdm

from prl_ur5_calibration.utils import visp_meas_filter
from prl_pinocchio.tools.utils import compare_poses

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
            success, pose = visp_meas_filter(30, "/visp_auto_tracker")
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
            # Plan and go at pose
            success = self.go_at_pose(pose)
            if not success:
                continue

            # Wait for the robot to be at the exact position
            is_at_pose = False
            rospy.logwarn("Wait for the robot to be exactly at the pose...")
            while(not rospy.is_shutdown()):
                rospy.sleep(2.5)
                gripper_pose = self.robot.get_frame_pose(self.robot.get_gripper_link(self.robot.left_gripper_name))
                is_at_pose = compare_poses(pose[0]+pose[1], gripper_pose, threshold=0.002)
                if is_at_pose:
                    rospy.logwarn("At pose")
                    break

            # Get the marker position
            camera_object = self.get_marker()
            if not camera_object:
                continue

            # Get the effector position
            world_effector = self.getTransforms("/left_base_link", "/left_tool")

            world_camera = self.getTransforms("/left_base_link", "/left_camera_color_optical_frame")
            # Todo: check that world_camera > camera_obect ~= O

            #Save values
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