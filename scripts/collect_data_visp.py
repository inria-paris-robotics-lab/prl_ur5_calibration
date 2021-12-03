#!/usr/bin/env python

from numpy.lib.function_base import angle
import moveit_commander
import rospy
import sys

from prl_ur5_demos.utils import make_pose
import pinocchio as pin
import numpy as np
from std_msgs.msg import Int8
from geometry_msgs.msg import Transform, Vector3, Quaternion, PoseStamped
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
from visp_hand2eye_calibration.msg import TransformArray

import tf

class Calibration():
    def __init__(self):
        # Transformations
        self.tf_listener = tf.TransformListener()

        # Initialize the robot
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        # Configure the planning pipeline
        self.robot.left_arm.set_max_velocity_scaling_factor(0.2)
        self.robot.left_arm.set_max_acceleration_scaling_factor(0.2)
        self.robot.left_arm.set_planning_time(5)
        self.robot.left_arm.set_planner_id('RRTConnect')

        # print(self.robot.left_arm.get_end_effector_link()) # left_tool

        # Calibration topics
        rospy.wait_for_service('compute_effector_camera_quick')
        self.srv_calibrate = rospy.ServiceProxy('compute_effector_camera_quick', compute_effector_camera_quick)

    def go_at_pose(self, pose):
        success = self.go_cartesian_wps([pose])
        if(not success): # If cannot go in cartesion path, plan an other path
            print("Cannot move to position in cartesian mode > Try RRT")
            success = self.robot.left_arm.go(pose, wait=True)
        return success

    def go_cartesian_wps(self, waypoints):
        left_path, left_fraction = self.robot.left_arm.compute_cartesian_path(waypoints, eef_step=0.005, jump_threshold=5.0)
        if left_fraction >= 1.0:
            self.robot.left_arm.execute(left_path, wait=True)
            return True
        return False

    def set_gripper(self, state):
        self.robot.left_gripper.set_named_target(state)
        self.robot.left_gripper.go(wait=True)


    def getTransforms(self, from_f, to_f):
        now = rospy.Time.now()
        self.tf_listener.waitForTransform(from_f, to_f, now, rospy.Duration(4.0))
        (trans,rot) = self.tf_listener.lookupTransform(from_f, to_f, now)

        return Transform(translation=Vector3(*trans), rotation=Quaternion(*rot))

    def get_marker(self, n_tries=50, delay_tries=0.02):
        for _ in range(n_tries):
            status = rospy.wait_for_message("/visp_auto_tracker/status", Int8)
            if(status.data > 1):
                transf_marker = rospy.wait_for_message("/visp_auto_tracker/object_position", PoseStamped)
                return transf_marker.pose
            rospy.sleep(delay_tries)
        return None

    def go_to_rpy(self, l, rpy):
        # c = camera optical frame
        # t = tool frame
        # b = base frame

        tMc = pin.XYZQUATToSE3([0.03201936605892359, 0.03266617144909692, 0.16032922582431414, -0.003042505015539654, -0.0005310059444330884, -0.9999890872814666, 0.00350625311478453])

        bMc = pin.XYZQUATToSE3([0,0,l, 0,1,0,0])
        cMc = pin.exp6(np.array([0]*3 + rpy))
        bMc = cMc * bMc


        bMt = bMc * tMc.inverse()

        # go at pose
        xyzquat_look = pin.SE3ToXYZQUAT(bMt)
        pose_look = make_pose(xyzquat_look[:3], xyzquat_look[3:])

        success = self.go_at_pose(pose_look)
        return success

    def run(self):
        self.set_gripper("open")

        sample_distance = 0.45
        # sample_rpy = (np.random.rand(50,3) - 0.5) * np.pi
        # sample_rpy = np.array([ np.array([ 0.0192312 , -0.10345522, -1.06005948]),
        #                         np.array([ 0.08778395,  0.16220219, -1.24213999]),
        #                         np.array([-0.50674563, -0.4712473 , -1.01117966]),
        #                         np.array([ 0.09749527, -0.43810841, -0.09447612]),
        #                         np.array([ 0.41483697, -0.82465037,  0.08239443]),
        #                         np.array([-0.37199952, -0.52893351,  0.24522931]),
        #                         np.array([-0.1719889 , -0.36764195,  1.04454859]),
        #                         np.array([-0.02858335, -0.32575534,  1.07704686]),
        #                         np.array([-0.04557244, -0.74919633,  1.18554096]),
        #                         np.array([-0.62278683,  0.02091559,  0.87078776]),
        #                         np.array([-0.41170385,  0.27149211, -0.02671961]),
        #                         np.array([ 0.10062518,  0.59164302, -0.03274805])
        #                         ])
        # sample_rpy = np.array([ # np.array([ -np.pi/6, 0, -np.pi/2]),
        #                         # np.array([ 0, np.pi/6, 0]),
        #                         # np.array([ 0, np.pi/6, np.pi/4]),
        #                         ])

        interest_angles = [0, np.pi/6, -np.pi/6, np.pi/4, -np.pi/4, np.pi/3, -np.pi/3, np.pi/2, -np.pi/2]
        sample_rpy = []
        for r in interest_angles:
            for p in interest_angles:
                for y in interest_angles:
                    sample_rpy.append(np.array([r, p, y]))
        # # Reachable
        # selected_samples = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 18, 19, 20, 21, 22, 23, 24, 25, 26, 36, 37, 38, 39, 40, 41, 44, 54, 55, 56, 57, 58, 89, 100, 101, 103, 105, 107, 119, 121, 123, 125, 135, 137, 139, 141, 143, 162, 163, 164, 165, 166, 167, 168, 169, 170, 178, 180, 181, 182, 183, 184, 185, 187, 199, 201, 203, 205, 217, 219, 221, 265, 267, 269, 279, 281, 283, 285, 299, 301, 303, 327, 331, 343, 345, 347, 349, 363, 365, 367, 381, 383, 385, 431, 443, 445, 447, 449, 465, 467, 493, 511, 529, 547, 575, 593]
        # Viewable
        selected_samples = [ 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 , 25 , 27 , 28 , 29 , 30 , 31 , 37 , 39 , 41 , 42 , 43 , 46 , 52 , 53 , 54 , 55 , 56 , 57 , 58 , 59 , 60 , 61 , 62 , 63 , 64 , 65 , 67 , 68 , 77 , 78 , 87 , 88 , 89 , 90 , 105]
        # # Spaced
        # selected_samples = [42 , 60 , 58 , 78 , 62 , 105 , 89 , 8 , 17 , 53 , 67 , 65 , 63 , 64 , 68 , 52 , 16 , 7 , 25 , 43 , 61 , 59 , 57 , 77 , 88 ]
        sample_rpy = [sample_rpy[i] for i in selected_samples]
        sample_rpy = np.array(sample_rpy)

        # from numpy import array
        # sample_rpy = np.array([array([ 0.        , -0.78539816, -1.04719755]), array([ 0.        , -1.04719755, -1.04719755]), array([ 0.        , -1.04719755, -0.78539816]), array([ 0.        , -1.57079633, -1.04719755]), array([ 0.        , -1.04719755, -1.57079633]), array([ 0.52359878, -0.52359878, -1.04719755]), array([ 0.52359878,  0.        , -1.57079633]), array([ 0.        ,  0.        , -1.57079633]), array([ 0.        ,  0.52359878, -1.57079633]), array([ 0.        ,  1.04719755, -1.57079633]), array([ 0.        ,  1.57079633, -0.78539816]), array([ 0.        ,  1.57079633, -0.52359878]), array([0.        , 1.57079633, 0.        ]), array([0.        , 1.57079633, 0.52359878]), array([0.        , 1.57079633, 1.04719755]), array([0.        , 1.04719755, 1.57079633]), array([0.        , 0.52359878, 1.57079633]), array([0.        , 0.        , 1.57079633]), array([ 0.        , -0.52359878,  1.57079633]), array([ 0.        , -0.78539816,  1.57079633]), array([ 0.        , -1.04719755,  1.57079633]), array([ 0.        , -1.04719755,  1.04719755]), array([ 0.        , -1.04719755,  0.78539816]), array([ 0.        , -1.57079633,  1.04719755]), array([0.52359878, 0.        , 1.57079633])])

        camera_object_list = []
        world_effector_list = []

        res_i = []
        for i, angles in enumerate(sample_rpy):
            success = self.go_to_rpy(sample_distance, list(angles))
            if not success:
                print("not happened")
                continue

            # rospy.sleep(0.5)

            res_i.append(i)

            # camera_object = self.get_marker()
            # if not camera_object:
            #     continue

            # world_effector = self.getTransforms("/left_base_link", "/left_tool")

            # camera_object_list.append(camera_object)
            # world_effector_list.append(world_effector)

        print(res_i)
        calibration = self.srv_calibrate(TransformArray(transforms=camera_object_list), TransformArray(transforms=world_effector_list))

        print(calibration)


node = None
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Calibration', log_level=rospy.INFO)
    node = Calibration()
    node.run()
