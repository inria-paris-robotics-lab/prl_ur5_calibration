#!/usr/bin/env python

import moveit_commander
import rospy
import sys

from prl_ur5_demos.utils import make_pose
import pinocchio as pin
import numpy as np
import rospkg

from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Transform, Vector3, Quaternion
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
        self.robot.left_arm.set_planning_time(2)
        self.robot.left_arm.set_planner_id('RRTstar')

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
        left_path, left_fraction = self.robot.left_arm.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=10.0)
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

    def get_aruco(self):
        for _ in range(10):
            transf_aruco = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
            if(len(transf_aruco.transforms) > 0):
                return transf_aruco.transforms[0].transform
            rospy.sleep(0.01)
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
        sample_rpy = np.array([[1.4564606963470341, -0.7441119173464478, -1.4654673942399523],
                                [1.3129474678024469, -0.9028080689374369, -1.3515655277445409],
                                [0.27758748, -0.60845145, -1.3089007],
                                [0.0192312, -0.10345522, -1.06005948],
                                [0.08778395, 0.16220219, -1.24213999],
                                [-0.50674563, -0.4712473, -1.01117966],
                                [0.09749526810290529, -0.4381084130984383, -0.09447612057178745],
                                [0.41483697, -0.82465037, 0.08239443],
                                [-0.3186498376634783, -1.1543141698475334, 0.005334004985295956],
                                [-0.3719995160649501, -0.5289335061783618, 0.24522930535547574],
                                [-0.17198890185481117, -0.367641954115124, 1.0445485851469207],
                                [-0.02858335, -0.32575534, 1.07704686],
                                [-0.04557244, -0.74919633, 1.18554096],
                                [-0.7286450525123809, -1.173168560457098, 1.2057157334973343],
                                [-0.90689106, -1.05620302, 1.14180119],
                                [-0.6227868337984509, 0.020915589351595123, 0.8707877563046359],
                                [-0.41170385, 0.27149211, -0.02671961],
                                [0.10062518, 0.59164302, -0.03274805]
                                ])

        camera_object_list = []
        world_effector_list = []

        for i, angles in enumerate(sample_rpy):
            success = self.go_to_rpy(sample_distance, list(angles))
            if not success:
                continue

            rospy.sleep(0.5)

            camera_object = self.get_aruco()
            if not camera_object:
                continue

            world_effector = self.getTransforms("/left_base_link", "/left_tool")

            camera_object_list.append(camera_object)
            world_effector_list.append(world_effector)

        calibration = self.srv_calibrate(TransformArray(transforms=camera_object_list), TransformArray(transforms=world_effector_list))

        print(calibration)


node = None
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Calibration', log_level=rospy.INFO)
    node = Calibration()
    node.run()
