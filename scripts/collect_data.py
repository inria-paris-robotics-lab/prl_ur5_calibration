#!/usr/bin/env python

import moveit_commander
import rospy
import sys
import traceback

from math import pi

from prl_ur5_demos.utils import make_pose
from fiducial_msgs.msg import FiducialTransformArray

import tf

class Calibration():
    def __init__(self):
        # Transformations
        self.tf_listener = tf.TransformListener()

        # Initialize the robot
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        # Configure the planning pipeline
        self.robot.left_arm.set_max_velocity_scaling_factor(0.25)
        self.robot.left_arm.set_max_acceleration_scaling_factor(0.25)
        self.robot.left_arm.set_planning_time(2.0)
        self.robot.left_arm.set_planner_id('RRTstar')

        self.robot.right_arm.set_max_velocity_scaling_factor(0.25)
        self.robot.right_arm.set_max_acceleration_scaling_factor(0.25)
        self.robot.right_arm.set_planning_time(5.0)
        self.robot.right_arm.set_planner_id('RRTstar')

        self.collected_datas = []

    def go_at_pose(self, pose):
        self.set_gripper("open")

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

        return trans, rot


    def run(self):
        self.set_gripper("open")

        poses = [
                  # make_pose((-0.45, 0, 0.40), (3*pi/4., 0, pi/2.)),
                  # make_pose((-0.45, 0, 0.40), (-3*pi/4., 0, -pi/2)),
                  # make_pose((-0.45, 0, 0.40), (3*pi/4, 0, 95*pi/180.)),
                  # make_pose((-0.45, 0, 0.40), (-3*pi/4, 0, -95*pi/180.)),
                  # make_pose((-0.45, 0, 0.40), (3*pi/4, 0, 75*pi/180.)),
                  # make_pose((-0.45, 0, 0.40), (-3*pi/4, 0, -75*pi/180.)),
                  #
                  # make_pose((-0.70, 0, 0.55), (3*pi/4., 0, pi/2.)),
                  # make_pose((-0.70, 0, 0.55), (-3*pi/4., 0, -pi/2)),
                  # make_pose((-0.70, 0, 0.55), (3*pi/4, 0, 100*pi/180.)),
                  # make_pose((-0.70, 0, 0.55), (3*pi/4, 0, 70*pi/180.)),
                  # make_pose((-0.70, 0, 0.55), (-3*pi/4, 0, -70*pi/180.)),
                  # make_pose((-0.70, 0, 0.55), (-3*pi/4, 0, -100*pi/180.)),
                  #
                  # make_pose((-0.05, 0.37, 0.43), (135*pi/180., 0, 0)),
                  # make_pose((-0.05, 0.37, 0.43), (-135*pi/180., 0, 180*pi/180)),
                  # make_pose((-0.05, 0.37, 0.43), (135*pi/180., 0, -5*pi/180)),
                  # make_pose((-0.05, 0.37, 0.43), (-135*pi/180., 0, 195*pi/180)),
                  # make_pose((-0.05, 0.37, 0.43), (135*pi/180., 0, 10*pi/180)),
                  # make_pose((-0.05, 0.37, 0.43), (-135*pi/180., 0, 205*pi/180)),
                  #
                  # make_pose((0, 0, 0.70), (-pi, 0, 0)),
                  # make_pose((0, 0, 0.70), (-pi, 0, pi)),
                  # make_pose((0, 0, 0.70), (-pi, 15*pi/180., pi)),
                  # make_pose((0, 0, 0.70), (-pi, 15*pi/180., 0)),
                  # make_pose((0, 0, 0.70), (-pi, -10*pi/180., 0)),
                  # make_pose((0, 0, 0.70), (-pi, 0, 0)),
                ]

        for pose in poses:
            self.go_at_pose(pose)

            rospy.sleep(rospy.Duration(1.))

            transf_ur = self.getTransforms("/left_tool", "/left_base_link")
            transf_table = self.getTransforms("/stand_link", "/prl_ur5_base")

            success = False
            tries = 0
            while (tries < 10000 and success == False):
                tries += 1
                transf_aruco = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
                try:
                    transf_aruco = transf_aruco.transforms[0].transform
                    transf_aruco = ([transf_aruco.translation.x, transf_aruco.translation.y, transf_aruco.translation.z], [transf_aruco.rotation.x, transf_aruco.rotation.y, transf_aruco.rotation.z, transf_aruco.rotation.w])

                    self.collected_datas.append([transf_aruco, transf_ur, transf_table])
                    success = True
                except Exception:
                    # print("Aruco Failed !")
                    # print(transf_aruco)
                    # traceback.print_exc()
                    pass

            if(success):
                rospy.loginfo(f"succeed after {tries} tries")
            else:
                rospy.loginfo(f"FAILED ! after {tries} tries")

        rospy.loginfo(self.collected_datas)

node = None
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Calibration', log_level=rospy.INFO)
    node = Calibration()
    node.run()
