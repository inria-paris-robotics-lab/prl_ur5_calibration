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
        self.robot.left_arm.set_planning_time(10)
        self.robot.left_arm.set_planner_id('RRTstar')
        self.robot.left_arm.set_end_effector_link('left_gripper_grasp_frame')

        # Get table object
        self.table_object = self.scene.get_objects()['table']

        # Some constant geometry
        self.GRIPPER_GRASP_F_HEIGHT = 0.022

        self.collected_datas = []

    def enable_table(self, enable):
        if(enable):
            self.scene.add_object(self.table_object)
        else:
            self.scene.remove_world_object('table')

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

        return trans, rot


    def calibrate_on_point(self, goal_x, goal_y, goal_z, x_offset=0, y_offset=0, z_offset=0):
        x, y, z = goal_x + x_offset, goal_y + y_offset, goal_z + z_offset

        # Go at rough pose
        pre_goal_pose = make_pose([x, y, z+0.1], [pi, 0, 0])
        goal_pose = make_pose([x, y, z], [pi, 0, 0])
        
        self.enable_table(True)
        self.go_at_pose(pre_goal_pose)
        self.go_at_pose(goal_pose)

        # Fine adjust with keyboard
        flipped = False
        stop = False
        self.enable_table(False)
        while not rospy.is_shutdown() and not stop:
            key = input("> ")

            if key == '2':
                z -= 0.001
            elif key == '8':
                z += 0.001
            elif key == '6' and not flipped:
                x -= 0.001
            elif key == '4' and not flipped:
                x += 0.001
            elif key == '6' and flipped:
                y -= 0.001
            elif key == '4' and flipped:
                y += 0.001
            elif key == '5':
                flipped = not flipped
            elif key == '.':
                stop = True

            pose = None
            if not flipped:
                pose = make_pose([x, y, z], [pi, 0, 0])
            else:
                pose = make_pose([x, y, z], [pi, 0, pi/2])
            self.go_cartesian_wps([pose])

        # Print the result
        res = [
            [goal_x, goal_y, goal_z],
            self.getTransforms("/left_base_link", "/left_gripper_grasp_frame"),
            self.getTransforms("/prl_ur5_base", "/stand_link")
        ]

        # Re-set the planning
        self.go_cartesian_wps([goal_pose])
        self.enable_table(True)

        print(res)
        return res


    def run(self):
        self.set_gripper("close")

        rospy.loginfo("4 8 6 2 keys to move - 5 to rotate - . to go to next point")

        self.collected_datas.append(self.calibrate_on_point(0, -0.135   , self.GRIPPER_GRASP_F_HEIGHT, z_offset=0.005))
        self.collected_datas.append(self.calibrate_on_point(0    , 0   , self.GRIPPER_GRASP_F_HEIGHT, z_offset=0.005))
        self.collected_datas.append(self.calibrate_on_point(-0.45, 0   , self.GRIPPER_GRASP_F_HEIGHT, z_offset=0.005))
        self.collected_datas.append(self.calibrate_on_point(-0.45, -0.27   , self.GRIPPER_GRASP_F_HEIGHT, z_offset=0.005))
        self.collected_datas.append(self.calibrate_on_point(-0.90, -0.27   , self.GRIPPER_GRASP_F_HEIGHT, z_offset=0.005))
        self.collected_datas.append(self.calibrate_on_point(-0.90, 0   , self.GRIPPER_GRASP_F_HEIGHT, z_offset=0.005))
        self.collected_datas.append(self.calibrate_on_point(-0.90, 0.135   , self.GRIPPER_GRASP_F_HEIGHT, z_offset=0.005))
        self.collected_datas.append(self.calibrate_on_point(-0.45, 0.27, self.GRIPPER_GRASP_F_HEIGHT, z_offset=0.005))
        self.collected_datas.append(self.calibrate_on_point(0    , 0.27, self.GRIPPER_GRASP_F_HEIGHT, z_offset=0.005))
        self.collected_datas.append(self.calibrate_on_point(0.45 , 0   , self.GRIPPER_GRASP_F_HEIGHT, z_offset=0.005))


        rospy.loginfo("Collected data : ")
        rospy.loginfo(self.collected_datas)

node = None
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Calibration', log_level=rospy.INFO)
    node = Calibration()
    node.run()
