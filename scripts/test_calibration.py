#!/usr/bin/env python
import moveit_commander
import rospy
import sys
from prl_ur5_demos.utils import make_pose
from math import pi

rospy.init_node("test_calibration")
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface(synchronous=True)

robot.left_arm.set_max_velocity_scaling_factor(0.25)
robot.left_arm.set_max_acceleration_scaling_factor(0.25)
robot.left_arm.set_planning_time(2.0)
robot.left_arm.set_planner_id('RRTstar')

robot.left_arm.set_end_effector_link("left_gripper_grasp_frame")


cube_height = 0.15
offset = 0.015
cube_height += offset

pose_list = [
    make_pose([ -0.695 ,  0    ,cube_height], (pi, 0, 0)),
    make_pose([ -0.695 ,  0    ,cube_height], (pi, 0, pi/2)),
    make_pose([ -0.695 ,  -0.175 ,cube_height], (pi, 0, 0)),
    make_pose([ -0.695 ,  -0.175 ,cube_height], (pi, 0, pi/2)),
    make_pose([-0.295,  -0.175 ,cube_height], (pi, 0, 0)),
    make_pose([-0.295,  -0.175 ,cube_height], (pi, 0, pi/2)),
    make_pose([-0.295,  0    ,cube_height], (pi, 0, 0)),
    make_pose([-0.295,  0    ,cube_height], (pi, 0, pi/2)),
    make_pose([-0.295, 0.175 ,cube_height], (pi, 0, 0)),
    make_pose([-0.295, 0.175 ,cube_height], (pi, 0, pi/2)),
]

for pose in pose_list:
    if rospy.is_shutdown():
        break

    # try cartesian
    path, fraction = robot.left_arm.compute_cartesian_path([pose], eef_step=0.01, jump_threshold=10.0)
    if fraction >= 1.0:
        robot.left_arm.execute(path, wait=True)
    else:
        # Use RRT
        robot.left_arm.set_pose_target(pose)
        robot.left_arm.go(wait=True)

    rospy.logwarn("Press enter to go to next pose")
    input("Press enter to go to next pose")