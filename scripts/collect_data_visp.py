#!/usr/bin/env python
import rospy
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

def sq_dist(p1, p2):
    '''Squared distance between two points'''
    vw = pin.log6(pin.XYZQUATToSE3(p1[0]+p1[1]).inverse() * pin.XYZQUATToSE3(p2[0]+p2[1]))
    return np.linalg.norm(vw.linear)**2 + 0*np.linalg.norm(vw.angular) **2

def sq_dist_array(point, array, skip=[]):
    '''Distances between a point and the two closest points of an array'''
    mini_d = float('inf')
    mini_d_2 = float('inf')
    for i, p2 in enumerate(array):
        if i in skip:
            continue
        d = sq_dist(point, p2)
        if(d < mini_d):
            mini_d_2 = mini_d
            mini_d = d
    return mini_d, mini_d_2


def remove_closest(array, k):
    '''Remove the closest point from an array of points'''
    distances = [sq_dist_array(array[i], array, [i]) for i in range(len(array))]

    idx = distances.index(sorted(distances)[0]) #argmin (over first element, then second)
    # distances.pop(idx)
    array.pop(idx)

    return array

def plot_points(points, ax = None, color = 'b', marker = '.'):
    xx = [p[0][0] for p in points]
    yy = [p[0][1] for p in points]
    zz = [p[0][2] for p in points]

    if not ax:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

    ax.scatter(xx, yy, zz, color=color, marker=marker)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    return ax

class Calibration():
    def __init__(self):
        from prl_hpp.ur5 import robot, planner
        self.planner = planner
        self.robot = robot

        self.planner.lock_grippers()
        self.planner.lock_right_arm()
        self.planner.set_velocity_limit(0.25)
        self.planner.set_acceleration_limit(0.25)

    def rpy_to_pose(self, l, rpy):
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
        pose_look = [list(xyzquat_look[:3]), list(xyzquat_look[3:])]

        return pose_look

    def is_pose_achievable(self, pose):
        ''' Test if a pose is achievable by the robot (path and final pose)'''
        try:
            self.planner.make_gripper_approach(self.robot.left_gripper_name, *pose, approach_distance = 0, do_not_plan = True)
        except AssertionError as e:
            # print(e)
            return False
        return True

    def run(self):
        # self.set_gripper("open")

        # Generate angles to try
        # sample_distance = 0.45
        # interest_angles = [0, np.pi/6, -np.pi/6, np.pi/4, -np.pi/4, np.pi/3, -np.pi/3, np.pi/2, -np.pi/2]
        # sample_rpy = []
        # for r in interest_angles:
        #     for p in interest_angles:
        #         for y in interest_angles:
        #             sample_rpy.append(np.array([r, p, y]))
        # sample_rpy = np.array(sample_rpy)

        # sample_pose = [self.rpy_to_pose(sample_distance, list(rpy)) for rpy in sample_rpy]
        # del(sample_rpy)

        # # Filter all the non reachable poses
        # sample_pose = list(filter(lambda p: self.is_pose_achievable(p), sample_pose))
        # print("REACHABLE")
        # print(sample_pose)

        sys.path.append('/home/earlaud/Documents/igors_demo/src/prl_ur5_calibration/scripts')
        from reachable import sample_pose
        ax = plot_points(sample_pose)

        # Keep only the n poses that are the farther away from each other
        n = 20
        while(len(sample_pose) > n):
            sample_pose = remove_closest(sample_pose, 1)

        print("k FAREST")
        print(sample_pose)

        plot_points(sample_pose, ax, 'r', 'o')

        plt.show(block=False)

        camera_object_list = []
        world_effector_list = []

        # camera_object = self.get_marker()
        # if not camera_object:
        #     continue
        # world_effector = self.getTransforms("/left_base_link", "/left_tool")
        # camera_object_list.append(camera_object)
        # world_effector_list.append(world_effector)

        # calibration = self.srv_calibrate(TransformArray(transforms=camera_object_list), TransformArray(transforms=world_effector_list))
        # print(calibration)



node = None
if __name__ == "__main__":
    node = Calibration()
    node.run()
