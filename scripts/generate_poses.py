#!/usr/bin/env python
import rospy
import pinocchio as pin
from prl_pinocchio.tools.utils import euler_to_quaternion
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pickle
from tqdm import tqdm
import rospkg
import yaml

def sq_dist(p1, p2):
    ''' Squared distance between two points '''
    vw = pin.log6(pin.XYZQUATToSE3(p1[0]+p1[1]).inverse() * pin.XYZQUATToSE3(p2[0]+p2[1]))
    return np.linalg.norm(vw.linear)**2 + 0*np.linalg.norm(vw.angular) **2

def sq_dist_array(point, array, skip=[]):
    ''' Distances between a point and the two closest points of an array '''
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
    ''' Remove the closest point from an array of points '''
    distances = [sq_dist_array(array[i], array, [i]) for i in range(len(array))]

    idx = distances.index(sorted(distances)[0]) #argmin (over first element, then second)
    # distances.pop(idx)
    array.pop(idx)

    return array

def plot_points(points, ax = None, color = 'b', marker = '.', markersize=15):
    ''' Plot poses in 3D '''
    xx = [p[0][0] for p in points]
    yy = [p[0][1] for p in points]
    zz = [p[0][2] for p in points]

    if not ax:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

    ax.scatter(xx, yy, zz, color=color, marker=marker, s=markersize)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    return ax

def rpy_to_pose(center, l, rpy):
    ''' Convert a spherical coordinates into a cartesian pose (with the camera optical frame pointing to the orgin) '''
    # c = camera optical frame
    # t = tool frame
    # g = goal frame
    # b = base frame
    # w = world frame

    tMc = pin.XYZQUATToSE3([0.032, 0.033, -0.081, 0, 0, 1, 0]) # TODO: Do not hardcode !

    wMb = pin.XYZQUATToSE3(center["xyz"] + euler_to_quaternion(center["rpy"])) # Rotate the base frame
    bMb = pin.XYZQUATToSE3([0, 0, 0] + euler_to_quaternion(rpy)) # Rotate the base frame
    bMc = pin.XYZQUATToSE3([0,0,l, 0,1,0,0])

    wMt = wMb * bMb * bMc * tMc.inverse()

    # pose
    xyzquat_look = pin.SE3ToXYZQUAT(wMt)
    pose_look = [list(xyzquat_look[:3]), list(xyzquat_look[3:] / np.linalg.norm(xyzquat_look[3:]))]

    return pose_look

class PoseGenerator():
    def __init__(self, abs_path, cfg):
        self.abs_path = abs_path
        self.cfg = cfg

    def generate_samples(self):
        ''' Generate a set of tool poses such that the camera frame is on a sphere (centered on the origin) and pointing to the origin. '''
        rospy.loginfo("Generating samples...")

        # Generate angles to try
        sample_distance = self.cfg["generation"]["distance"]
        interest_angles = [eval(angle) if isinstance(angle, str) else angle for angle in self.cfg["generation"]["angles"]]
        sample_rpy = []
        for r in tqdm(interest_angles):
            for p in interest_angles:
                for y in interest_angles:
                    sample_rpy.append([r, p, y])
        sample_rpy = sample_rpy

        sample_pose = [rpy_to_pose(self.cfg["marker_pose"], sample_distance, rpy) for rpy in sample_rpy]
        pickle.dump(sample_pose, open(self.abs_path + self.cfg["output_paths"]["all"], "wb"))
        rospy.loginfo(F"{len(sample_pose)} samples generated.\n")

    def filter_reachable(self):
        ''' Filter all the sampled poses. Keep the one that have valid solutions (without collision). '''
        rospy.loginfo("Filtering reachable poses...")
        # Init the robot and planner
        from prl_hpp.ur5 import robot, planner
        planner.lock_grippers()
        planner.lock_right_arm()
        planner.set_velocity_limit(0.25)
        planner.set_acceleration_limit(0.25)

        # Load the sampled poses
        poses_sample = pickle.load(open(self.abs_path + self.cfg["output_paths"]["all"], "rb"))

        def is_pose_achievable(pose):
            ''' Test if a pose is achievable by the robot (path and final pose)'''
            try:
                planner.make_gripper_approach(robot.left_gripper_name, pose, approach_distance = 0, do_not_plan = True)
            except AssertionError as e:
                # rospy.logwarn(e)
                return False
            return True

        # Filter all the non reachable poses
        poses_reachable = []
        for pose in tqdm(poses_sample):
            assert not rospy.is_shutdown(), "ROS shutdown"
            if pose[0][2] < 0: # Basic filter as the camera shouldn't be below the table surface...
                continue
            if is_pose_achievable(pose):
                poses_reachable.append(pose)

        pickle.dump(poses_reachable, open(self.abs_path + self.cfg["output_paths"]["reachable"], "wb"))
        rospy.loginfo(F"{len(poses_reachable)} reachable poses found from samples.\n")

    def filter_decimate(self):
        ''' Find the n "most representative" poses (ie. the one that are the most spaced out) from the all reachable poses. '''
        rospy.loginfo("Decimate reachable poses...")
        # Load the reachable poses
        poses = pickle.load(open(self.abs_path + self.cfg["output_paths"]["reachable"], "rb"))
        n = self.cfg["n_final_poses"]

        if(len(poses) > n):
            pbar = tqdm(total=len(poses)-n)

            # Keep only the n poses that are the farther away from each other
            while(len(poses) > n):
                poses = remove_closest(poses, 1)
                pbar.update(1)

        pickle.dump(poses, open(self.abs_path + self.cfg["output_paths"]["final"], "wb"))
        rospy.loginfo(F"Done decimating. {len(poses)} reachable poses kept.\n")


node = None
if __name__ == "__main__":
    rospy.init_node("generate_poses", anonymous=True, log_level=rospy.INFO)

    # Read parameters
    config_file = rospy.get_param("~config_file")

    generate_samples = rospy.get_param("~generate_samples", True) # Generate or not the initial sampling of space
    filter_reachable = rospy.get_param("~filter_reachable", True)   # Filter the initial sampling : test is each pose can be achieved by the robot
    filter_decimate = rospy.get_param("~filter_decimate", True)     # Filter the reachable poses : keep only the n poses that are the farther away from each other

    create_plot = rospy.get_param("~plot", True)     # Plot the poses
    plot_samples = rospy.get_param("~plot_samples", True)
    plot_reachables = rospy.get_param("~plot_reachables", True)
    plot_finals = rospy.get_param("~plot_finals", True)

    abs_path = rospkg.RosPack().get_path("prl_ur5_calibration") + "/"

    # Read config file
    cfg_file = open(abs_path + config_file, "r")
    cfg = yaml.safe_load(cfg_file)
    cfg_file.close()

    # Instatiate main class
    node = PoseGenerator(abs_path, cfg)

    # Run generation steps
    if generate_samples and not rospy.is_shutdown():
        node.generate_samples()

    if filter_reachable and not rospy.is_shutdown():
        node.filter_reachable()

    if filter_decimate and not rospy.is_shutdown():
        node.filter_decimate()

    # Plot
    if create_plot and not rospy.is_shutdown():
        ax = None
        legend = []

        if plot_samples:
            poses_sample = pickle.load(open(abs_path + cfg["output_paths"]["all"], "rb"))
            ax = plot_points(poses_sample, ax=ax, color='k', marker='.', markersize=5)
            legend.append("Sampled poses")
        if plot_reachables:
            poses_reachables = pickle.load(open(abs_path + cfg["output_paths"]["reachable"], "rb"))
            ax = plot_points(poses_reachables, ax=ax, color='b', marker='2', markersize=25)
            legend.append("Reachables poses")
        if plot_finals:
            poses_final = pickle.load(open(abs_path + cfg["output_paths"]["final"], "rb"))
            ax = plot_points(poses_final, ax=ax, color='r', marker='v', markersize=30)
            legend.append("Final set")
        if ax:
            ax.legend(legend)
            ax.view_init(elev=90, azim=-180)
        plt.show(block=False)

        input("Press enter to exit...")