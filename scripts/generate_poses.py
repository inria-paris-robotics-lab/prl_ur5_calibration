#!/usr/bin/env python
import rospy
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pickle
import rospkg
from tqdm import tqdm

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

class PoseGenerator():
    def __init__(self) -> None:
        current_path = rospkg.RosPack().get_path("prl_ur5_calibration")
        self.files_path = current_path + "/files/"

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


    def generate_samples(self):
        print("Generating samples...")

        # Read configurations
        cfg_file = open(self.files_path + "sample_config.yaml", "r")
        import yaml
        cfg = yaml.safe_load(cfg_file)
        cfg_file.close()

        # Generate angles to try
        sample_distance = cfg["distance"]
        interest_angles = [eval(angle) if isinstance(angle, str) else angle for angle in cfg["angles"]]
        sample_rpy = []
        for r in tqdm(interest_angles):
            for p in interest_angles:
                for y in interest_angles:
                    sample_rpy.append(np.array([r, p, y]))
        sample_rpy = np.array(sample_rpy)

        sample_pose = [self.rpy_to_pose(sample_distance, list(rpy)) for rpy in sample_rpy]
        pickle.dump(sample_pose, open(self.files_path + "samples.p", "wb"))
        print(F"{len(sample_pose)} samples generated.\n")

    def filter_reachable(self):
        print("Filtering reachable poses...")
        # Init the robot and planner
        from prl_hpp.ur5 import robot, planner
        planner = planner
        robot = robot

        planner.lock_grippers()
        planner.lock_right_arm()
        planner.set_velocity_limit(0.25)
        planner.set_acceleration_limit(0.25)

        # Load the sampled poses
        poses_sample = pickle.load(open(self.files_path + "samples.p", "rb"))

        def is_pose_achievable(pose):
            ''' Test if a pose is achievable by the robot (path and final pose)'''
            try:
                planner.make_gripper_approach(robot.left_gripper_name, *pose, approach_distance = 0, do_not_plan = True)
            except AssertionError as e:
                return False
            return True

        # Filter all the non reachable poses
        poses_reachable = []
        for pose in tqdm(poses_sample):
            if is_pose_achievable(pose):
                poses_reachable.append(pose)

        pickle.dump(poses_reachable, open(self.files_path + "reachables.p", "wb"))
        print(F"{len(poses_reachable)} reachable poses found from samples.\n")

    def filter_decimate(self, n):
        print("Decimate reachable poses...")
        # Load the reachable poses
        poses = pickle.load(open(self.files_path + "reachables.p", "rb"))

        if(len(poses) > n):
            pbar = tqdm(total=len(poses)-n)

            # Keep only the n poses that are the farther away from each other
            while(len(poses) > n):
                poses = remove_closest(poses, 1)
                pbar.update(1)

        pickle.dump(poses, open(self.files_path + "final.p", "wb"))
        print(F"Done decimating. {len(poses)} reachable poses kept.\n")


node = None
if __name__ == "__main__":
    rospy.init_node("generate_poses", anonymous=True)

    generate_samples = rospy.get_param("~generate_samples", True) # Generate or not the initial sampling of space
    filter_reachable = rospy.get_param("~filter_reachable", True)   # Filter the initial sampling : test is each pose can be achieved by the robot
    filter_decimate = rospy.get_param("~filter_decimate", True)     # Filter the reachable poses : keep only the n poses that are the farther away from each other

    n_poses = rospy.get_param("~n_poses", 20) # Number of final poses to generate

    node = PoseGenerator()

    if generate_samples:
        node.generate_samples()

    if filter_reachable:
        node.filter_reachable()

    if filter_decimate:
        node.filter_decimate(n_poses)
