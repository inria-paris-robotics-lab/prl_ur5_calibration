from data_2 import data
from scipy.optimize import fmin_bfgs
import pinocchio as pin

camera_transf = [0.0, 0.02, 0.15],  [-0.5, -0.5, 0.5, 0.5]
arm_transf = [-0.18, 0.0, 0.79],   [0, -0.924, 0, 0.38]

# PARSE (xyz, quat) INTO se3
def parseLine(line):
    cam_T_aruco = transf_to_se3(line[0])
    screws_T_cam = transf_to_se3(line[1])
    tool_T_shoulder = transf_to_se3(line[2])
    stand_T_base = transf_to_se3(line[3])

    return stand_T_base.inverse(), tool_T_shoulder.inverse(), screws_T_cam, cam_T_aruco

def transf_to_se3(transf):
    return pin.XYZQUATToSE3(list(transf[0]) + list(transf[1]))

# DEBUG FUNCTIONS
def print_all_subtransforms(transform_list, name_list):
    for j in range(len(transform_list)+1):
        T = transf_to_se3([[0,0,0], [0,0,0,1]])
        for i in range(j):
            T = T*transform_list[i]
        print_transform(T, name_list[j])

def print_transform(T, name):
    XYZQuat = pin.SE3ToXYZQUAT(T)
    print("")
    print("base_T_"+name)
    print("Trans : " + str(XYZQuat[:3]))
    print("Quat : " + str(XYZQuat[3:]))
    print("----")

# TEST ON FIRST EXAMPLE
base_T_stand, shoulder_T_tool, screws_T_cam, cam_T_aruco = parseLine(data[0])
tool_T_screws = transf_to_se3(camera_transf)
stand_T_shoulder = transf_to_se3(arm_transf)

transform_list = [base_T_stand, stand_T_shoulder, shoulder_T_tool, tool_T_screws, screws_T_cam, cam_T_aruco]
name_list = ["base", "stand", "shoulder", "tool", "screws", "cam", "aruco"]

print_all_subtransforms(transform_list, name_list)