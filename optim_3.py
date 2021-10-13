from data_3 import data
from scipy.optimize import fmin_bfgs
import pinocchio as pin
from math import cos, sqrt
from scipy.spatial.transform import Rotation

q_arm_0 = [-0.18, 0.0,  0.79,  0,    -0.924, 0,   0.38]

# PARSE (xyz, quat) INTO se3
def parseLine(line):
    gripper_translation = line[0]
    shoulder_T_tool = transf_to_se3(line[1])
    base_T_stand = transf_to_se3(line[2])

    return base_T_stand, shoulder_T_tool, gripper_translation

def transf_to_se3(transf):
    return pin.XYZQUATToSE3(list(transf[0]) + list(transf[1]))

# COMPUTE
def error(T, translation):
    res = (T.translation[0] - translation[0]) ** 2 + \
          (T.translation[1] - translation[1]) ** 2 + \
          (T.translation[2] - translation[2]) ** 2
    return res

def normalize(quat):
    norm_sq = 0
    for c in quat:
        norm_sq += c**2
    norm = sqrt(norm_sq)
    return [c/norm for c in quat]

def x_to_q(x):
    q = list(x)
    q = q[:3] + normalize(q[3:])
    return q

def diff(a, b):
    assert len(a) == len(b), "Cannot diff different sizes"
    return [a[i] - b[i] for i in range(len(a))]


# DEBUG FUNCTIONS
def print_transform(T, name):
    XYZQuat = pin.SE3ToXYZQUAT(T)
    print("")
    print("base_T_"+name)
    print("Trans : " + str(XYZQuat[:3]))
    print("Quat : " + str(XYZQuat[3:]))
    print("----")

# # TEST ON FIRST EXAMPLE
# base_T_stand, shoulder_T_tool, screws_T_cam, cam_T_aruco = parseLine(data[0])
# tool_T_screws = transf_to_se3(camera_transf)
# stand_T_shoulder = transf_to_se3(arm_transf)

# transform_list = [base_T_stand, stand_T_shoulder, shoulder_T_tool, tool_T_screws, screws_T_cam, cam_T_aruco]
# name_list = ["base", "stand", "shoulder", "tool", "screws", "cam", "aruco"]

# T = print_all_subtransforms(transform_list, name_list)
# print(pin.log(T))
# print(error(T))

# OPTIMIZE
def cost(x):
    q_arm = x_to_q(x)

    stand_T_shoulder = pin.XYZQUATToSE3(q_arm)

    c = 0
    for d in data:
        base_T_stand, shoulder_T_tool, tool_translation = parseLine(d) 
        T = base_T_stand * stand_T_shoulder * shoulder_T_tool
        c += error(T, tool_translation)
    return c / len(data)

x0 = q_arm_0
xopt_bfgs = fmin_bfgs(cost, x0)

# Normalize quaternions
qopt_arm = x_to_q(xopt_bfgs)

print("")
print("Arm : ")
print(f"qopt_arm: \t{qopt_arm}\ndiff: \t\t{diff(qopt_arm, q_arm_0)}")

print(f"Start cost : {cost(x0)} --> Error OoM: {sqrt(cost(x0))}")
print(f"Final cost : {cost(xopt_bfgs)} --> Error OoM: {sqrt(cost(xopt_bfgs))}")

arm_rot_euler = Rotation.from_quat(qopt_arm[3:]).as_euler('XYZ')

print("")
print(f"arm translation: \t{qopt_arm[:3]}\narm rotation_euler: \t{arm_rot_euler}")