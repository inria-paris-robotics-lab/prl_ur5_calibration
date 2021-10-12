from data_2 import data
from scipy.optimize import fmin_bfgs
import pinocchio as pin
from math import cos, sqrt
from scipy.spatial.transform import Rotation

q_cam_0 = [0.0,   0.02, 0.15,  -0.5, -0.5,   0.5, 0.5]
q_arm_0 = [-0.18, 0.0,  0.79,  0,    -0.924, 0,   0.38]

# PARSE (xyz, quat) INTO se3
def parseLine(line):
    cam_T_aruco = transf_to_se3(line[0])
    screws_T_cam = transf_to_se3(line[1])
    tool_T_shoulder = transf_to_se3(line[2])
    stand_T_base = transf_to_se3(line[3])

    return stand_T_base.inverse(), tool_T_shoulder.inverse(), screws_T_cam, cam_T_aruco

def transf_to_se3(transf):
    return pin.XYZQUATToSE3(list(transf[0]) + list(transf[1]))

# COMPUTE
def error(T):
    v = pin.log(T).vector
    return (v**2).sum()

def normalize(quat):
    norm_sq = 0
    for c in quat:
        norm_sq += c**2
    norm = sqrt(norm_sq)
    return [c/norm for c in quat]

def x_to_qs(x):
    q1 = list(x)[:7]
    q2 = list(x)[7:]

    q1 = q1[:3] + normalize(q1[3:])
    q2 = q2[:3] + normalize(q2[3:])
    
    return q1, q2

def diff(a, b):
    assert len(a) == len(b), "Cannot diff different sizes"
    return [a[i] - b[i] for i in range(len(a))]


# DEBUG FUNCTIONS
def print_all_subtransforms(transform_list, name_list):
    T = None
    for j in range(len(transform_list)+1):
        T = transf_to_se3([[0,0,0], [0,0,0,1]])
        for i in range(j):
            T = T*transform_list[i]
        print_transform(T, name_list[j])
    return T # return the last Transform

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
    q_cam, q_arm = x_to_qs(x)

    tool_T_screws = pin.XYZQUATToSE3(q_cam)
    stand_T_shoulder = pin.XYZQUATToSE3(q_arm)

    c = 0
    for d in data:
        base_T_stand, shoulder_T_tool, screws_T_cam, cam_T_aruco = parseLine(d) 
        T = base_T_stand * stand_T_shoulder * shoulder_T_tool * tool_T_screws * screws_T_cam * cam_T_aruco
        
        ''' THIS IS A TEMPORARY FIX FOR THIS DATA '''
        T = T*pin.XYZQUATToSE3([0,0,0, 0, 0, 0.707, 0.707])
        '''~THIS IS A TEMPORARY FIX FOR THIS DATA~'''
        
        c += error(T)
    return c / len(data)

x0 = q_cam_0 + q_arm_0

xopt_bfgs = fmin_bfgs(cost, x0)

# Normalize quaternions
qopt_cam, qopt_arm = x_to_qs(xopt_bfgs)

print("")
print("Camera : ")
print(f"qopt_cam: \t{qopt_cam}\ndiff: \t\t{diff(qopt_cam, q_cam_0)}")
print("Arm : ")
print(f"qopt_arm: \t{qopt_arm}\ndiff: \t\t{diff(qopt_arm, q_arm_0)}")

print(f"Start cost : {cost(x0)}")
print(f"Final cost : {cost(xopt_bfgs)}")

cam_rot_euler = Rotation.from_quat(qopt_cam[3:]).as_euler('XYZ') #Rotation.from_quat(qopt_cam[3:]).as_euler('xyz')
arm_rot_euler = Rotation.from_quat(qopt_arm[3:]).as_euler('XYZ')

print("")
print(f"cam translation: \t{qopt_cam[:3]}\ncam rotation_euler: \t{cam_rot_euler}")
print(f"arm translation: \t{qopt_arm[:3]}\narm rotation_euler: \t{arm_rot_euler}")