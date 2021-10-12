from data_2 import data
from scipy.optimize import fmin_bfgs
import tf
import pinocchio as pin

camera_transf = [0.0, 0.02, 0.15],  [-0.5, -0.5, 0.5, 0.5]
arm_transf = [-0.18, 0.0, 0.79],   [0, -0.924, 0, 0.38]

def transf_to_se3(transf):
    return pin.XYZQUATToSE3(list(transf[0]) + list(transf[1]))

meas_0 = data[0]
transf_0 = meas_0[0]

se3_aruco = transf_to_se3(meas_0[0])
se3_optical = transf_to_se3(meas_0[1])
se3_camera = transf_to_se3(camera_transf)
se3_arm = transf_to_se3(meas_0[2])
se3_stand = transf_to_se3(arm_transf)
se3_base = transf_to_se3(meas_0[3])

print(se3_base)

stand_link = se3_stand*se3_base
print(stand_link)

tool_link = stand_link.inverse() * se3_arm
print(tool_link)

camera_screw = tool_link.inverse() * se3_camera
print(camera_screw)

camera_optical = se3_optical * camera_screw.inverse()
print(camera_optical)
