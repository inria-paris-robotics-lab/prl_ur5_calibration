from prl_ur5_diffsim2real.interface import PickEnv
import time

pickenv = PickEnv()

pickenv.reset()
error_nb = 0
time_prev = time.time()
time_list = []

for i in range(100):
    res = pickenv.step({"linear_velocity": [0, 0, 0.01], "angular_velocity": [0, 0, 0], "gripper_velocity": [0]})
    time_curr = time.time()
    time_list.append(time_curr - time_prev)
    time_prev = time_curr
    if(res[1] == False):
        error_nb += 1

print("Error nb : " + str(error_nb))
print("Time deltas : " + str(time_list))
error_nb = 0
time_prev = time.time()
time_list = []

for i in range(100):
    res = pickenv.step({"linear_velocity": [0, 0.01, 0], "angular_velocity": [0, 0, 0], "gripper_velocity": [0]})
    time_curr = time.time()
    time_list.append(time_curr - time_prev)
    time_prev = time_curr
    if(res[1] == False):
        error_nb += 1
print("Error nb : " + str(error_nb))
print("Time deltas : " + str(time_list))
error_nb = 0
time_prev = time.time()
time_list = []

for i in range(100):
    res = pickenv.step({"linear_velocity": [0.01, 0, 0], "angular_velocity": [0, 0, 0], "gripper_velocity": [0]})
    time_curr = time.time()
    time_list.append(time_curr - time_prev)
    time_prev = time_curr
    if(res[1] == False):
        error_nb += 1
print("Error nb : " + str(error_nb))
print("Time deltas : " + str(time_list))
error_nb = 0
time_prev = time.time()
time_list = []
