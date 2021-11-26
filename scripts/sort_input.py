import numpy as np
from copy import deepcopy

samples = [[0.10062518, 0.59164302, -0.03274805],
            [0.08778395, 0.16220219, -1.24213999],
            [-0.90689106, -1.05620302, 1.14180119],
            [-0.50674563, -0.4712473, -1.01117966],
            [-0.41170385, 0.27149211, -0.02671961],
            [0.41483697, -0.82465037, 0.08239443],
            [-0.04557244, -0.74919633, 1.18554096],
            [0.27758748, -0.60845145, -1.3089007],
            [-0.02858335, -0.32575534, 1.07704686],
            [0.0192312, -0.10345522, -1.06005948],
            [-0.3186498376634783, -1.1543141698475334, 0.005334004985295956],
            [-0.17198890185481117, -0.367641954115124, 1.0445485851469207],
            [-0.3719995160649501, -0.5289335061783618, 0.24522930535547574],
            [-0.6227868337984509, 0.020915589351595123, 0.8707877563046359],
            [1.3129474678024469, -0.9028080689374369, -1.3515655277445409],
            [0.09749526810290529, -0.4381084130984383, -0.09447612057178745],
            [1.4564606963470341, -0.7441119173464478, -1.4654673942399523],
            [-0.7286450525123809, -1.173168560457098, 1.2057157334973343]]

def dist(s1, s2):
    return (s1[0]-s2[0])**2 + (s1[1]-s2[1])**2 + (s1[2]-s2[2])**2

def dist_array(array):
    d = 0
    for i in range(len(array)-1):
        d += dist(array[i], array[i+1])
    return d

def min_sample(sample, array):
    mini = dist(sample, array[0])
    argmini = 0
    for i in range(1,len(array)):
        d = dist(sample, array[i])
        if(d < mini):
            mini = d
            argmini = i
    return argmini


def sort_array(start_index, samples):
    samples = deepcopy(samples)

    # first one
    output = [samples.pop(start_index)]

    for _ in range(len(samples)):
        closest = min_sample(output[-1], samples)
        output.append(samples.pop(closest))

    return output

mini = dist_array(samples)
mini_array = deepcopy(samples)

for i in range(len(samples)):
    sorted_array = sort_array(i, samples)
    d = dist_array(sorted_array)
    if d < mini:
        mini = d
        mini_array = deepcopy(sorted_array)
