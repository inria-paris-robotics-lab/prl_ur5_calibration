import numpy as np
from copy import deepcopy

interest_angles = [0, np.pi/6, -np.pi/6, np.pi/4, -np.pi/4, np.pi/3, -np.pi/3, np.pi/2, -np.pi/2]
sample_rpy = []
for r in interest_angles:
    for p in interest_angles:
        for y in interest_angles:
            sample_rpy.append(np.array([r, p, y]))

readable_pos = [ 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 , 25 , 27 , 28 , 29 , 30 , 31 , 37 , 39 , 41 , 42 , 43 , 46 , 52 , 53 , 54 , 55 , 56 , 57 , 58 , 59 , 60 , 61 , 62 , 63 , 64 , 65 , 67 , 68 , 77 , 78 , 87 , 88 , 89 , 90 , 105]

sample_rpy = [sample_rpy[i] for i in readable_pos]

def dist(s1, s2):
    return (s1[0]-s2[0])**2 + (s1[1]-s2[1])**2 + (s1[2]-s2[2])**2

def dist_array(array):
    d = 0
    for i in range(len(array)-1):
        d += dist(array[i], array[i+1])
    return d

def dist_to_array_sum(s1, array):
    d = 0
    for s2 in array:
        d += dist(s1, s2)
    return d

def closest_sample(sample, array):
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
        closest = closest_sample(output[-1], samples)
        output.append(samples.pop(closest))

    return output

# Remove the sample that are the closest to everyone until the array has the desired size
while len(sample_rpy) > 25:
    distances = []
    for s in sample_rpy:
      distances.append(dist_to_array_sum(s, sample_rpy))
    i_remove = np.argmin(distances)
    sample_rpy.pop(i_remove)

# Reordre the array to have a minimu travel distance
mini = dist_array(sample_rpy)
mini_array = deepcopy(sample_rpy)

for i in range(len(sample_rpy)):
    sorted_array = sort_array(i, sample_rpy)
    d = dist_array(sorted_array)
    if d < mini:
        mini = d
        mini_array = deepcopy(sorted_array)

print(mini_array)