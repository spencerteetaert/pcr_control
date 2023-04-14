import os
import glob
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

# Task space distribution 
mux = 1 # 0: CC, 1: PID 

all_xs = []
all_ys = []

trials = 10 
if mux: 
    path = '/home/spencer/Documents/thesis/pcr_control/data/PID_repetition/meta_data.txt'
else:
    path = '/home/spencer/Documents/thesis/pcr_control/data/CC_repetition/meta_data.txt'

files = glob.glob(path)

x_min, x_max, y_min, y_max = 0, 0.5, 0, 0.5
durations = [0 for _ in range(trials)]
path_lengths = [0 for _ in range(trials)]

plt.tight_layout(pad=2)
plt.ylabel('Y position [m]')
plt.xlabel('X position [m]')
plt.xlim(0, 0.5)
plt.ylim(0, 0.5)

for file in files:
    f = open(file, 'rb')
    lines = f.readlines()
    trials_to_plot = []

    counter = 0 
    for i in range(1, len(lines)):
        trial_prefix,termination_cause,duration,start_x,start_y,end_x,end_y = lines[i].decode().split(",")
        if termination_cause == 'SUCCESS':
            trials_to_plot += [os.path.join('/'.join(file.split('/')[:-1]), trial_prefix + '_aurora.txt')]
            durations[counter] += float(duration)
            counter += 1

    assert len(trials_to_plot) == trials, trials_to_plot

    for i, trial in enumerate(trials_to_plot):
        _f = open(trial, 'rb')
        _lines = _f.readlines()
        xs = []
        ys = []

        for j in range(1,len(_lines)-1):
            time,x,y,z,x_raw,y_raw,z_raw,q1,q2,q3,q4 = _lines[j].decode().split(",")
            x, y = float(x), float(y)

            # Position 
            xs += [x]
            ys += [y]
        
        plt.plot(xs, ys)

        all_xs += [xs]
        all_ys += [ys]

        tracked = np.array([xs, ys]).T
        delta = tracked[1:] - tracked[:-1]
        dist = np.linalg.norm(delta, axis=1)
        path_lengths[i] += np.sum(dist)

plt.scatter(0.15, 0.2)

print("durations", [durations[i] / trials for i in range(trials)])
print("path_lengths", [path_lengths[i] / trials for i in range(trials)])


# Calculate Fréchet distance using a Euclidean distance function between all trials. The maximum 
# distance is used as a closeness metric 
max_length = 0 
for xs in all_xs:
    if len(xs) > max_length:
        max_length = len(xs)
for i in range(len(all_xs)):
    all_xs[i] += [all_xs[i][-1] for _ in range(max_length - len(all_xs[i]))]
    all_ys[i] += [all_ys[i][-1] for _ in range(max_length - len(all_ys[i]))]

xs = np.array(all_xs)
ys = np.array(all_ys)

from scipy.spatial.distance import cdist

distances = np.zeros((len(xs), len(xs)))
for i in range(len(xs)):
    traj1 = np.vstack([xs[i], ys[i]]).T
    
    for j in range(len(xs)):
        traj2 = np.vstack([xs[j], ys[j]]).T
        distances[i, j] = np.max(np.diag(cdist(traj1, traj2, metric='euclidean')))

print("Maximal Fréchet distance:", np.max(distances))

plt.show()
input()