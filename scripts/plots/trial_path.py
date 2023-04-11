import os
import glob
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

# Task space distribution 
mux = 1 # 0: CC, 1: PID 
mux1 = 0 # 0: point, 1: traj 

if mux: 
    path = '/home/spencer/Documents/thesis/pcr_control/data/PID_test*/meta_data.txt'
else:
    path = '/home/spencer/Documents/thesis/pcr_control/data/CC_test*/meta_data.txt'

files = glob.glob(path)

ref_trajs = [
    np.load(open("/home/spencer/Documents/thesis/pcr_control/data/test_trajs/square.npy", 'rb')),
    np.load(open("/home/spencer/Documents/thesis/pcr_control/data/test_trajs/circle.npy", 'rb')),
    np.load(open("/home/spencer/Documents/thesis/pcr_control/data/test_trajs/zigzag.npy", 'rb')),
    np.load(open("/home/spencer/Documents/thesis/pcr_control/data/test_trajs/hline.npy", 'rb')),
    np.load(open("/home/spencer/Documents/thesis/pcr_control/data/test_trajs/vline.npy", 'rb')),
]

print("ref durations", [len(traj) * 0.2 for traj in ref_trajs])

dists = []
for traj in ref_trajs:
    delta = traj[1:] - traj[:-1]
    dists += [np.sum(np.linalg.norm(delta, axis=1))]

print("ref lengths", dists)


size = 500

accum = {
    'readings': 0,
}
x_min, x_max, y_min, y_max = 0, 0.5, 0, 0.5
rmses = [0 for _ in range(5)]
path_lengths = [0 for _ in range(5)]
durations = [0 for _ in range(5)]

fig, axs = plt.subplots(1, 5)

if mux: 
    fig.suptitle("PID Controller")
else:
    fig.suptitle("Differential CC Controller")

# fig.tight_layout()
fig.set_size_inches(16, 4)
axs[0].set_xlabel('X position [m]')
axs[0].set_ylabel('Y position [m]')
axs[0].set_xlim(0, 0.5)
axs[0].set_ylim(0, 0.5)
axs[1].set_xlabel('X position [m]')
# axs[1].set_ylabel('Y position [m]')
axs[1].set_xlim(0, 0.5)
axs[1].set_ylim(0, 0.5)
axs[2].set_xlabel('X position [m]')
# axs[2].set_ylabel('Y position [m]')
axs[2].set_xlim(0, 0.5)
axs[2].set_ylim(0, 0.5)
axs[3].set_xlabel('X position [m]')
# axs[3].set_ylabel('Y position [m]')
axs[3].set_xlim(0, 0.5)
axs[3].set_ylim(0, 0.5)
axs[4].set_xlabel('X position [m]')
# axs[4].set_ylabel('Y position [m]')
axs[4].set_xlim(0, 0.5)
axs[4].set_ylim(0, 0.5)

for i in range(5):
    axs[i].plot(ref_trajs[i][:,0], ref_trajs[i][:,1], label="Reference Trajectory")

for file in files:
    if mux1: 
        if '_pos' in file:
            continue
    else:
        if '_pos' not in file:
            continue
    f = open(file, 'rb')
    lines = f.readlines()
    trials_to_plot = []

    counter = 0 
    for i in range(1, len(lines)):
        trial_prefix,termination_cause,duration,start_x,start_y,end_x,end_y = lines[i].decode().split(",")
        if termination_cause == 'TRACKING_FINISHED':
            trials_to_plot += [os.path.join('/'.join(file.split('/')[:-1]), trial_prefix + '_aurora.txt')]
            durations[counter] += float(duration)
            counter += 1

    assert len(trials_to_plot) == 5, trials_to_plot

    for i, trial in enumerate(trials_to_plot):
        _f = open(trial, 'rb')
        _lines = _f.readlines()
        xs = []
        ys = []

        for j in range(1,len(_lines)-1):
            time,x,y,z,x_raw,y_raw,z_raw,q1,q2,q3,q4 = _lines[j].decode().split(",")
            x, y = float(x), float(y)
            accum['readings'] += 1

            # Position 
            xs += [x]
            ys += [y]
        
        axs[i].plot(xs, ys, label=trial.split('/')[-1])

        tracked = np.array([xs, ys]).T

        if mux1:
            rmse = (np.sum(np.linalg.norm(ref_trajs[i][:len(tracked)] - tracked, axis=1)**2) / len(tracked))**0.5
            rmses[i] += rmse


        delta = tracked[1:] - tracked[:-1]
        dist = np.linalg.norm(delta, axis=1)
        path_lengths[i] += np.sum(dist)

if not mux1:
    print("rmses", [rmses[i] for i in range(5)])
    print("path_lengths", [path_lengths[i] for i in range(5)])
    print("durations", [durations[i] for i in range(5)])
else:
    print("rmses", [rmses[i] / 3 for i in range(5)])
    print("path_lengths", [path_lengths[i] / 3 for i in range(5)])
    print("durations", [durations[i] / 3 for i in range(5)])

print(json.dumps(accum, indent=2))

fig.show()
input()